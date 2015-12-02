/*
 * Copyright (C) 2015  T+A elektroakustik GmbH & Co. KG
 *
 * This file is part of DCPSPI.
 *
 * DCPSPI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 3 as
 * published by the Free Software Foundation.
 *
 * DCPSPI is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DCPSPI.  If not, see <http://www.gnu.org/licenses/>.
 */

#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <errno.h>

#include "spi.h"
#include "named_pipe.h"
#include "gpio.h"
#include "dcpdefs.h"
#include "messages.h"
#include "os.h"
#include "versioninfo.h"

/*!
 * Current state of the DCP transaction.
 */
enum transaction_state
{
    TR_IDLE = 0,                                    /*!< Idle, waiting for activities. */

    /* master transactions */
    TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD,  /*!< Reading header from DCP process. */
    TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD,    /*!< Reading data from DCP process. */
    TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE,         /*!< Sending write request to slave over SPI. */

    /* slave transactions */
    TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE,       /*!< Reading header from slave over SPI. */

    /* for slave write commands */
    TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE,    /*!< Reading data from slave over SPI. */
    TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD,           /*!< Sending write request to DCP process. */

    /* for slave read commands */
    TR_SLAVE_READCMD_FORWARDING_TO_DCPD,            /*!< Sending read request to DCP process. */
    TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD,    /*!< Reading header from DCP process. */
    TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD,      /*!< Reading data from DCP process. */
    TR_SLAVE_READCMD_FORWARDING_TO_SLAVE,           /*!< Sending answer to slave over SPI. */

    /* any slave request */
    TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT,             /*!< Wait for slave to deassert request. */
};

/*!
 * Some buffer and a simple embedded iterator.
 */
struct buffer
{
    uint8_t *buffer;
    size_t size;
    size_t pos;
};

enum slave_request_line_state_t
{
    REQ_NOT_REQUESTED = 0,
    REQ_ASSERTED,
    REQ_DEASSERTED,
};

/*!
 * State of the DCP transaction in progress.
 */
struct dcp_transaction
{
    enum transaction_state state;

    struct buffer dcp_buffer;
    struct buffer spi_buffer;

    uint16_t pending_size_of_transaction;
    size_t flush_to_dcpd_buffer_pos;
    bool pending_escape_sequence_in_spi_buffer;

    enum slave_request_line_state_t request_state;
};

struct collision_check_data
{
    const struct gpio_handle *const gpio;
    const struct dcp_transaction *const current_transaction;
};

static void show_version_info(void)
{
    printf("%s\n"
           "Revision %s%s\n"
           "         %s+%d, %s\n",
           PACKAGE_STRING,
           VCS_FULL_HASH, VCS_WC_MODIFIED ? " (tainted)" : "",
           VCS_TAG, VCS_TICK, VCS_DATE);
}

static void log_version_info(void)
{
    msg_info("Rev %s%s, %s+%d, %s",
             VCS_FULL_HASH, VCS_WC_MODIFIED ? " (tainted)" : "",
             VCS_TAG, VCS_TICK, VCS_DATE);
}

/*!
 * Whether or not the DCP process is allowed to send any data.
 */
static bool expecting_dcp_data(const struct dcp_transaction *transaction)
{
    return (transaction->state == TR_IDLE ||
            transaction->state == TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD ||
            transaction->state == TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD ||
            transaction->state == TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD ||
            transaction->state == TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD);
}

/*!
 * Global flag that gets cleared in the SIGTERM signal handler.
 *
 * For clean shutdown.
 */
static volatile bool keep_running = true;

static void clear_buffer(struct buffer *buffer)
{
    buffer->pos = 0;
}

static void swap_buffers(struct buffer *const first,
                         struct buffer *const second)
{
    const struct buffer temp = *first;

    *first = *second;
    *second = temp;
}

static bool is_buffer_full(const struct buffer *buffer)
{
    return buffer->pos >= buffer->size;
}

static int fill_buffer_from_fd(struct buffer *buffer, size_t count, int fd)
{
    if(count == 0)
        return 0;

    ssize_t len = read(fd, buffer->buffer + buffer->pos, count);

    if(len > 0)
    {
        count -= len;
        buffer->pos += len;
    }
    else if(len == 0)
    {
        msg_error(errno, LOG_NOTICE,
                  "Premature end of input on named pipe %d", fd);
    }
    else if(errno != EINTR && errno != EAGAIN)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed reading %zu bytes from fd %d", count, fd);
        return -1;
    }

    return len;
}

static ssize_t send_buffer_to_fd(struct buffer *buffer,
                                 size_t offset, size_t count, int fd)
{
    errno = 0;

    ssize_t len = write(fd, buffer->buffer + offset, count);

    if(len > 0)
        return len;

    msg_error(errno, LOG_EMERG,
              "Failed writing %zu bytes to fd %d (write() returned %zd)",
              count, fd, len);
    return len < 0 ? -1 : 0;
}

static void reset_transaction_struct(struct dcp_transaction *transaction)
{
    msg_info("## Reset transaction");

    transaction->state = TR_IDLE;

    clear_buffer(&transaction->dcp_buffer);
    transaction->pending_size_of_transaction = 0;
    transaction->flush_to_dcpd_buffer_pos = 0;

    clear_buffer(&transaction->spi_buffer);

    transaction->request_state = REQ_NOT_REQUESTED;
}

static void reset_transaction(struct dcp_transaction *transaction)
{
    switch(transaction->state)
    {
      case TR_IDLE:
      case TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD:
      case TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE:
        break;

      case TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE:
      case TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE:
      case TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD:
      case TR_SLAVE_READCMD_FORWARDING_TO_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_READCMD_FORWARDING_TO_SLAVE:
      case TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT:
        if(transaction->request_state == REQ_ASSERTED)
        {
            msg_info("End of transaction, waiting for slave to release request line");
            transaction->state = TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT;
            return;
        }

        break;
    }

    reset_transaction_struct(transaction);
}

static uint8_t get_dcp_command_type(uint8_t dcp_header_first_byte)
{
    return dcp_header_first_byte & 0x0f;
}

static bool is_read_command(const uint8_t *dcp_header)
{
    const uint8_t command_type = get_dcp_command_type(dcp_header[0]);

    return (command_type == DCP_COMMAND_READ_REGISTER ||
            command_type == DCP_COMMAND_MULTI_READ_REGISTER);
}

static uint16_t get_dcp_data_size(const uint8_t *dcp_header)
{
    const uint8_t command_type = get_dcp_command_type(dcp_header[0]);

    if(command_type == DCP_COMMAND_MULTI_WRITE_REGISTER ||
       command_type == DCP_COMMAND_MULTI_READ_REGISTER)
        return dcp_header[2] | (dcp_header[3] << 8);
    else
        return 0;
}

static size_t compute_read_size(const struct dcp_transaction *transaction)
{
    size_t read_size =
        transaction->dcp_buffer.size - transaction->dcp_buffer.pos;

    if(read_size > transaction->pending_size_of_transaction)
        read_size = transaction->pending_size_of_transaction;

    return read_size;
}

static const char *tr_log_prefix(enum transaction_state state)
{
    switch(state)
    {
      case TR_IDLE:
        return "No transaction";

      case TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD:
      case TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE:
        return "Master write transaction";

      case TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE:
      case TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT:
        return "Slave transaction";

      case TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE:
      case TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD:
        return "Slave write transaction";

      case TR_SLAVE_READCMD_FORWARDING_TO_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_READCMD_FORWARDING_TO_SLAVE:
        return "Slave read transaction";
    }

    return "INVALID transaction";
}

static void process_transaction_receive_data(struct dcp_transaction *transaction,
                                             int fifo_in_fd, int spi_fd,
                                             unsigned int spi_timeout_ms)
{
    const char *read_peer =
        (transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE) ? "slave" : "DCPD";
    const char *write_peer =
        (transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE) ? "DCPD" : "slave";

    msg_info("%s: need to receive %u bytes from %s",
             tr_log_prefix(transaction->state),
             transaction->pending_size_of_transaction, read_peer);

    const size_t read_size = compute_read_size(transaction);
    const int bytes_read =
        (transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE)
        ? spi_read_buffer(spi_fd,
                          transaction->dcp_buffer.buffer + transaction->dcp_buffer.pos,
                          read_size, spi_timeout_ms)
        : fill_buffer_from_fd(&transaction->dcp_buffer, read_size, fifo_in_fd);

    if(bytes_read < 0)
    {
        msg_error(0, LOG_ERR, "%s: communication with %s broken",
                  tr_log_prefix(transaction->state), read_peer);
        reset_transaction(transaction);
        return;
    }

    if(transaction->state == TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE)
        transaction->dcp_buffer.pos += bytes_read;

    transaction->pending_size_of_transaction -= (size_t)bytes_read;
    msg_info("%s: still pending %u", tr_log_prefix(transaction->state),
             transaction->pending_size_of_transaction);

    if(transaction->pending_size_of_transaction == 0 ||
       is_buffer_full(&transaction->dcp_buffer))
    {
        msg_info("%s: flush buffer to %s", tr_log_prefix(transaction->state), write_peer);
        transaction->flush_to_dcpd_buffer_pos = 0;
        if(transaction->state == TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD)
            transaction->state = TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE;
        else if(transaction->state == TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD)
            transaction->state = TR_SLAVE_READCMD_FORWARDING_TO_SLAVE;
        else
            transaction->state = TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD;
    }
}

static bool check_slave_request_collision(void *data)
{
    const struct collision_check_data *const check_data = data;

    return gpio_is_active(check_data->gpio);
}

static void process_transaction(struct dcp_transaction *transaction,
                                struct buffer *deferred_transaction_data,
                                int fifo_in_fd, int fifo_out_fd,
                                int spi_fd, unsigned int spi_timeout_ms,
                                struct collision_check_data *ccdata)
{
    switch(transaction->state)
    {
      case TR_IDLE:
        switch(transaction->request_state)
        {
          case REQ_NOT_REQUESTED:
            transaction->state = TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD;
            msg_info("%s: begin (assuming write command)", tr_log_prefix(transaction->state));
            break;

          case REQ_ASSERTED:
            transaction->state = TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE;
            msg_info("%s: begin", tr_log_prefix(transaction->state));
            spi_new_transaction();
            return;

          case REQ_DEASSERTED:
            msg_info("No transaction to process");
            reset_transaction_struct(transaction);
            return;
        }

        /* fall-through */

      case TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD:
        msg_info("%s: receiving command header from DCPD", tr_log_prefix(transaction->state));
        if(fill_buffer_from_fd(&transaction->dcp_buffer,
                               DCP_HEADER_SIZE - transaction->dcp_buffer.pos,
                               fifo_in_fd) < 0)
        {
            msg_error(0, LOG_ERR, "%s: communication with DCPD broken",
                      tr_log_prefix(transaction->state));
            reset_transaction(transaction);
            break;
        }

        if(transaction->dcp_buffer.pos != DCP_HEADER_SIZE)
        {
            msg_info("%s: header incomplete, waiting for more input",
                     tr_log_prefix(transaction->state));
            break;
        }

        /*
         * Possibly received a DCP header. Note that we explictly do not
         * validate the header content here because this is going to be done by
         * the receiver of the data. We simply assume that we have a header
         * here and that the length can be determined.
         */
        msg_info("%s: command header received: 0x%02x 0x%02x 0x%02x 0x%02x",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.buffer[0],
                 transaction->dcp_buffer.buffer[1],
                 transaction->dcp_buffer.buffer[2],
                 transaction->dcp_buffer.buffer[3]);

        transaction->pending_size_of_transaction =
            get_dcp_data_size(transaction->dcp_buffer.buffer);

        if(transaction->pending_size_of_transaction > 0)
        {
            if(transaction->state == TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD)
                transaction->state = TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD;
            else
                transaction->state = TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD;
            break;
        }

        if(transaction->state == TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD)
            transaction->state = TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE;
        else
            transaction->state = TR_SLAVE_READCMD_FORWARDING_TO_SLAVE;

        /* fall-through */

      case TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE:
      case TR_SLAVE_READCMD_FORWARDING_TO_SLAVE:
        log_assert(transaction->pending_size_of_transaction == 0);

        clear_buffer(&transaction->spi_buffer);
        transaction->spi_buffer.pos =
            spi_fill_buffer_from_raw_data(transaction->spi_buffer.buffer,
                                          transaction->spi_buffer.size,
                                          transaction->dcp_buffer.buffer,
                                          transaction->dcp_buffer.pos);

        msg_info("%s: send %zu bytes over SPI (were %zu bytes)",
                 tr_log_prefix(transaction->state),
                 transaction->spi_buffer.pos, transaction->dcp_buffer.pos);
        int ret =
            spi_send_buffer(spi_fd, transaction->spi_buffer.buffer,
                            transaction->spi_buffer.pos, spi_timeout_ms,
                            check_slave_request_collision, ccdata);

        if(ret < 0)
        {
            /* hard failure; abort transaction */
            reset_transaction(transaction);
            break;
        }
        else if(ret == 1)
        {
            if(transaction->state == TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE)
            {
                /* collision; process slave request first, reschedule master
                 * transaction when the slave request is done */
                if(deferred_transaction_data->pos > 0)
                    BUG("Lost deferred transaction");

                swap_buffers(deferred_transaction_data, &transaction->dcp_buffer);
                transaction->state = TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE;

                break;
            }
            else
            {
                /* slave didn't wait long enough for our answer (or we were
                 * just very slow due to an unlucky schedule in an overloaded
                 * system); in this state, we already got the complete answer
                 * to the ongoing transaction from DCPD, and we can throw it
                 * away because the slave is not interested anymore */
                msg_info("%s: slave interrupted active transaction with new "
                         "request, dropping answer to previous request",
                         tr_log_prefix(transaction->state));;
            }
        }

        msg_info("%s: DONE", tr_log_prefix(transaction->state));
        reset_transaction(transaction);
        break;

      case TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE:
        msg_info("%s: receiving command header from slave", tr_log_prefix(transaction->state));
        if(spi_read_buffer(spi_fd, transaction->dcp_buffer.buffer,
                           DCP_HEADER_SIZE, spi_timeout_ms) < DCP_HEADER_SIZE)
        {
            reset_transaction(transaction);
            break;
        }

        transaction->dcp_buffer.pos = DCP_HEADER_SIZE;

        msg_info("%s: command header received: 0x%02x 0x%02x 0x%02x 0x%02x",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.buffer[0],
                 transaction->dcp_buffer.buffer[1],
                 transaction->dcp_buffer.buffer[2],
                 transaction->dcp_buffer.buffer[3]);

        transaction->pending_size_of_transaction =
            get_dcp_data_size(transaction->dcp_buffer.buffer);

        if(transaction->pending_size_of_transaction >
           (transaction->dcp_buffer.size - DCP_HEADER_SIZE))
        {
            msg_error(EINVAL, LOG_ERR,
                      "%s: transaction size %u exceeds maximum size of %zu",
                      tr_log_prefix(transaction->state),
                      transaction->pending_size_of_transaction,
                      transaction->dcp_buffer.size - DCP_HEADER_SIZE);
            reset_transaction(transaction);
            break;
        }

        if(transaction->pending_size_of_transaction > 0)
        {
            transaction->state = TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE;
            break;
        }

        transaction->pending_size_of_transaction = DCP_HEADER_SIZE;
        transaction->state = (is_read_command(transaction->dcp_buffer.buffer)
                              ? TR_SLAVE_READCMD_FORWARDING_TO_DCPD
                              : TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD);
        break;

      case TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD:
      case TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE:
        process_transaction_receive_data(transaction, fifo_in_fd, spi_fd,
                                         spi_timeout_ms);
        break;

      case TR_SLAVE_READCMD_FORWARDING_TO_DCPD:
      case TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD:
        msg_info("%s: send %zu bytes answer to DCPD (%zu pending)",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.pos,
                 transaction->dcp_buffer.pos - transaction->flush_to_dcpd_buffer_pos);

        ssize_t sent_bytes =
            send_buffer_to_fd(&transaction->dcp_buffer,
                              transaction->flush_to_dcpd_buffer_pos,
                              transaction->dcp_buffer.pos - transaction->flush_to_dcpd_buffer_pos,
                              fifo_out_fd);

        if(sent_bytes < 0)
        {
            msg_error(0, LOG_ERR, "%s: communication with DCPD broken",
                      tr_log_prefix(transaction->state));
            reset_transaction(transaction);
            break;
        }

        transaction->flush_to_dcpd_buffer_pos += sent_bytes;

        if(transaction->flush_to_dcpd_buffer_pos >= transaction->dcp_buffer.pos)
        {
            if(transaction->state == TR_SLAVE_READCMD_FORWARDING_TO_DCPD)
            {
                clear_buffer(&transaction->dcp_buffer);
                transaction->state = TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD;
            }
            else
            {
                msg_info("%s: DONE", tr_log_prefix(transaction->state));
                reset_transaction(transaction);
            }
        }
        break;

      case TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT:
        reset_transaction(transaction);
        break;
    }

    if(transaction->state == TR_IDLE && deferred_transaction_data->pos > 0)
    {
        msg_info("Continue processing deferred master transaction");

        swap_buffers(deferred_transaction_data, &transaction->dcp_buffer);
        transaction->state = TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE;
    }
}

static void process_request_line(struct dcp_transaction *transaction,
                                 struct buffer *deferred_transaction_data,
                                 int fifo_in_fd, int fifo_out_fd,
                                 int spi_fd, unsigned int spi_timeout_ms,
                                 struct collision_check_data *ccdata,
                                 bool *prev_gpio_state)
{
    bool current_gpio_state = gpio_is_active(ccdata->gpio);

    if(current_gpio_state)
    {
        if(transaction->state == TR_IDLE)
        {
            log_assert(transaction->request_state == REQ_NOT_REQUESTED);

            /* assertion of request line starts slave transaction */
            transaction->request_state = REQ_ASSERTED;
            process_transaction(transaction, deferred_transaction_data,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, ccdata);
        }
        else if(transaction->request_state == REQ_DEASSERTED &&
                current_gpio_state != *prev_gpio_state)
            msg_info("New slave request during ongoing transaction");
    }
    else if(transaction->request_state == REQ_ASSERTED)
    {
        log_assert(transaction->state != TR_IDLE);

        /* deassertion of request line signals that slave is ready for
         * next transaction after this one has been completed */
        transaction->request_state = REQ_DEASSERTED;
    }
    else if(transaction->request_state == REQ_DEASSERTED)
    {
        log_assert(transaction->state != TR_IDLE);
        if(current_gpio_state != *prev_gpio_state)
            msg_info("Lost slave request during ongoing transaction");
    }

    *prev_gpio_state = current_gpio_state;
}

static void wait_for_dcp_data(struct dcp_transaction *transaction,
                              struct buffer *deferred_transaction_data,
                              const int fifo_in_fd, const int fifo_out_fd,
                              const int spi_fd, unsigned int spi_timeout_ms,
                              const int gpio_fd,
                              struct collision_check_data *ccdata,
                              bool *prev_gpio_state)
{
    struct pollfd fds[2] =
    {
        {
            .fd = gpio_fd,
            .events = POLLPRI | POLLERR,
        },
        {
            .fd = fifo_in_fd,
            .events = POLLIN,
        },
    };

    msg_info("Waiting for activities.");

    int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), -1);

    if(ret <= 0)
    {
        if(ret == 0)
            msg_error(errno, LOG_WARNING, "poll() unexpected timeout");
        else if(errno != EINTR)
            msg_error(errno, LOG_CRIT, "poll() failed");

        return;
    }

    if(fds[0].revents & POLLPRI)
        process_request_line(transaction, deferred_transaction_data,
                             fifo_in_fd, fifo_out_fd,
                             spi_fd, spi_timeout_ms, ccdata, prev_gpio_state);

    if(fds[0].revents & ~(POLLPRI | POLLERR))
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on gpio_fd %d: %04x",
                  gpio_fd, fds[0].revents);

    if(fds[1].revents & POLLHUP)
    {
        msg_error(0, LOG_ERR, "DCP daemon died, terminating");
        keep_running = false;
    }

    if(fds[1].revents & POLLIN)
    {
        if(expecting_dcp_data(transaction))
            process_transaction(transaction, deferred_transaction_data,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, ccdata);
        else
            /* FIXME: the DCP process needs to know about this */
            msg_info("collision: DCP tries to send command in the middle of a transaction");
    }

    if(fds[1].revents & ~POLLIN)
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on fifo_fd %d: %04x",
                  fifo_in_fd, fds[1].revents);
}

/*!
 * Copy data back and forth.
 *
 * As long as no transaction is in progress, we are waiting on activities on
 * the named pipe and the request pin. A transaction is started if either the
 * request pin is activated or if some process is sending data to the named
 * pipe.
 *
 * Once a transaction has been started, data needs to be copied and
 * transformed. There are two cases:
 * - Transaction initiated by the slave ("slave transaction")
 * - Transaction initiated by the master ("master transaction")
 *
 * Slave transaction:
 * - Read four bytes long command from SPI
 * - In case of write command: Read optional data from SPI
 * - Transform for DCPD (unescape raw data)
 * - Send transformed command and data to named pipe
 * - In case of read command: Wait for answer from named pipe
 * - In case of read command: Transform for SPI (insert escape sequences)
 * - In case of read command: Send transformed data to SPI
 *
 * Master transaction (always write commands):
 * - Read four bytes long write command from DCPD
 * - Read optional data from DCPD
 * - Transform for SPI (insert escape sequences)
 * - Send transformed data to SPI
 *
 * \param fifo_in_fd
 *     File descriptor of the ingoing named pipe that DCPD is supposed to
 *     connect to for writing to us.
 *
 * \param fifo_out_fd
 *     File descriptor of the outgoing named pipe that DCPD is supposed to
 *     connect to for reading from us.
 *
 * \param spi_fd
 *     File descriptor of the SPI interface.
 *
 * \param gpio
 *     Structure that represents the request input pin the slave device is
 *     supposed to use for requesting data and data rate limitation.
 */
static void main_loop(const int fifo_in_fd, const int fifo_out_fd,
                      const int spi_fd, const struct gpio_handle *const gpio)
{
    msg_info("Ready for accepting traffic");

    static uint8_t dcp_double_buffer[260][2];
    static uint8_t spi_backing_buffer[260 * 2];

    struct dcp_transaction transaction =
    {
        .dcp_buffer =
        {
            .buffer = dcp_double_buffer[0],
            .size = sizeof(dcp_double_buffer[0]),
        },
        .spi_buffer =
        {
            .buffer = spi_backing_buffer,
            .size = sizeof(spi_backing_buffer),
        },
    };

    struct buffer deferred_transaction_data =
    {
        .buffer = dcp_double_buffer[1],
        .size = sizeof(dcp_double_buffer[1]),
    };

    struct collision_check_data ccdata =
    {
        .gpio = gpio,
        .current_transaction = &transaction,
    };

    reset_transaction_struct(&transaction);

    static const unsigned int spi_timeout_ms = 1000;

    const int gpio_fd = gpio != NULL ? gpio_get_poll_fd(gpio) : -1;
    bool prev_gpio_state = gpio != NULL ? gpio_is_active(gpio) : false;

    while(keep_running)
    {
        if(gpio != NULL)
            process_request_line(&transaction, &deferred_transaction_data,
                                 fifo_in_fd, fifo_out_fd,
                                 spi_fd, spi_timeout_ms, &ccdata, &prev_gpio_state);

        if(expecting_dcp_data(&transaction))
            wait_for_dcp_data(&transaction, &deferred_transaction_data,
                              fifo_in_fd, fifo_out_fd,
                              spi_fd, spi_timeout_ms,
                              gpio_fd, &ccdata, &prev_gpio_state);
        else
            process_transaction(&transaction, &deferred_transaction_data,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, &ccdata);
    }
}

struct parameters
{
    const char *fifo_in_name;
    const char *fifo_out_name;
    const char *spidev_name;
    uint32_t spi_clock;
    unsigned int gpio_num;
    bool run_in_foreground;
    bool gpio_needs_debouncing;
    bool dummy_mode;
};

/*!
 * Open devices, daemonize.
 */
static int setup(const struct parameters *parameters,
                 int *fifo_in_fd, int *fifo_out_fd,
                 int *spi_fd, struct gpio_handle **gpio)
{
    msg_enable_syslog(!parameters->run_in_foreground);

    if(!parameters->run_in_foreground)
        openlog("dcpspi", LOG_PID, LOG_DAEMON);

    if(!parameters->run_in_foreground)
    {
        if(daemon(0, 0) < 0)
        {
            msg_error(errno, LOG_EMERG, "Failed to run as daemon");
            return -1;
        }
    }

    log_version_info();

    *fifo_in_fd = fifo_create_and_open(parameters->fifo_in_name, false);
    if(*fifo_in_fd < 0)
        return -1;

    *fifo_out_fd = fifo_create_and_open(parameters->fifo_out_name, true);
    if(*fifo_out_fd < 0)
        goto error_fifo_out;

    if(parameters->dummy_mode)
    {
        *spi_fd = -1;
        *gpio = NULL;
        return 0;
    }

    *spi_fd = spi_open_device(parameters->spidev_name);
    if(*spi_fd < 0)
        goto error_spi_open;

    *gpio = gpio_open(parameters->gpio_num, false);
    if(*gpio == NULL)
        goto error_gpio_open;

    if(parameters->gpio_needs_debouncing)
        gpio_enable_debouncing(*gpio);

    spi_set_speed_hz(parameters->spi_clock);

    return 0;

error_gpio_open:
    spi_close_device(*spi_fd);

error_spi_open:
    fifo_close_and_delete(fifo_out_fd, parameters->fifo_out_name);

error_fifo_out:
    fifo_close_and_delete(fifo_in_fd, parameters->fifo_in_name);
    return -1;
}

static void usage(const char *program_name)
{
    printf("Usage: %s --fifo name --spidev name --irq gpio\n"
           "\n"
           "Options:\n"
           "  --help         Show this help.\n"
           "  --version      Print version information to stdout.\n"
           "  --fg           Run in foreground, don't run as daemon.\n"
           "  --ififo name   Name of the named pipe the DCP daemon writes to.\n"
           "  --ofifo name   Name of the named pipe the DCP daemon reads from.\n"
           "  --spidev name  Name of the SPI device.\n"
           "  --spiclk hz    Clock frequency on SPI bus.\n"
           "  --gpio num     Number of the slave request pin.\n"
           "  --debounce     Enable software debouncing of request pin.\n",
           program_name);
}

static int process_command_line(int argc, char *argv[],
                                struct parameters *parameters)
{
    parameters->fifo_in_name = "/tmp/dcp_to_spi";
    parameters->fifo_out_name = "/tmp/spi_to_dcp";
    parameters->spidev_name = "/dev/spidev0.0";
    parameters->spi_clock = 0;
    parameters->gpio_num = 4;
    parameters->run_in_foreground = false;
    parameters->gpio_needs_debouncing = false;
    parameters->dummy_mode = false;

#define CHECK_ARGUMENT() \
    do \
    { \
        if(i + 1 >= argc) \
        { \
            fprintf(stderr, "Option %s requires an argument.\n", argv[i]); \
            return -1; \
        } \
        ++i; \
    } \
    while(0)

    for(int i = 1; i < argc; ++i)
    {
        if(strcmp(argv[i], "--help") == 0)
            return 1;
        else if(strcmp(argv[i], "--version") == 0)
            return 2;
        else if(strcmp(argv[i], "--fg") == 0)
            parameters->run_in_foreground = true;
        else if(strcmp(argv[i], "--ififo") == 0)
        {
            CHECK_ARGUMENT();
            parameters->fifo_in_name = argv[i];
        }
        else if(strcmp(argv[i], "--ofifo") == 0)
        {
            CHECK_ARGUMENT();
            parameters->fifo_out_name = argv[i];
        }
        else if(strcmp(argv[i], "--spidev") == 0)
        {
            CHECK_ARGUMENT();
            parameters->spidev_name = argv[i];
        }
        else if(strcmp(argv[i], "--spiclk") == 0)
        {
            CHECK_ARGUMENT();

            char *endptr;
            unsigned long temp = strtoul(argv[i], &endptr, 10);

            if(*endptr != '\0' || temp > UINT32_MAX || (temp == ULONG_MAX && errno == ERANGE))
            {
                fprintf(stderr, "Invalid value \"%s\". Please try --help.\n", argv[i]);
                return -1;
            }

            parameters->spi_clock = temp;
        }
        else if(strcmp(argv[i], "--gpio") == 0)
        {
            CHECK_ARGUMENT();

            char *endptr;
            unsigned long temp = strtoul(argv[i], &endptr, 10);

            if(*endptr != '\0' || temp > UINT_MAX || (temp == ULONG_MAX && errno == ERANGE))
            {
                fprintf(stderr, "Invalid value \"%s\". Please try --help.\n", argv[i]);
                return -1;
            }

            parameters->gpio_num = temp;
        }
        else if(strcmp(argv[i], "--debounce") == 0)
            parameters->gpio_needs_debouncing = true;
        else
        {
            fprintf(stderr, "Unknown option \"%s\". Please try --help.\n", argv[i]);
            return -1;
        }
    }

#undef CHECK_ARGUMENT

    if(parameters->spidev_name[0] == '-' && parameters->spidev_name[1] == '\0')
        parameters->dummy_mode = true;

    return 0;
}

static void signal_handler(int signum, siginfo_t *info, void *ucontext)
{
    keep_running = false;
}

int main(int argc, char *argv[])
{
    static struct parameters parameters;

    int ret = process_command_line(argc, argv, &parameters);

    if(ret == -1)
        return EXIT_FAILURE;
    else if(ret == 1)
    {
        usage(argv[0]);
        return EXIT_SUCCESS;
    }
    else if(ret == 2)
    {
        show_version_info();
        return EXIT_SUCCESS;
    }

    int fifo_in_fd, fifo_out_fd, spi_fd;
    struct gpio_handle *gpio;

    if(setup(&parameters, &fifo_in_fd, &fifo_out_fd, &spi_fd, &gpio) < 0)
        return EXIT_FAILURE;

    static struct sigaction action =
    {
        .sa_sigaction = signal_handler,
        .sa_flags = SA_SIGINFO | SA_RESETHAND,
    };

    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    main_loop(fifo_in_fd, fifo_out_fd, spi_fd, gpio);

    msg_info("Terminated, shutting down");

    if(!parameters.dummy_mode)
        spi_close_device(spi_fd);

    fifo_close_and_delete(&fifo_in_fd, parameters.fifo_in_name);
    fifo_close_and_delete(&fifo_out_fd, parameters.fifo_out_name);

    if(!parameters.dummy_mode)
        gpio_close(gpio);

    return EXIT_SUCCESS;
}
