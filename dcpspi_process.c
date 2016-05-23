/*
 * Copyright (C) 2016  T+A elektroakustik GmbH & Co. KG
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

#include <errno.h>

#include "dcpspi_process.h"
#include "dcpdefs.h"
#include "spi.h"
#include "named_pipe.h"
#include "gpio.h"
#include "messages.h"
#include "os.h"

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

static bool expecting_gpio_change(const struct dcp_transaction *transaction)
{
    return (transaction->state == TR_IDLE ||
            transaction->state == TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT);
}

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

    ssize_t len = os_read(fd, buffer->buffer + buffer->pos, count);

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

    ssize_t len = os_write(fd, buffer->buffer + offset, count);

    if(len > 0)
        return len;

    msg_error(errno, LOG_EMERG,
              "Failed writing %zu bytes to fd %d (write() returned %zd)",
              count, fd, len);
    return len < 0 ? -1 : 0;
}

void reset_transaction_struct(struct dcp_transaction *transaction)
{
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

    msg_info("%s: expecting %u bytes from %s",
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

    if(transaction->pending_size_of_transaction == 0 ||
       is_buffer_full(&transaction->dcp_buffer))
    {
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
            break;

          case REQ_ASSERTED:
            transaction->state = TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE;
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
            msg_info("%s: header from DCPD incomplete, waiting for more input",
                     tr_log_prefix(transaction->state));
            break;
        }

        /*
         * Possibly received a DCP header. Note that we explictly do not
         * validate the header content here because this is going to be done by
         * the receiver of the data. We simply assume that we have a header
         * here and that the length can be determined.
         */
        msg_info("%s: command header from DCPD: 0x%02x 0x%02x 0x%02x 0x%02x",
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

        msg_info("%s: send %zu bytes over SPI",
                 tr_log_prefix(transaction->state),
                 transaction->spi_buffer.pos);

        const enum SpiSendResult ret =
            spi_send_buffer(spi_fd, transaction->spi_buffer.buffer,
                            transaction->spi_buffer.pos, spi_timeout_ms,
                            check_slave_request_collision, ccdata);
        bool leave_switch = false;

        switch(ret)
        {
          case SPI_SEND_RESULT_OK:
            break;

          case SPI_SEND_RESULT_TIMEOUT:
          case SPI_SEND_RESULT_FAILURE:
            /* hard failure or timeout; abort transaction */
            reset_transaction(transaction);
            leave_switch = true;
            break;

          case SPI_SEND_RESULT_COLLISION:
            if(transaction->state == TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE)
            {
                /* collision; process slave request first, reschedule master
                 * transaction when the slave request is done */
                if(deferred_transaction_data->pos > 0)
                    BUG("Lost deferred transaction");

                swap_buffers(deferred_transaction_data, &transaction->dcp_buffer);
                transaction->state = TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE;
                transaction->request_state = REQ_ASSERTED;
                transaction->spi_buffer.pos = 0;
                leave_switch = true;
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

            break;
        }

        if(leave_switch)
            break;

        reset_transaction(transaction);
        break;

      case TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE:
        if(spi_read_buffer(spi_fd, transaction->dcp_buffer.buffer,
                           DCP_HEADER_SIZE, spi_timeout_ms) < DCP_HEADER_SIZE)
        {
            reset_transaction(transaction);
            break;
        }

        transaction->dcp_buffer.pos = DCP_HEADER_SIZE;

        msg_info("%s: command header from SPI: 0x%02x 0x%02x 0x%02x 0x%02x",
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
        msg_info("%s: send %zu bytes to DCPD",
                 tr_log_prefix(transaction->state),
                 transaction->dcp_buffer.pos);

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
                reset_transaction(transaction);
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
        transaction->request_state = REQ_NOT_REQUESTED;
    }
}

static bool process_request_line(struct dcp_transaction *transaction,
                                 struct buffer *deferred_transaction_data,
                                 int fifo_in_fd, int fifo_out_fd,
                                 int spi_fd, unsigned int spi_timeout_ms,
                                 struct collision_check_data *ccdata,
                                 bool *prev_gpio_state)
{
    bool slave_has_released_request_line = false;
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
        slave_has_released_request_line = true;
    }
    else if(transaction->request_state == REQ_DEASSERTED)
    {
        log_assert(transaction->state != TR_IDLE);
        if(current_gpio_state != *prev_gpio_state)
            msg_info("Lost slave request during ongoing transaction");
    }

    *prev_gpio_state = current_gpio_state;

    return slave_has_released_request_line;
}

static bool wait_for_dcp_data(struct dcp_transaction *transaction,
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

    int ret = os_poll(fds, sizeof(fds) / sizeof(fds[0]), -1);

    if(ret <= 0)
    {
        if(ret == 0)
            msg_error(errno, LOG_WARNING, "poll() unexpected timeout");
        else if(errno != EINTR)
            msg_error(errno, LOG_CRIT, "poll() failed");

        return true;
    }

    if(fds[0].revents & POLLPRI)
    {
        if(process_request_line(transaction, deferred_transaction_data,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, ccdata, prev_gpio_state))
            process_transaction(transaction, deferred_transaction_data,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms, ccdata);
    }

    if(fds[0].revents & ~(POLLPRI | POLLERR))
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on gpio_fd %d: %04x",
                  gpio_fd, fds[0].revents);

    bool keep_running = true;

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
            BUG("DCP tries to send command in the middle of a transaction");
    }

    if(fds[1].revents & ~POLLIN)
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on fifo_fd %d: %04x",
                  fifo_in_fd, fds[1].revents);

    return keep_running;
}

bool dcpspi_process(const int fifo_in_fd, const int fifo_out_fd,
                    const int spi_fd, const int gpio_fd,
                    bool is_running_for_real,
                    struct dcp_transaction *const transaction,
                    struct buffer *const deferred_transaction_data,
                    struct collision_check_data *const ccdata,
                    bool *prev_gpio_state)
{
    static const unsigned int spi_timeout_ms = 1000;

    if(is_running_for_real)
        process_request_line(transaction, deferred_transaction_data,
                             fifo_in_fd, fifo_out_fd,
                             spi_fd, spi_timeout_ms, ccdata, prev_gpio_state);

    if(expecting_dcp_data(transaction) || expecting_gpio_change(transaction))
        return wait_for_dcp_data(transaction, deferred_transaction_data,
                                 fifo_in_fd, fifo_out_fd,
                                 spi_fd, spi_timeout_ms,
                                 gpio_fd, ccdata, prev_gpio_state);

    process_transaction(transaction, deferred_transaction_data,
                        fifo_in_fd, fifo_out_fd,
                        spi_fd, spi_timeout_ms, ccdata);

    return true;
}
