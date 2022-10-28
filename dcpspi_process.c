/*
 * Copyright (C) 2016, 2018, 2019, 2022  T+A elektroakustik GmbH & Co. KG
 *
 * This file is part of DCPSPI.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
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
 * What has happened on the request pin since we've checked last.
 *
 * The result #REQUEST_LINE_DEASSERTED_AND_ASSERTED or
 * #REQUEST_LINE_ASSERTED_AND_DEASSERTED is returned in case we were to slow to
 * see the individual GPIO changes. Still, poll(2) will report the changes,
 * albeit as a single event, as the low-level driver has seen and reported the
 * changes.
 *
 * It is well possible that there were more changes of GPIO state that
 * indicated by the result codes, but we will never be able to figure this out.
 * For instance, #REQUEST_LINE_ASSERTED may be reported in case the state
 * changed from low to high, then to low, and then to high again, but if these
 * changes came in too fast for the system to recognize/process, these
 * information will be lost.
 */
enum RequestLineChanges
{
    REQUEST_LINE_ASSERTED,
    REQUEST_LINE_DEASSERTED,

    REQUEST_LINE_DEASSERTED_AND_ASSERTED,
    REQUEST_LINE_ASSERTED_AND_DEASSERTED,
};

#define EVENT_FD_COUNT 2

static struct
{
    uint16_t next_dcpsync_serial;
    struct program_statistics statistics;
}
dcpspi_globals;

#define STATISTICS_STRUCT(S) \
    (dcpspi_globals.statistics.is_enabled ? &dcpspi_globals.statistics.S : NULL)

static uint16_t mk_serial(void)
{
    if(dcpspi_globals.next_dcpsync_serial < DCPSYNC_SLAVE_SERIAL_MIN ||
       dcpspi_globals.next_dcpsync_serial > DCPSYNC_SLAVE_SERIAL_MAX)
    {
        dcpspi_globals.next_dcpsync_serial = DCPSYNC_SLAVE_SERIAL_MIN;
    }

    return dcpspi_globals.next_dcpsync_serial++;
}

void dcpspi_init(void)
{
    dcpspi_globals.next_dcpsync_serial = 0;
    dcpspi_statistics_reset();
}

void dcpspi_statistics_reset(void)
{
    stats_context_reset(&dcpspi_globals.statistics.busy_unspecific);
    stats_context_reset(&dcpspi_globals.statistics.busy_gpio);
    stats_context_reset(&dcpspi_globals.statistics.busy_transaction);
    stats_context_reset(&dcpspi_globals.statistics.wait_for_events);
    stats_io_reset(&dcpspi_globals.statistics.spi_transfers);
    stats_io_reset(&dcpspi_globals.statistics.dcpd_reads);
    stats_io_reset(&dcpspi_globals.statistics.dcpd_writes);
}

const struct program_statistics *dcpspi_statistics_get(void)
{
    return &dcpspi_globals.statistics;
}

bool dcpspi_statistics_enable(bool enable)
{
    const bool result = dcpspi_globals.statistics.is_enabled;
    dcpspi_globals.statistics.is_enabled = enable;
    return result;
}

/*!
 * Whether or not the DCP process is allowed to send any data.
 */
static bool expecting_dcp_data(const struct dcp_transaction *transaction)
{
    return (transaction->state == TR_IDLE ||
            transaction->state == TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD ||
            transaction->state == TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD ||
            transaction->state == TR_MASTER_COMMAND_SKIPPING_DATA_FROM_DCPD);
}

static bool expecting_gpio_change(const struct dcp_transaction *transaction)
{
    return (transaction->state == TR_IDLE ||
            (transaction->state == TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT &&
             transaction->request_state == REQSTATE_LOCKED));
}

static void clear_buffer(struct buffer *buffer)
{
    buffer->pos = 0;
}

static bool is_buffer_full(const struct buffer *buffer)
{
    return buffer->pos >= buffer->size;
}

static int fill_buffer_from_fd(struct buffer *buffer, size_t count, int fd,
                               struct stats_io *io)
{
    if(count == 0)
        return 0;

    struct stats_context *prev_ctx = stats_io_begin(io);

    ssize_t len = os_read(fd, buffer->buffer + buffer->pos, count);

    stats_io_end(io, prev_ctx, len <= 0 ? 1 : 0, len >= 0 ? len : 0);

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
                                 size_t offset, size_t count, int fd,
                                 struct stats_io *io)
{
    errno = 0;

    struct stats_context *prev_ctx = stats_io_begin(io);

    ssize_t len = os_write(fd, buffer->buffer + offset, count);

    stats_io_end(io, prev_ctx, len <= 0 ? 1 : 0, len >= 0 ? len : 0);

    if(len > 0)
        return len;

    msg_error(errno, LOG_EMERG,
              "Failed writing %zu bytes to fd %d (write() returned %zd)",
              count, fd, len);
    return len < 0 ? -1 : 0;
}

bool reset_transaction_struct(struct dcp_transaction *transaction,
                              bool is_initial_reset)
{
    transaction->state = TR_IDLE;

    clear_buffer(&transaction->dcp_buffer);
    transaction->serial = 0;
    transaction->pending_size_of_transaction = 0;
    transaction->flush_to_dcpd_buffer_pos = 0;

    clear_buffer(&transaction->spi_buffer);

    if(is_initial_reset)
    {
        transaction->request_state = REQSTATE_IDLE;
        return false;
    }

    switch(transaction->request_state)
    {
      case REQSTATE_IDLE:
      case REQSTATE_RELEASED:
        transaction->request_state = REQSTATE_IDLE;
        break;

      case REQSTATE_LOCKED:
        BUG("Reset locked transaction, chaos expected");
        return true;

      case REQSTATE_NEXT_PENDING:
        msg_vinfo(MESSAGE_LEVEL_DIAG, "Processing pending slave transaction");
        transaction->request_state = REQSTATE_LOCKED;
        return true;

      case REQSTATE_MISSED:
        /* check residual bytes in SPI input buffer, if any */
        return true;
    }

    return false;
}

static void reuse_transaction_for_collision(struct dcp_transaction *transaction)
{
    transaction->state = TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE;

    transaction->serial = 0;
    transaction->spi_buffer.pos = 0;
}

static bool reset_transaction(struct dcp_transaction *transaction)
{
    switch(transaction->state)
    {
      case TR_IDLE:
      case TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD:
      case TR_MASTER_COMMAND_FORWARDING_TO_SLAVE:
      case TR_MASTER_COMMAND_SKIPPING_DATA_FROM_DCPD:
      case TR_MASTER_COMMAND_SKIPPED:
        break;

      case TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE:
      case TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE:
      case TR_SLAVE_COMMAND_FORWARDING_TO_DCPD:
      case TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT:
        if(transaction->request_state == REQSTATE_LOCKED)
        {
            msg_vinfo(MESSAGE_LEVEL_DEBUG,
                      "About to end transaction 0x%04x in state %d, "
                      "waiting for slave to release request line",
                      transaction->serial, transaction->state);
            transaction->state = TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT;

            return false;
        }
        else
        {
            const char *const what_next =
                (transaction->request_state == REQSTATE_NEXT_PENDING
                 ? "slave request pending"
                 : (transaction->request_state == REQSTATE_MISSED
                    ? "looking for missed transactions"
                    : "return to idle state"));

            msg_vinfo(MESSAGE_LEVEL_DEBUG,
                      "End of transaction 0x%04x in state %d, %s",
                      transaction->serial, transaction->state, what_next);
        }

        break;
    }

    return reset_transaction_struct(transaction, false);
}

static uint8_t get_dcp_command_type(uint8_t dcp_header_first_byte)
{
    return dcp_header_first_byte & 0x0f;
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

static void fill_dcpsync_header_generic(uint8_t *const dcpsync_header,
                                        const uint8_t command,
                                        const uint8_t ttl,
                                        const uint16_t serial,
                                        const uint16_t dcp_packet_size)
{
    dcpsync_header[0] = command;
    dcpsync_header[1] = ttl;
    dcpsync_header[2] = (serial >> 8) & UINT8_MAX;
    dcpsync_header[3] = (serial >> 0) & UINT8_MAX;
    dcpsync_header[4] = (dcp_packet_size >> 8) & UINT8_MAX;
    dcpsync_header[5] = (dcp_packet_size >> 0) & UINT8_MAX;
}

static void fill_dcpsync_header_for_slave(uint8_t *dcpsync_header,
                                          uint16_t serial,
                                          uint16_t dcp_packet_size)
{
    fill_dcpsync_header_generic(dcpsync_header, 'c', 0, serial, dcp_packet_size);
}

static void send_packet_accepted_message(struct buffer *buffer,
                                         uint16_t serial, int fd)
{
    msg_vinfo(MESSAGE_LEVEL_TRACE, "ACK 0x%04x", serial);
    fill_dcpsync_header_generic(buffer->buffer, 'a', 0, serial, 0);
    send_buffer_to_fd(buffer, 0, DCPSYNC_HEADER_SIZE, fd,
                      STATISTICS_STRUCT(dcpd_writes));
}

static void send_packet_rejected_message(struct buffer *buffer,
                                         uint16_t serial, uint8_t ttl, int fd)
{
    if(ttl > 0)
        msg_vinfo(MESSAGE_LEVEL_TRACE, "NACK 0x%04x", serial);
    else
        msg_vinfo(MESSAGE_LEVEL_TRACE, "DROP 0x%04x", serial);

    fill_dcpsync_header_generic(buffer->buffer, 'n', ttl, serial, 0);
    send_buffer_to_fd(buffer, 0, DCPSYNC_HEADER_SIZE, fd,
                      STATISTICS_STRUCT(dcpd_writes));
}

static void send_packet_dropped_message(struct buffer *buffer,
                                        uint16_t serial, int fd)
{
    send_packet_rejected_message(buffer, serial, 0, fd);
}

static inline uint8_t get_dcpsync_command(const uint8_t *dcpsync_header)
{
    return dcpsync_header[0];
}

static inline uint8_t get_dcpsync_ttl(const uint8_t *dcpsync_header)
{
    return dcpsync_header[1];
}

static inline uint16_t get_dcpsync_serial(const uint8_t *dcpsync_header)
{
    return (dcpsync_header[2] << 8) | dcpsync_header[3];
}

static inline uint16_t get_dcpsync_data_size(const uint8_t *dcpsync_header)
{
    return (dcpsync_header[4] << 8) | dcpsync_header[5];
}

static const char *tr_log_prefix(enum transaction_state state)
{
    switch(state)
    {
      case TR_IDLE:
        return "No transaction";

      case TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD:
      case TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD:
      case TR_MASTER_COMMAND_FORWARDING_TO_SLAVE:
      case TR_MASTER_COMMAND_SKIPPING_DATA_FROM_DCPD:
      case TR_MASTER_COMMAND_SKIPPED:
        return "Master transaction";

      case TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE:
      case TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE:
      case TR_SLAVE_COMMAND_FORWARDING_TO_DCPD:
      case TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT:
        return "Slave transaction";
    }

    return "INVALID transaction";
}

static bool process_transaction_receive_data(struct dcp_transaction *transaction,
                                             struct slave_request_and_lock_data *rldata,
                                             int fifo_in_fd, int spi_fd,
                                             unsigned int spi_timeout_ms)
{
    const char *read_peer =
        (transaction->state == TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE) ? "slave" : "DCPD";

    const size_t read_size = compute_read_size(transaction);
    const int bytes_read =
        (transaction->state == TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE)
        ? spi_read_buffer(spi_fd,
                          transaction->dcp_buffer.buffer + transaction->dcp_buffer.pos,
                          read_size, spi_timeout_ms,
                          STATISTICS_STRUCT(spi_transfers))
        : fill_buffer_from_fd(&transaction->dcp_buffer, read_size, fifo_in_fd,
                              STATISTICS_STRUCT(dcpd_reads));

    if(bytes_read < 0)
    {
        msg_error(0, LOG_ERR, "%s: communication with %s broken",
                  tr_log_prefix(transaction->state), read_peer);
        return reset_transaction(transaction);
    }

    if(transaction->state == TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE)
        transaction->dcp_buffer.pos += bytes_read;

    transaction->pending_size_of_transaction -= (size_t)bytes_read;

    if(transaction->pending_size_of_transaction == 0 ||
       is_buffer_full(&transaction->dcp_buffer))
    {
        transaction->flush_to_dcpd_buffer_pos = 0;
        if(transaction->state == TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD)
            transaction->state = TR_MASTER_COMMAND_FORWARDING_TO_SLAVE;
        else if(transaction->state == TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE)
            transaction->state = TR_SLAVE_COMMAND_FORWARDING_TO_DCPD;
        else
        {
            /* ignore packet with data, take next packet from DCPD */
            clear_buffer(&transaction->dcp_buffer);
            transaction->state = TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD;
        }
    }

    return false;
}

static void reject_or_drop(struct dcp_transaction *transaction,
                           int fifo_out_fd)
{
    if(transaction->ttl > 0)
    {
        --transaction->ttl;

        send_packet_rejected_message(&transaction->dcp_buffer,
                                     transaction->serial, transaction->ttl,
                                     fifo_out_fd);
    }
    else
        msg_vinfo(MESSAGE_LEVEL_DIAG,
                  "Silently dropping 0x%04x", transaction->serial);
}

static bool do_process_transaction(struct dcp_transaction *transaction,
                                   struct slave_request_and_lock_data *rldata,
                                   int fifo_in_fd, int fifo_out_fd,
                                   int spi_fd, unsigned int spi_timeout_ms)
{
    msg_vinfo(MESSAGE_LEVEL_DEBUG,
              "Process transaction state %d, serial 0x%04x, lock state %d, "
              "pending size %u, flush pos %zu",
              transaction->state, transaction->serial,
              transaction->request_state,
              transaction->pending_size_of_transaction,
              transaction->flush_to_dcpd_buffer_pos);

    bool retval = false;

    switch(transaction->state)
    {
      case TR_IDLE:
        switch(transaction->request_state)
        {
          case REQSTATE_IDLE:
            transaction->state = TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD;
            break;

          case REQSTATE_LOCKED:
            transaction->state = TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE;
            spi_new_transaction();
            return false;

          case REQSTATE_RELEASED:
            msg_info("No transaction to process");
            return reset_transaction(transaction);

          case REQSTATE_MISSED:
            transaction->request_state = REQSTATE_RELEASED;

            if(!spi_input_buffer_weed())
            {
                msg_info("No lost packets found in SPI input buffer");
                return reset_transaction(transaction);
            }

            msg_info("Possibly found lost packet(s) in SPI input buffer");
            transaction->state = TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE;

            return true;

          case REQSTATE_NEXT_PENDING:
            BUG("Invalid request state %d for idle transaction",
                transaction->request_state);
            return false;
        }

        /* fall-through */

      case TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD:
        if(fill_buffer_from_fd(&transaction->dcp_buffer,
                               (DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE) - transaction->dcp_buffer.pos,
                               fifo_in_fd,
                               STATISTICS_STRUCT(dcpd_reads)) < 0)
        {
            msg_error(0, LOG_ERR, "%s: communication with DCPD broken (receive)",
                      tr_log_prefix(transaction->state));
            retval = reset_transaction(transaction);
            break;
        }

        if(transaction->dcp_buffer.pos != DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE)
        {
            msg_vinfo(MESSAGE_LEVEL_DIAG,
                      "%s: header from DCPD incomplete, waiting for more input",
                      tr_log_prefix(transaction->state));
            break;
        }

        /*
         * Possibly received a DCP header. Note that we explictly do not
         * validate the header content here because this is going to be done by
         * the receiver of the data. We rely on the DCPSYNC header instead.
         */
        msg_vinfo(MESSAGE_LEVEL_DIAG,
                  "%s: command header from DCPD: 0x%02x 0x%02x 0x%02x 0x%02x",
                  tr_log_prefix(transaction->state),
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 0],
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 1],
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 2],
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 3]);

        transaction->ttl = get_dcpsync_ttl(transaction->dcp_buffer.buffer);
        transaction->serial = get_dcpsync_serial(transaction->dcp_buffer.buffer);
        transaction->pending_size_of_transaction =
            get_dcpsync_data_size(transaction->dcp_buffer.buffer) - DCP_HEADER_SIZE;

        /* cap the number of retries */
        if(transaction->ttl > 10)
            transaction->ttl = 10;

        if(get_dcpsync_command(transaction->dcp_buffer.buffer) == 'c')
        {
            if(transaction->pending_size_of_transaction > 0)
            {
                transaction->state = TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD;
                break;
            }
            else
                transaction->state = TR_MASTER_COMMAND_FORWARDING_TO_SLAVE;
        }
        else
        {
            BUG("Unexpected DCPSYNC command 0x%02x for packet serial 0x%04x "
                "with ttl %u, skipping %u bytes",
                get_dcpsync_command(transaction->dcp_buffer.buffer),
                transaction->serial, transaction->ttl,
                transaction->pending_size_of_transaction);

            if(transaction->pending_size_of_transaction > 0)
                transaction->state = TR_MASTER_COMMAND_SKIPPING_DATA_FROM_DCPD;
            else
                transaction->state = TR_MASTER_COMMAND_SKIPPED;

            break;
        }

        /* fall-through */

      case TR_MASTER_COMMAND_FORWARDING_TO_SLAVE:
        log_assert(transaction->pending_size_of_transaction == 0);

        clear_buffer(&transaction->spi_buffer);

        const bool is_dummy_header =
            transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE] == UINT8_MAX;

        if(!is_dummy_header)
            transaction->spi_buffer.pos =
                spi_fill_buffer_from_raw_data(transaction->spi_buffer.buffer,
                                              transaction->spi_buffer.size,
                                              transaction->dcp_buffer.buffer + DCPSYNC_HEADER_SIZE,
                                              transaction->dcp_buffer.pos - DCPSYNC_HEADER_SIZE);

        const enum SpiSendResult ret = is_dummy_header
            ? SPI_SEND_RESULT_OK
            : spi_send_buffer(spi_fd, transaction->spi_buffer.buffer,
                              transaction->spi_buffer.pos, spi_timeout_ms,
                              STATISTICS_STRUCT(spi_transfers));
        switch(ret)
        {
          case SPI_SEND_RESULT_OK:
            send_packet_accepted_message(&transaction->dcp_buffer,
                                         transaction->serial, fifo_out_fd);
            retval = reset_transaction(transaction);

            break;

          case SPI_SEND_RESULT_FAILURE:
            /* hard failure or timeout; abort transaction */
            send_packet_dropped_message(&transaction->dcp_buffer,
                                        transaction->serial, fifo_out_fd);
            retval = reset_transaction(transaction);

            break;

          case SPI_SEND_RESULT_TIMEOUT:
            reject_or_drop(transaction, fifo_out_fd);
            retval = reset_transaction(transaction);

            break;

          case SPI_SEND_RESULT_COLLISION:
            /* slave coincidentally interrupted our attempt to tell something,
             * need to try again later */
            reject_or_drop(transaction, fifo_out_fd);
            reuse_transaction_for_collision(transaction);

            break;
        }

        break;

      case TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE:
        if(spi_read_buffer(spi_fd,
                           transaction->dcp_buffer.buffer + DCPSYNC_HEADER_SIZE,
                           DCP_HEADER_SIZE, spi_timeout_ms,
                           STATISTICS_STRUCT(spi_transfers)) < DCP_HEADER_SIZE)
        {
            retval = reset_transaction(transaction);
            break;
        }

        transaction->dcp_buffer.pos = DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE;

        msg_vinfo(MESSAGE_LEVEL_DIAG,
                  "%s: command header from SPI: 0x%02x 0x%02x 0x%02x 0x%02x",
                  tr_log_prefix(transaction->state),
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 0],
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 1],
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 2],
                  transaction->dcp_buffer.buffer[DCPSYNC_HEADER_SIZE + 3]);

        const uint16_t dcp_payload_size =
            get_dcp_data_size(transaction->dcp_buffer.buffer + DCPSYNC_HEADER_SIZE);

        if(dcp_payload_size > (transaction->dcp_buffer.size - DCP_HEADER_SIZE))
        {
            msg_error(EINVAL, LOG_ERR,
                      "%s: transaction size %u exceeds maximum size of %zu",
                      tr_log_prefix(transaction->state),
                      dcp_payload_size,
                      transaction->dcp_buffer.size - DCP_HEADER_SIZE);
            retval = reset_transaction(transaction);
            break;
        }

        transaction->serial = mk_serial();

        fill_dcpsync_header_for_slave(transaction->dcp_buffer.buffer,
                                      transaction->serial,
                                      DCP_HEADER_SIZE + dcp_payload_size);

        if(dcp_payload_size> 0)
        {
            transaction->pending_size_of_transaction = dcp_payload_size;
            transaction->state = TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE;
        }
        else
        {
            transaction->pending_size_of_transaction = transaction->dcp_buffer.pos;
            transaction->state = TR_SLAVE_COMMAND_FORWARDING_TO_DCPD;
            break;
        }

        /* fall-through */

      case TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD:
      case TR_MASTER_COMMAND_SKIPPING_DATA_FROM_DCPD:
      case TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE:
        retval = process_transaction_receive_data(transaction, rldata,
                                                  fifo_in_fd, spi_fd,
                                                  spi_timeout_ms);
        break;

      case TR_MASTER_COMMAND_SKIPPED:
        send_packet_dropped_message(&transaction->dcp_buffer,
                                    transaction->serial, fifo_out_fd);
        retval = reset_transaction(transaction);
        break;

      case TR_SLAVE_COMMAND_FORWARDING_TO_DCPD:
        {
            ssize_t sent_bytes =
                send_buffer_to_fd(&transaction->dcp_buffer,
                                  transaction->flush_to_dcpd_buffer_pos,
                                  transaction->dcp_buffer.pos - transaction->flush_to_dcpd_buffer_pos,
                                  fifo_out_fd,
                                  STATISTICS_STRUCT(dcpd_writes));

            if(sent_bytes < 0)
            {
                msg_error(0, LOG_ERR, "%s: communication with DCPD broken (send)",
                          tr_log_prefix(transaction->state));
                retval = reset_transaction(transaction);
                break;
            }

            transaction->flush_to_dcpd_buffer_pos += sent_bytes;

        }

        if(transaction->flush_to_dcpd_buffer_pos >= transaction->dcp_buffer.pos)
            retval = reset_transaction(transaction);

        break;

      case TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT:
        retval = reset_transaction(transaction);
        break;
    }

    return retval;
}

static void process_transaction(struct dcp_transaction *transaction,
                                struct slave_request_and_lock_data *rldata,
                                int fifo_in_fd, int fifo_out_fd,
                                int spi_fd, unsigned int spi_timeout_ms)
{
    struct stats_context *prev_ctx =
        stats_context_switch(STATISTICS_STRUCT(busy_transaction));

    while(do_process_transaction(transaction, rldata,
                                 fifo_in_fd, fifo_out_fd,
                                 spi_fd, spi_timeout_ms))
        ;

    stats_context_switch(prev_ctx);
}

static enum RequestLineChanges determine_gpio_changes(bool current_state,
                                                      bool prev_state)
{
    if(current_state != prev_state)
    {
        msg_vinfo(MESSAGE_LEVEL_TRACE, "*** GPIO %d -> %d ***", prev_state, current_state);
        return current_state ? REQUEST_LINE_ASSERTED : REQUEST_LINE_DEASSERTED;
    }

    msg_vinfo(MESSAGE_LEVEL_TRACE, "*** GPIO %d -> %d -> %d ***",
              current_state, !current_state, current_state);

    return current_state
        ? REQUEST_LINE_DEASSERTED_AND_ASSERTED
        : REQUEST_LINE_ASSERTED_AND_DEASSERTED;
}

static bool process_request_line(struct dcp_transaction *transaction,
                                 struct slave_request_and_lock_data *rldata,
                                 int fifo_in_fd, int fifo_out_fd,
                                 int spi_fd, unsigned int spi_timeout_ms)
{
    struct stats_context *prev_ctx =
        stats_context_switch(STATISTICS_STRUCT(busy_gpio));

    bool need_more_processing = false;
    bool current_gpio_state = gpio_is_active(rldata->gpio);

    enum RequestLineChanges changes =
        determine_gpio_changes(current_gpio_state, rldata->previous_gpio_state);

    rldata->previous_gpio_state = current_gpio_state;

    if(!current_gpio_state &&
       transaction->state == TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE)
        APPLIANCE_BUG("Transaction was requested by slave, but request pin is deasserted now. "
                      "We will try to process this pending transaction anyway.");

    bool processing = true;

    while(processing)
    {
        switch(changes)
        {
          case REQUEST_LINE_ASSERTED:
          case REQUEST_LINE_ASSERTED_AND_DEASSERTED:
            switch(transaction->request_state)
            {
              case REQSTATE_IDLE:
                transaction->request_state = REQSTATE_LOCKED;

                if(transaction->state == TR_IDLE)
                {
                    /* assertion of request line starts slave transaction */
                    process_transaction(transaction, rldata,
                                        fifo_in_fd, fifo_out_fd,
                                        spi_fd, spi_timeout_ms);
                }
                else
                    msg_vinfo(MESSAGE_LEVEL_DIAG,
                              "Transaction 0x%04x interrupted by slave request",
                              transaction->serial);

                break;

              case REQSTATE_RELEASED:
              case REQSTATE_MISSED:
                transaction->request_state = REQSTATE_NEXT_PENDING;

                msg_vinfo(MESSAGE_LEVEL_DIAG,
                          "Pending slave request while processing transaction 0x%04x",
                          transaction->serial);

                break;

              case REQSTATE_LOCKED:
                transaction->request_state = REQSTATE_NEXT_PENDING;

                BUG("Slave request while processing locked transaction 0x%04x",
                    transaction->serial);

                break;

              case REQSTATE_NEXT_PENDING:
                transaction->request_state = REQSTATE_MISSED;

                BUG("Slave request while processing transaction 0x%04x with pending transaction",
                    transaction->serial);

                break;
            }

            if(changes == REQUEST_LINE_ASSERTED_AND_DEASSERTED)
                changes = REQUEST_LINE_DEASSERTED;
            else
                processing = false;

            break;

          case REQUEST_LINE_DEASSERTED:
          case REQUEST_LINE_DEASSERTED_AND_ASSERTED:
            if(transaction->state == TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT)
                msg_info("Slave has deasserted the request pin%s",
                         changes == REQUEST_LINE_DEASSERTED_AND_ASSERTED
                         ? " (and has asserted it again)"
                         : "");

            switch(transaction->request_state)
            {
              case REQSTATE_LOCKED:
                /* deassertion of request line signals that slave is ready for
                 * next transaction after this one has been completed */
                transaction->request_state = REQSTATE_RELEASED;
                need_more_processing = (changes == REQUEST_LINE_DEASSERTED);

                break;

              case REQSTATE_NEXT_PENDING:
                /* slave requested a transaction, but then gave up */
                transaction->request_state = REQSTATE_MISSED;

                msg_error(0, LOG_WARNING,
                          "Lost slave request while processing transaction 0x%04x",
                          transaction->serial);

                break;

              case REQSTATE_IDLE:
                BUG("Deasserted idle transaction 0x%04x", transaction->serial);

                break;

              case REQSTATE_RELEASED:
                transaction->state = TR_IDLE;

                BUG("Deasserted released transaction 0x%04x", transaction->serial);

                break;

              case REQSTATE_MISSED:
                transaction->state = TR_IDLE;

                BUG("Deasserted released transaction 0x%04x with lost transaction request(s)",
                    transaction->serial);

                break;
            }

            if(changes == REQUEST_LINE_DEASSERTED_AND_ASSERTED)
                changes = REQUEST_LINE_ASSERTED;
            else
                processing = false;

            break;
        }
    }

    stats_context_switch(prev_ctx);

    return need_more_processing;
}

static bool wait_for_events(const struct dcp_transaction *const transaction,
                            const int gpio_fd, int fifo_in_fd,
                            bool is_quick_check, struct pollfd *fds)
{
    if(transaction->state == TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT)
    {
        msg_info("Waiting for slave to deassert the request pin");
        fifo_in_fd = -1;
    }

    if(gpio_fd < 0 && fifo_in_fd < 0)
        BUG("No fds to wait for");

    fds[0].fd = gpio_fd;
    fds[0].events = POLLPRI | POLLERR;
    fds[1].fd = fifo_in_fd;
    fds[1].events = POLLIN;

    struct stats_context *prev_ctx =
        stats_context_switch(STATISTICS_STRUCT(wait_for_events));

    const int ret = os_poll(fds, EVENT_FD_COUNT, is_quick_check ? 0 : -1);

    stats_context_switch(prev_ctx);

    if(ret > 0)
    {
        if(fds[0].fd >= 0 && (fds[0].revents & POLLPRI) != 0)
            msg_vinfo(MESSAGE_LEVEL_TRACE, "*** GPIO poll(2) event ***");

        return true;
    }

    if(ret == 0)
    {
        if(!is_quick_check)
            msg_error(errno, LOG_WARNING, "poll() unexpected timeout");
    }
    else if(errno != EINTR)
        msg_error(errno, LOG_CRIT, "poll() failed");

    return false;
}

static bool wait_for_dcp_data(struct dcp_transaction *transaction,
                              const int fifo_in_fd, const int fifo_out_fd,
                              const int spi_fd, unsigned int spi_timeout_ms,
                              struct slave_request_and_lock_data *rldata)
{
    struct pollfd fds[EVENT_FD_COUNT];

    if(!wait_for_events(transaction, rldata->gpio_fd, fifo_in_fd, false, fds))
        return true;

    if(fds[0].revents & POLLPRI)
    {
        if(process_request_line(transaction, rldata,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms))
            process_transaction(transaction, rldata,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms);
    }

    if(fds[0].revents & ~(POLLPRI | POLLERR))
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on gpio_fd %d: %04x",
                  fds[0].fd, fds[0].revents);

    bool keep_running = true;

    if(fds[1].revents & POLLHUP)
    {
        msg_error(0, LOG_ERR, "DCP daemon died, terminating");
        keep_running = false;
    }

    if(fds[1].revents & POLLIN)
    {
        if(expecting_dcp_data(transaction))
            process_transaction(transaction, rldata,
                                fifo_in_fd, fifo_out_fd,
                                spi_fd, spi_timeout_ms);
    }

    if(fds[1].revents & ~POLLIN)
        msg_error(0, LOG_WARNING,
                  "Unexpected poll() events on fifo_fd %d: %04x",
                  fifo_in_fd, fds[1].revents);

    return keep_running;
}

bool dcpspi_process(const int fifo_in_fd, const int fifo_out_fd,
                    const int spi_fd,
                    struct dcp_transaction *const transaction,
                    struct slave_request_and_lock_data *const rldata)
{
    stats_context_switch(STATISTICS_STRUCT(busy_unspecific));

    static const unsigned int spi_timeout_ms = 5000;

    if(rldata->is_running_for_real)
    {
        struct pollfd fds[EVENT_FD_COUNT];

        if(wait_for_events(transaction, rldata->gpio_fd, -1, true, fds) &&
           (fds[0].revents & POLLPRI) != 0)
        {
            process_request_line(transaction, rldata,
                                 fifo_in_fd, fifo_out_fd,
                                 spi_fd, spi_timeout_ms);
        }
    }

    if(expecting_dcp_data(transaction) || expecting_gpio_change(transaction))
        return wait_for_dcp_data(transaction,
                                 fifo_in_fd, fifo_out_fd,
                                 spi_fd, spi_timeout_ms, rldata);

    process_transaction(transaction, rldata,
                        fifo_in_fd, fifo_out_fd,
                        spi_fd, spi_timeout_ms);

    return true;
}
