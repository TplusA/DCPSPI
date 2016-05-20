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

#ifndef DCPSPI_PROCESS_H
#define DCPSPI_PROCESS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

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

/*!
 * Request line monitoring for slave transactions.
 */
enum slave_request_line_state_t
{
    REQ_NOT_REQUESTED = 0,  /*!< For idle and master transactions. */
    REQ_ASSERTED,           /*!< Slave transactions: Request line was asserted
                             *   the last time we looked. */
    REQ_DEASSERTED,         /*!< Slave transactions: Request line was
                             *   deasserted, transaction may finish. */
};

/*!
 * State of the DCP transaction in progress.
 *
 * This structure contains all there is to know about a transaction. A
 * transaction has a state of type #transaction_state, where #TR_IDLE means
 * that there is no transaction going on.
 *
 * The request line state is read out at certain points in the program. This
 * state is stored in the transaction as \e request \e line \e state of type
 * #slave_request_line_state_t. The idle transaction is in request line state
 * #REQ_NOT_REQUESTED.
 *
 * As soon as the request line is asserted, the idle transaction is set to
 * request line state #REQ_ASSERTED and it gets processed, meaning that its
 * state is propagated to #TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE and data is
 * read from SPI. The transaction is then called a \e slave \e transaction. If
 * the request line is deasserted while the transaction is processed, its
 * request line state is propagated to #REQ_DEASSERTED. The transaction is not
 * considered finished as long as the state remains #REQ_ASSERTED. If
 * necessary, the transaction processing code will wait for the #REQ_DEASSERTED
 * state even if the transaction has otherwise been completely processed.
 *
 * Otherwise, the idle transaction is propagated to state
 * #TR_MASTER_WRITECMD_RECEIVING_HEADER_FROM_DCPD in case the named pipe from
 * DCPD contains any data, but the request line has not been asserted. In this
 * case, the request line state remains #REQ_NOT_REQUESTED. The transaction is
 * then called a \e master \e transaction.
 *
 * As long as the SPI slave as has not signaled its ready state and thus
 * committed to any transaction, a master transaction can be interrupted by
 * asserting the request line. This may happen at any time before the ready
 * state has been clearly signaled by a short SPI poll phase because the slave
 * does not know anything about the transaction yet. The ready state poll phase
 * itself may also be interrupted by data that indicates non-ready state, which
 * is treated the same as an asserted request line at that point. Both cases
 * are called a \e collision. After the ready state has been signaled, however,
 * the request line is basically ignored and the master transaction is going to
 * be processed until its end.
 */
struct dcp_transaction
{
    enum transaction_state state;

    struct buffer dcp_buffer;
    struct buffer spi_buffer;

    uint16_t pending_size_of_transaction;
    size_t flush_to_dcpd_buffer_pos;
    bool pending_escape_sequence_in_spi_buffer;

    /*!
     * Request line state changes while the transaction is processed.
     */
    enum slave_request_line_state_t request_state;
};

struct collision_check_data
{
    const struct gpio_handle *gpio;
};

#ifdef __cplusplus
extern "C" {
#endif

void reset_transaction_struct(struct dcp_transaction *transaction);

bool dcpspi_process(const int fifo_in_fd, const int fifo_out_fd,
                    const int spi_fd, const int gpio_fd,
                    bool is_running_for_real,
                    struct dcp_transaction *const transaction,
                    struct buffer *const deferred_transaction_data,
                    struct collision_check_data *const ccdata,
                    bool *prev_gpio_state);

#ifdef __cplusplus
}
#endif

#endif /* !DCPSPI_PROCESS_H */
