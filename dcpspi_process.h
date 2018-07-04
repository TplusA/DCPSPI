/*
 * Copyright (C) 2016, 2018  T+A elektroakustik GmbH & Co. KG
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

#include "statistics.h"

/*!
 * Current state of the DCP transaction.
 */
enum transaction_state
{
    TR_IDLE = 0,                                    /*!< Idle, waiting for activities. */

    /* master transactions */
    TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD,  /*!< Reading header from DCP process. */
    TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD,    /*!< Reading data from DCP process. */
    TR_MASTER_COMMAND_FORWARDING_TO_SLAVE,         /*!< Sending request to slave over SPI. */
    TR_MASTER_COMMAND_SKIPPING_DATA_FROM_DCPD,     /*!< Reading and ignoring DCP data. */
    TR_MASTER_COMMAND_SKIPPED,                     /*!< Command skipped. */

    /* slave transactions */
    TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE,  /*!< Reading header from slave over SPI. */
    TR_SLAVE_COMMAND_RECEIVING_DATA_FROM_SLAVE,    /*!< Reading data from slave over SPI. */
    TR_SLAVE_COMMAND_FORWARDING_TO_DCPD,           /*!< Sending request to DCP process. */
    TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT,    /*!< Wait for slave to deassert request. */
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
 * Effects of request line changes on transaction.
 */
enum transaction_request_state
{
    /*!
     * No specific request state for idle and master transactions.
     *
     * Master transactions and slave transactions triggered by SPI collisions
     * start in this state. Subsequent sampling of request GPIO state is used
     * to transition to other states.
     */
    REQSTATE_IDLE = 0,

    /*!
     * Request line was asserted for the first time while the transaction was
     * running and has not been deasserted the last time we looked.
     *
     * Slave transactions are started when the request line is asserted and
     * always begin in state #REQSTATE_LOCKED if triggered by GPIO request.
     * They cannot finish before the request line has been deasserted, even in
     * case the transaction has been processed internally already. This case
     * occurs if dcpspi manages to complete a slave transaction sooner than the
     * slave releases the request line.
     */
    REQSTATE_LOCKED,

    /*!
     * Request line was deasserted while the transaction was running.
     *
     * Slave transactions in this state may finish as soon as the transaction
     * has been processed internally.
     */
    REQSTATE_RELEASED,

    /*!
     * Request line was asserted a second time (or more often) while the
     * transaction was running.
     *
     * It is possible and common for the slave to request the next transaction
     * while the current transaction is still in progress. This state is used
     * to store this situation and to take it over to the next slave request.
     *
     * This state is basically #REQSTATE_LOCKED, but stored for the next
     * transaction.
     */
    REQSTATE_NEXT_PENDING,

    /*!
     * Request line was deasserted a second time (or more often) while the
     * transaction was running.
     *
     * Slave has requested a transaction, but timed out. It is not interested
     * anymore and we have lost a slave transaction.
     *
     * This state is like #REQSTATE_RELEASED, but with the extra information
     * that we have lost a transaction request.
     */
    REQSTATE_MISSED,
};

/*!
 * State of the DCP transaction in progress.
 *
 * This structure contains all there is to know about a transaction. A
 * transaction has a state of type #transaction_state, where #TR_IDLE means
 * that there is no transaction going on.
 *
 * The request line state is read out at certain points in the program. This
 * state is stored inside the transaction as \e request \e state of type
 * #transaction_request_state. The idle transaction, for instance, is in
 * request state #REQSTATE_IDLE, and a slave transaction that has just been
 * started via GPIO request is in request state #REQSTATE_LOCKED.
 *
 * As soon as the request line is asserted, the idle transaction is set to
 * request state #REQSTATE_LOCKED and it gets processed, meaning that its state
 * is propagated to #TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE and data is
 * read from SPI. The transaction is then called a \e slave \e transaction.
 * When the request line is deasserted while the transaction is processed, its
 * request state is propagated to #REQSTATE_RELEASED. The transaction is not
 * considered finished as long as the state remains #REQSTATE_LOCKED, and if
 * necessary, the transaction processing code will wait for the
 * #REQSTATE_RELEASED state even if the transaction has otherwise been
 * completely processed.
 *
 * Otherwise, if the named pipe from DCPD contains any data, but the request
 * line has not been asserted yet, the idle transaction is propagated to state
 * #TR_MASTER_COMMAND_RECEIVING_HEADER_FROM_DCPD. In this case, the request
 * state remains #REQSTATE_IDLE and the transaction is called a \e master
 * \e transaction. Request line changes can interrupt master transactions until
 * to the point the data is actually written to SPI. The request line is not
 * considered while writing master transaction data, unless an SPI collision is
 * detected (slave responds with non-zero, non-NOP bytes) or an SPI slave
 * timeout occurs (slave constantly responds with NOP bytes).
 *
 * An SPI collision causes the master transaction to be aborted and a NACK
 * being sent upstream. It also triggers the start of a new slave transaction
 * which will start in request state #REQSTATE_IDLE (because the GPIO was not
 * used to start the transaction). The slave is expected to have the GPIO
 * asserted when the collision happens, but the sampling of the GPIO happens at
 * very specific points in the program. This code will run after the collision
 * has been detected and takes care of correct state transitions.
 *
 * An SPI timeout causes the master transaction to be aborted with a failure.
 * The master transaction ends with a NACK sent upstream. The master
 * transaction may be repeated until the transaction TTL drops to zero, in
 * which case the master transaction is completely discarded.
 */
struct dcp_transaction
{
    enum transaction_state state;

    struct buffer dcp_buffer;
    struct buffer spi_buffer;

    uint16_t serial;
    uint8_t ttl;

    uint16_t pending_size_of_transaction;
    size_t flush_to_dcpd_buffer_pos;
    bool pending_escape_sequence_in_spi_buffer;

    /*!
     * Request line state changes while the transaction is processed.
     */
    enum transaction_request_state request_state;
};

struct slave_request_and_lock_data
{
    const bool is_running_for_real;
    const struct gpio_handle *gpio;
    const int gpio_fd;

    bool previous_gpio_state;
};

struct program_statistics
{
    bool is_enabled;
    struct stats_context busy_unspecific;
    struct stats_context busy_gpio;
    struct stats_context busy_transaction;
    struct stats_context wait_for_events;
    struct stats_io spi_transfers;
    struct stats_io dcpd_reads;
    struct stats_io dcpd_writes;
};

#ifdef __cplusplus
extern "C" {
#endif

bool reset_transaction_struct(struct dcp_transaction *transaction,
                              bool is_initial_reset);

void dcpspi_init(void);

void dcpspi_statistics_reset(void);
const struct program_statistics *dcpspi_statistics_get(void);
bool dcpspi_statistics_enable(bool enable);

bool dcpspi_process(const int fifo_in_fd, const int fifo_out_fd,
                    const int spi_fd,
                    struct dcp_transaction *const transaction,
                    struct slave_request_and_lock_data *const rldata);

#ifdef __cplusplus
}
#endif

#endif /* !DCPSPI_PROCESS_H */
