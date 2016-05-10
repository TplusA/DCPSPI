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
