/*
 * Copyright (C) 2015, 2016, 2018, 2019, 2022  T+A elektroakustik GmbH & Co. KG
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

#ifndef SPI_H
#define SPI_H

#include <inttypes.h>
#include <stdbool.h>
#include <unistd.h>

#include "statistics.h"

enum SpiSendResult
{
    SPI_SEND_RESULT_OK,
    SPI_SEND_RESULT_FAILURE,
    SPI_SEND_RESULT_TIMEOUT,
    SPI_SEND_RESULT_COLLISION,
};

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Open SPI device by name.
 *
 * \returns A file descriptor, or -1 on error.
 */
int spi_open_device(const char *devname);

/*!
 * Close SPI device specified by file descriptor.
 */
void spi_close_device(int fd);

/*!
 * Copy buffer content, escape special characters according to DCP specs.
 *
 * \returns
 *     The number of bytes written to \p dest.
 */
size_t spi_fill_buffer_from_raw_data(uint8_t *dest, size_t dest_size,
                                     const uint8_t *src, size_t src_size);

/*!
 * Send buffer as is over SPI.
 *
 * \param fd
 *     File descriptor of SPI device.
 *
 * \param buffer, length
 *     Buffer to send and its size.
 *
 * \param io
 *     Structure for gathering I/O statistics.
 *
 * \retval #SPI_SEND_RESULT_OK         On success.
 * \retval #SPI_SEND_RESULT_FAILURE    On unrecoverable, hard error.
 * \retval #SPI_SEND_RESULT_TIMEOUT    On timeout.
 * \retval #SPI_SEND_RESULT_COLLISION  On collision (slave requested start of a
 *                                     transaction).
 */
enum SpiSendResult spi_send_buffer(int fd, const uint8_t *buffer, size_t length,
                                   struct stats_io *io);

/*!
 * Fill buffer from SPI, but remove 0xff NOP bytes.
 *
 * This function attempts to read \p length clean bytes from SPI. Any NOP bytes
 * and escape sequences are removed from the input and the number of remaining
 * useful bytes is returned.
 *
 * If the buffer couldn't be filled with non-NOP bytes after exceeding a
 * timeout, the function returns prematurely with the number of bytes
 * actually read. That is, this function will return 0 in case only NOP bytes
 * were received.
 *
 * \returns Number of non-NOP bytes, or -1 on error.
 */
ssize_t spi_read_buffer(int fd, uint8_t *buffer, size_t length,
                        struct stats_io *io);

/*!
 * Check if there could be another packet in the internal receive buffer.
 *
 * Also move the content so that a possible next packet is aligned with the
 * beginning of the buffer. This function is useful to find out if there is any
 * useful pending data in the buffer that would otherwise be discarded during
 * the next slave transaction.
 */
bool spi_input_buffer_weed(void);

/*!
 * Begin new transaction, clear internal receive buffer.
 *
 * This just calls #spi_reset(), but it also prints a log message in case there
 * were any bytes left in the input buffer.
 */
void spi_new_transaction(void);

/*!
 * Reset internal state.
 *
 * In case of emergency during error recovery and for unit tests.
 */
void spi_reset(void);

/*!
 * Set clock speed for SPI transfers in Hz.
 */
void spi_set_speed_hz(uint32_t hz);

/*!
 * Dump all SPI traffic only at highest debug level.
 *
 * This is the default behavior.
 */
void spi_disable_traffic_dump(void);

/*!
 * Dump all SPI traffic despite of debug level.
 */
void spi_enable_traffic_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* !SPI_H */
