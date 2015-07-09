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

#ifndef SPI_H
#define SPI_H

#include <inttypes.h>
#include <stdbool.h>
#include <unistd.h>

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
 * \param timeout_ms
 *     The maximum amount of time to wait for the slave to get ready in
 *     milliseconds.
 *
 * \param is_slave_interrupting
 *     After each check for slave ready state, this function is called to check
 *     if the slave has asserted the request line. If it has, then there is a
 *     collision and sending the buffer is aborted. We are doing this through a
 *     callback function for decoupling SPI from GPIO and possibly more state
 *     checks to come.
 *
 * \param user_data
 *     Pointer to opaque data passed to \p is_slave_interrupting().
 *
 * \retval 0  On success.
 * \retval 1  On collision (slave requested start of a transaction).
 * \retval -1 On hard error or timeout.
 */
int spi_send_buffer(int fd, const uint8_t *buffer, size_t length,
                    unsigned int timeout_ms,
                    bool (*is_slave_interrupting)(void *data),
                    void *user_data);

/*!
 * Fill buffer from SPI, but remove 0xff NOP bytes.
 *
 * This function attempts to read \p length clean bytes from SPI. Any NOP bytes
 * and escape sequences are removed from the input and the number of remaining
 * useful bytes is returned.
 *
 * If the buffer couldn't be filled with non-NOP bytes after \p timeout_ms
 * milliseconds, the function returns prematurely with the number of bytes
 * actually read. That is, this function will return 0 in case only NOP bytes
 * were received.
 *
 * \returns Number of non-NOP bytes, or -1 on error.
 */
ssize_t spi_read_buffer(int fd, uint8_t *buffer, size_t length,
                        unsigned int timeout_ms);

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

#ifdef __cplusplus
}
#endif

#endif /* !SPI_H */
