#ifndef SPI_H
#define SPI_H

#include <inttypes.h>
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
 */
int spi_send_buffer(int fd, const uint8_t *buffer, size_t length,
                    unsigned int timeout_ms);

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

#ifdef __cplusplus
}
#endif

#endif /* !SPI_H */
