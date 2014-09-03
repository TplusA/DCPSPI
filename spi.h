#ifndef SPI_H
#define SPI_H

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
 * Send buffer as is over SPI.
 */
int spi_send_buffer(int fd, const uint8_t *buffer, size_t length);

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

#endif /* !SPI_H */
