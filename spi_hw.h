#ifndef SPI_HW_H
#define SPI_HW_H

#include <unistd.h>
#include <linux/spi/spidev.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Open SPI device by name (target-specific code).
 *
 * \returns A file descriptor, or -1 on error.
 */
int spi_hw_open_device(const char *devname);

/*!
 * Close SPI device specified by file descriptor (target-specific code).
 */
void spi_hw_close_device(int fd);

/*!
 * Do an SPI transfer (target-specific code).
 */
int spi_hw_do_transfer(int fd, const struct spi_ioc_transfer spi_transfer[],
                       size_t number_of_fragments);

#ifdef __cplusplus
}
#endif

#endif /* !SPI_HW_H */
