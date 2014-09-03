/*
 * Copyright (C) 2015, 2019  T+A elektroakustik GmbH & Co. KG
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
