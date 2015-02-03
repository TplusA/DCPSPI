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

#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

#include "spi_hw.h"
#include "spi.h"
#include "messages.h"

int spi_hw_open_device(const char *devname)
{
    int fd;

    while((fd = open(devname, O_RDWR | O_SYNC)) < 0 &&  errno == EINTR)
        ;

    if(fd < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed opening SPI device \"%s\"", devname);
        return -1;
    }

    static const uint32_t spi_mode = SPI_MODE_0;

    if(ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed setting SPI mode %u on device \"%s\"",
                  spi_mode, devname);
        goto error_set_mode;
    }

    return fd;

error_set_mode:
    spi_close_device(fd);
    return -1;
}

void spi_hw_close_device(int fd)
{
    int ret;

    while((ret = close(fd)) < 0 && errno == EINTR)
        ;

    if(ret < 0)
        msg_error(errno, LOG_ERR, "Failed closing SPI device fd %d", fd);
}

int spi_hw_do_transfer(int fd, const struct spi_ioc_transfer spi_transfer[],
                       size_t number_of_fragments)
{
    return ioctl(fd, SPI_IOC_MESSAGE(number_of_fragments), spi_transfer);
}
