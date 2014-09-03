#define _XOPEN_SOURCE 500

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include <linux/spi/spidev.h>

#include "spi.h"
#include "dcpdefs.h"
#include "messages.h"

int spi_open_device(const char *devname)
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

void spi_close_device(int fd)
{
    int ret;

    while((ret = close(fd)) < 0 && errno == EINTR)
        ;

    if(ret < 0)
        msg_error(errno, LOG_ERR, "Failed closing SPI device fd %d", fd);
}

int spi_send_buffer(int fd, const uint8_t *buffer, size_t length)
{
    const struct spi_ioc_transfer spi_transfer[] =
    {
        {
            .tx_buf = (unsigned long)buffer,
            .len = length,
            .speed_hz = 128000,
            .bits_per_word = 8,
        },
    };

    int ret =
        ioctl(fd,
              SPI_IOC_MESSAGE(sizeof(spi_transfer) / sizeof(spi_transfer[0])),
              spi_transfer);

    if(ret < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed writing %zu bytes to SPI device fd %d", length, fd);
        return -1;
    }
    else
        return 0;
}

static inline uint8_t unescape_byte(uint8_t byte)
{
    return byte == 0x01 ? UINT8_MAX : byte;
}

static bool skip_nops(const uint8_t *restrict src, size_t src_size,
                      size_t *restrict src_pos)
{
    for(/* nothing */; *src_pos < src_size; ++*src_pos)
    {
        if(src[*src_pos] != UINT8_MAX)
            return true;
    }

    return false;
}

static size_t filtered_copy(const uint8_t *restrict src, size_t src_size,
                            size_t *restrict src_pos,
                            bool *restrict pending_escape_sequence,
                            uint8_t *restrict dest, size_t dest_size)
{
    if(!skip_nops(src, src_size, src_pos))
        return 0;

    size_t dest_pos = 0;

    if(*pending_escape_sequence)
    {
        dest[dest_pos++] = unescape_byte(src[*src_pos++]);
        *pending_escape_sequence = false;
    }

    while(skip_nops(src, src_size, src_pos))
    {
        const uint8_t ch = src[*src_pos++];

        if(ch == DCP_ESCAPE_CHARACTER)
        {
            ++*src_pos;

           if(!skip_nops(src, src_size, src_pos))
            {
                *pending_escape_sequence = true;
                return dest_pos;
            }

            dest[dest_pos++] = unescape_byte(src[*src_pos++]);
        }
        else
            dest[dest_pos++] = ch;
    }

    return dest_pos;
}

static void compute_expiration_time(struct timespec *t, unsigned int timeout_ms)
{
    clock_gettime(CLOCK_MONOTONIC_RAW, t);

    const unsigned long timeout_remainder_ns =
        (timeout_ms % 1000U) * 1000ULL * 1000ULL;

    t->tv_sec += timeout_ms / 1000U;

    if(timeout_remainder_ns < 1000ULL * 1000ULL * 1000ULL - t->tv_nsec)
        t->tv_nsec += timeout_remainder_ns;
    else
    {
        ++t->tv_sec;
        t->tv_nsec =
            timeout_remainder_ns - (1000ULL * 1000ULL * 1000ULL - t->tv_nsec);
    }
}

static bool timeout_expired(const struct timespec *restrict timeout,
                            const struct timespec *restrict current)
{
    if(current->tv_sec > timeout->tv_sec)
        return true;
    else if(current->tv_sec < timeout->tv_sec)
        return false;

    return current->tv_nsec >= timeout->tv_nsec;
}

ssize_t spi_read_buffer(int fd, uint8_t *buffer, size_t length,
                        unsigned int timeout_ms)
{
    struct timespec expiration_time;
    compute_expiration_time(&expiration_time, timeout_ms);

    size_t buffer_pos = 0;
    bool pending_escape_sequence = false;

    while(buffer_pos < length)
    {
        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);

        if(timeout_expired(&expiration_time, &current_time))
        {
            msg_error(0, LOG_NOTICE,
                      "SPI read timeout, returning %zd of %zu bytes",
                      buffer_pos, length);
            return buffer_pos;
        }

        static uint8_t spi_input_buffer[64];
        static size_t spi_input_buffer_pos;

        if(spi_input_buffer_pos >= sizeof(spi_input_buffer))
        {
            const struct spi_ioc_transfer spi_transfer[] =
            {
                {
                    .rx_buf = (unsigned long)spi_input_buffer,
                    .len = sizeof(spi_input_buffer),
                    .speed_hz = 128000,
                    .bits_per_word = 8,
                },
            };

            int ret =
                ioctl(fd,
                      SPI_IOC_MESSAGE(sizeof(spi_transfer) / sizeof(spi_transfer[0])),
                      spi_transfer);

            if(ret < 0)
            {
                msg_error(errno, LOG_EMERG,
                          "Failed reading %u bytes from SPI device fd %d",
                          spi_transfer[0].len, fd);
                return -1;
            }

            spi_input_buffer_pos = 0;
        }

        buffer_pos +=
            filtered_copy(spi_input_buffer, sizeof(spi_input_buffer),
                          &spi_input_buffer_pos, &pending_escape_sequence,
                          buffer + buffer_pos, length - buffer_pos);
    }

    return buffer_pos;
}
