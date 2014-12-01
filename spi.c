#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <string.h>
#include <errno.h>
#include <assert.h>

#include "spi.h"
#include "spi_hw.h"
#include "dcpdefs.h"
#include "messages.h"
#include "os.h"

#define NUMBER_OF_BYTES_PER_SPI_TRANSFER  32U

/*!
 * SPI speed in Hz for all transfers.
 */
static const uint32_t spi_speed_hz = 128000U;

int spi_open_device(const char *devname)
{
    return spi_hw_open_device(devname);
}

void spi_close_device(int fd)
{
    return spi_hw_close_device(fd);
}

int spi_send_buffer(int fd, const uint8_t *buffer, size_t length,
                    unsigned int timeout_ms)
{
    const struct spi_ioc_transfer spi_transfer[] =
    {
        {
            .tx_buf = (unsigned long)buffer,
            .len = length,
            .speed_hz = spi_speed_hz,
            .bits_per_word = 8,
        },
    };

    int ret =
        spi_hw_do_transfer(fd, spi_transfer,
                           sizeof(spi_transfer) / sizeof(spi_transfer[0]));

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

static bool skip_nops(const uint8_t *const src, size_t src_size,
                      size_t *const src_pos)
{
    for(/* nothing */; *src_pos < src_size; ++*src_pos)
    {
        if(src[*src_pos] != UINT8_MAX)
            return true;
    }

    return false;
}

static size_t filter_input(uint8_t *const buffer, size_t buffer_size,
                           bool *const pending_escape_sequence)
{
    size_t src_pos = 0;

    if(!skip_nops(buffer, buffer_size, &src_pos))
        return 0;

    size_t dest_pos = 0;

    if(*pending_escape_sequence)
    {
        buffer[dest_pos++] = unescape_byte(buffer[src_pos++]);
        *pending_escape_sequence = false;
    }

    while(skip_nops(buffer, buffer_size, &src_pos))
    {
        const uint8_t ch = buffer[src_pos++];

        if(ch == DCP_ESCAPE_CHARACTER)
        {
            ++src_pos;

            if(!skip_nops(buffer, buffer_size, &src_pos))
            {
                *pending_escape_sequence = true;
                return dest_pos;
            }

            buffer[dest_pos++] = unescape_byte(buffer[src_pos++]);
        }
        else
            buffer[dest_pos++] = ch;
    }

    return dest_pos;
}

static void compute_expiration_time(struct timespec *t, unsigned int timeout_ms)
{
    os_clock_gettime(CLOCK_MONOTONIC_RAW, t);

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

/*!
 * Read a few, fixed number of bytes from SPI, return them filtered.
 *
 * The buffer size must be at least #NUMBER_OF_BYTES_PER_SPI_TRANSFER bytes.
 */
static ssize_t read_chunk(int fd, uint8_t *buffer,
                          bool *const pending_escape_sequence)
{
    /* kernel spidev driver defaults to writing hard-coded 0's in case we
     * don't pass a tx_buf, but we need 0xff */
    static const uint8_t dummy_bytes[NUMBER_OF_BYTES_PER_SPI_TRANSFER] =
    {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };

    const struct spi_ioc_transfer spi_transfer[] =
    {
        {
            .tx_buf = (unsigned long)dummy_bytes,
            .rx_buf = (unsigned long)buffer,
            .len = sizeof(dummy_bytes),
            .speed_hz = spi_speed_hz,
            .bits_per_word = 8,
        },
    };

    int ret =
        spi_hw_do_transfer(fd, spi_transfer,
                           sizeof(spi_transfer) / sizeof(spi_transfer[0]));

    if(ret < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed reading %u bytes from SPI device fd %d",
                  spi_transfer[0].len, fd);
        return -1;
    }

    return filter_input(buffer, sizeof(dummy_bytes), pending_escape_sequence);
}

static size_t consume_from_buffer(uint8_t *src, size_t *src_size,
                                  uint8_t *dest, size_t dest_size)
{
    if(*src_size == 0 || dest_size == 0)
        return 0;

    const size_t consumed = *src_size < dest_size ? *src_size : dest_size;

    memcpy(dest, src, consumed);

    if(consumed < *src_size)
    {
        memmove(src, src + consumed, *src_size - consumed);
        *src_size -= consumed;
    }
    else
        *src_size = 0;

    return consumed;
}

static struct
{
    uint8_t input_buffer[NUMBER_OF_BYTES_PER_SPI_TRANSFER];
    size_t input_buffer_pos;
    bool pending_escape_sequence;
}
spi_read_globals;

ssize_t spi_read_buffer(int fd, uint8_t *buffer, size_t length,
                        unsigned int timeout_ms)
{
    struct timespec expiration_time;
    compute_expiration_time(&expiration_time, timeout_ms);

    /* first consume bytes from the buffer, if any */
    size_t output_buffer_pos =
        consume_from_buffer(spi_read_globals.input_buffer,
                            &spi_read_globals.input_buffer_pos,
                            buffer, length);

    while(output_buffer_pos < length)
    {
        struct timespec current_time;
        os_clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);

        if(timeout_expired(&expiration_time, &current_time))
        {
            msg_error(0, LOG_NOTICE,
                      "SPI read timeout, returning %zu of %zu bytes",
                      output_buffer_pos, length);
            break;
        }

        assert(spi_read_globals.input_buffer_pos == 0);

        /* read a few bytes from SPI into our buffer (with escape characters
         * removed); keep them around for potential extra bytes that have been
         * read, but were not requested by the caller (we cannot "unread" on
         * SPI) */
        const ssize_t chunk_size =
            read_chunk(fd, spi_read_globals.input_buffer,
                       &spi_read_globals.pending_escape_sequence);

        /* error out in case of hard communication error and return what got so
         * far */
        if(chunk_size < 0)
            break;

        /* slave not ready, try again... */
        if(chunk_size == 0)
            continue;

        /* got something */
        assert((size_t)chunk_size <= sizeof(spi_read_globals.input_buffer));

        spi_read_globals.input_buffer_pos += chunk_size;

        output_buffer_pos +=
            consume_from_buffer(spi_read_globals.input_buffer,
                                &spi_read_globals.input_buffer_pos,
                                buffer + output_buffer_pos,
                                length - output_buffer_pos);
    }

    return output_buffer_pos;
}

void spi_reset(void)
{
    memset(&spi_read_globals, 0, sizeof(spi_read_globals));
}
