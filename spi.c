/*
 * Copyright (C) 2015, 2016  T+A elektroakustik GmbH & Co. KG
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

#include <string.h>
#include <errno.h>

#include "spi.h"
#include "spi_hw.h"
#include "dcpdefs.h"
#include "messages.h"
#include "hexdump.h"
#include "os.h"

/*!
 * SPI speed in Hz for all transfers.
 */
static uint32_t spi_speed_hz = 900U * 1000U;

/*!
 * Kernel spidev driver defaults to writing hard-coded 0's in case we
 * don't pass a tx_buf, but we need 0xff.
 */
static const uint8_t spi_dummy_bytes[32] =
{
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

struct spi_input_buffer
{
    uint8_t buffer[sizeof(spi_dummy_bytes)];
    size_t buffer_pos;
    bool pending_escape_sequence;
};

int spi_open_device(const char *devname)
{
    return spi_hw_open_device(devname);
}

void spi_close_device(int fd)
{
    return spi_hw_close_device(fd);
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

static enum SpiSendResult
wait_for_spi_slave(int fd, unsigned int timeout_ms,
                   uint8_t *const buffer, const size_t buffer_size,
                   bool *have_significant_data)
{
    const struct spi_ioc_transfer spi_transfer[] =
    {
        {
            .tx_buf = (unsigned long)spi_dummy_bytes,
            .rx_buf = (unsigned long)buffer,
            .len = buffer_size,
            .speed_hz = spi_speed_hz,
            .bits_per_word = 8,
        },
    };

    *have_significant_data = false;

    struct timespec expiration_time;
    compute_expiration_time(&expiration_time, timeout_ms);

    while(1)
    {
        int ret =
            spi_hw_do_transfer(fd, spi_transfer,
                               sizeof(spi_transfer) / sizeof(spi_transfer[0]));

        if(ret < 0)
        {
            msg_error(errno, LOG_EMERG,
                      "Failed waiting for slave device on fd %d", fd);
            return SPI_SEND_RESULT_FAILURE;
        }

        hexdump_to_log(MESSAGE_LEVEL_TRACE, buffer, buffer_size, "Received");

        for(size_t i = 0; i < buffer_size; ++i)
        {
            if(buffer[i] == 0)
                return SPI_SEND_RESULT_OK;

            if(buffer[i] != UINT8_MAX)
            {
                msg_error(0, LOG_NOTICE, "Collision detected (got funny poll bytes)");
                *have_significant_data = true;
                return SPI_SEND_RESULT_COLLISION;
            }
        }

        /* only NOPs, try again if we are within the specified timeout... */
        struct timespec current_time;
        os_clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);

        if(timeout_expired(&expiration_time, &current_time))
        {
            msg_error(0, LOG_NOTICE,
                      "SPI write timeout, slave didn't get ready within %u ms",
                      timeout_ms);
            return SPI_SEND_RESULT_TIMEOUT;
        }

        /* give the slave (and ourselves) a break */
        static const struct timespec delay_between_slave_ready_probes =
        {
            .tv_nsec = 5L * 1000L * 1000L,
        };

        os_nanosleep(&delay_between_slave_ready_probes);
    }
}

static void handle_collision(uint8_t *const poll_bytes_buffer,
                             const size_t poll_bytes_buffer_size,
                             struct spi_input_buffer *in)
{
    bool pending_escape_sequence = false;
    const size_t bytes_left = filter_input(poll_bytes_buffer,
                                           poll_bytes_buffer_size,
                                           &pending_escape_sequence);

    if(in->buffer_pos > 0)
        BUG("Discarding %zu bytes from SPI receive buffer after collision",
            in->buffer_pos);

    if(bytes_left > 0)
    {
        memcpy(in->buffer, poll_bytes_buffer, bytes_left);
        hexdump_to_log(MESSAGE_LEVEL_DIAG,
                       in->buffer, bytes_left, "Colliding poll bytes");
    }

    in->buffer_pos = bytes_left;
    in->pending_escape_sequence = pending_escape_sequence;
}

static struct spi_input_buffer global_spi_input_buffer;

enum SpiSendResult spi_send_buffer(int fd, const uint8_t *buffer, size_t length,
                                   unsigned int timeout_ms)
{
    if(fd < 0)
    {
        /* operating in dummy mode */
        return SPI_SEND_RESULT_OK;
    }

    uint8_t poll_bytes_buffer[sizeof(spi_dummy_bytes) > 2 ? 2 : sizeof(spi_dummy_bytes)];
    bool have_significant_data;
    const enum SpiSendResult wait_result =
        wait_for_spi_slave(fd, timeout_ms,
                           poll_bytes_buffer, sizeof(poll_bytes_buffer),
                           &have_significant_data);

    if(wait_result != SPI_SEND_RESULT_OK)
    {
        if(wait_result == SPI_SEND_RESULT_COLLISION)
        {
            hexdump_to_log(MESSAGE_LEVEL_DIAG,
                           buffer, length, "Tried to send during collision");

            if(have_significant_data)
                hexdump_to_log(MESSAGE_LEVEL_DIAG,
                               poll_bytes_buffer, sizeof(poll_bytes_buffer),
                               "Received poll bytes during collision");
        }

        if(wait_result == SPI_SEND_RESULT_COLLISION && have_significant_data)
            handle_collision(poll_bytes_buffer, sizeof(poll_bytes_buffer),
                             &global_spi_input_buffer);

        return wait_result;
    }

    const struct spi_ioc_transfer spi_transfer[] =
    {
        {
            .tx_buf = (unsigned long)buffer,
            .len = length,
            .speed_hz = spi_speed_hz,
            .bits_per_word = 8,
        },
    };

    const int ret =
        spi_hw_do_transfer(fd, spi_transfer,
                           sizeof(spi_transfer) / sizeof(spi_transfer[0]));

    if(ret < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed writing %zu bytes to SPI device fd %d", length, fd);
        return SPI_SEND_RESULT_FAILURE;
    }
    else
    {
        hexdump_to_log(MESSAGE_LEVEL_TRACE, buffer, length, "Sent");
        return SPI_SEND_RESULT_OK;
    }
}

size_t spi_fill_buffer_from_raw_data(uint8_t *dest, size_t dest_size,
                                     const uint8_t *src, size_t src_size)
{
    size_t pos = 0;

    for(size_t i = 0; i < src_size && pos < dest_size; ++i)
    {
        const uint8_t ch = src[i];

        if(ch == UINT8_MAX)
        {
            dest[pos++] = DCP_ESCAPE_CHARACTER;
            if(pos < dest_size)
                dest[pos++] = 0x01;
        }
        else if(ch == DCP_ESCAPE_CHARACTER)
        {
            dest[pos++] = DCP_ESCAPE_CHARACTER;
            if(pos < dest_size)
                dest[pos++] = DCP_ESCAPE_CHARACTER;
        }
        else
            dest[pos++] = ch;
    }

    return pos;
}

/*!
 * Read a few, fixed number of bytes from SPI, return them filtered.
 *
 * The buffer size must be at least as big as the #spi_dummy_bytes array.
 */
static ssize_t read_chunk(int fd, struct spi_input_buffer *const in)
{
    const struct spi_ioc_transfer spi_transfer[] =
    {
        {
            .tx_buf = (unsigned long)spi_dummy_bytes,
            .rx_buf = (unsigned long)in->buffer,
            .len = sizeof(spi_dummy_bytes),
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

    hexdump_to_log(MESSAGE_LEVEL_TRACE,
                   in->buffer, sizeof(spi_dummy_bytes), "Received");

    return filter_input(in->buffer, sizeof(spi_dummy_bytes),
                        &in->pending_escape_sequence);
}

static size_t consume_from_buffer(struct spi_input_buffer *const restrict src,
                                  uint8_t *const restrict dest, size_t dest_size)
{
    if(src->buffer_pos == 0 || dest_size == 0)
        return 0;

    const size_t consumed =
        src->buffer_pos < dest_size ? src->buffer_pos : dest_size;

    if(dest != NULL)
        memcpy(dest, src->buffer, consumed);

    if(consumed < src->buffer_pos)
    {
        memmove(src->buffer, src->buffer + consumed,
                src->buffer_pos - consumed);
        src->buffer_pos -= consumed;
    }
    else
        src->buffer_pos = 0;

    return consumed;
}

ssize_t spi_read_buffer(int fd, uint8_t *buffer, size_t length,
                        unsigned int timeout_ms)
{
    bool reset_timeout = false;
    struct timespec expiration_time;
    compute_expiration_time(&expiration_time, timeout_ms);

    /* first consume bytes from the buffer, if any */
    size_t output_buffer_pos =
        consume_from_buffer(&global_spi_input_buffer, buffer, length);

    while(output_buffer_pos < length)
    {
        log_assert(global_spi_input_buffer.buffer_pos == 0);

        /* read a few bytes from SPI into our buffer (with escape characters
         * removed); keep them around for potential extra bytes that have been
         * read, but were not requested by the caller (we cannot "unread" on
         * SPI) */
        const ssize_t chunk_size = read_chunk(fd, &global_spi_input_buffer);

        /* error out in case of hard communication error and return what got so
         * far */
        if(chunk_size < 0)
            break;

        /* slave not ready, try again... */
        if(chunk_size == 0)
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

            if(reset_timeout)
            {
                /* got something, do not consider timeout anymore for fast
                 * failure when necessary */
                expiration_time = current_time;
                reset_timeout = false;
            }

            continue;
        }

        /* got something */
        log_assert((size_t)chunk_size <= sizeof(global_spi_input_buffer.buffer));

        reset_timeout = true;

        global_spi_input_buffer.buffer_pos += chunk_size;
        output_buffer_pos +=
            consume_from_buffer(&global_spi_input_buffer,
                                buffer + output_buffer_pos,
                                length - output_buffer_pos);
    }

    return output_buffer_pos;
}

bool spi_input_buffer_weed(void)
{
    if(global_spi_input_buffer.buffer_pos == 0)
        return false;

    /* first byte must be zero, otherwise it's junk */
    if(global_spi_input_buffer.buffer[0] != 0x00)
        return false;

    int cmd_idx;

    if(global_spi_input_buffer.buffer_pos > 0 && global_spi_input_buffer.buffer[1] != 0x00)
        cmd_idx = 1;
    else if(global_spi_input_buffer.buffer_pos > 1 && global_spi_input_buffer.buffer[2] != 0x00)
        cmd_idx = 2;
    else
        cmd_idx = -1;

    if(cmd_idx < 0)
        return false;

    if(global_spi_input_buffer.buffer[cmd_idx] > DCP_COMMAND_MULTI_READ_REGISTER)
        return false;

    consume_from_buffer(&global_spi_input_buffer, NULL, cmd_idx);

    return true;
}

void spi_new_transaction(void)
{
    if(global_spi_input_buffer.buffer_pos > 0)
    {
        msg_info("Discarding %zu bytes from SPI receive buffer",
                 global_spi_input_buffer.buffer_pos);
        hexdump_to_log(MESSAGE_LEVEL_DEBUG,
                       global_spi_input_buffer.buffer,
                       global_spi_input_buffer.buffer_pos, "Discarded buffer");
    }

    spi_reset();
}

void spi_reset(void)
{
    memset(&global_spi_input_buffer, 0, sizeof(global_spi_input_buffer));
}

void spi_set_speed_hz(uint32_t hz)
{
    if(hz > 0)
        spi_speed_hz = hz;
}
