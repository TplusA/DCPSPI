/*
 * Copyright (C) 2015, 2016, 2018, 2019  T+A elektroakustik GmbH & Co. KG
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

#include <cppcutter.h>
#include <array>

#include "spi.h"
#include "dcpdefs.h"
#include "hexdump.h"

#include "mock_messages.hh"
#include "mock_spi_hw.hh"
#include "mock_os.hh"
#include "spi_hw_data.hh"

/*!
 * \addtogroup spi_communication_tests Unit tests
 * \ingroup spi_communication
 *
 * SPI communication unit tests.
 */
/*!@{*/

/* Dummy implementation */
void hexdump_to_log(enum MessageVerboseLevel level,
                    const uint8_t *const buffer, size_t buffer_length,
                    const char *what)
{
    cppcut_assert_not_null(buffer);
    cppcut_assert_not_null(what);
}

namespace spi_communication_tests
{

static constexpr int expected_spi_fd = 42;
static constexpr unsigned long delay_between_slave_probes_ms = 5;

static MockMessages *mock_messages;
static MockOs *mock_os;
static MockSPIHW *mock_spi_hw;

static const auto mock_spi_transfer = mock_spi_tx_template<expected_spi_fd>;

void cut_setup()
{
    mock_messages = new MockMessages;
    cppcut_assert_not_null(mock_messages);
    mock_messages->init();
    mock_messages_singleton = mock_messages;

    mock_os = new MockOs;
    cppcut_assert_not_null(mock_os);
    mock_os->init();
    mock_os_singleton = mock_os;

    mock_spi_hw = new MockSPIHW;
    cppcut_assert_not_null(mock_spi_hw);
    mock_spi_hw->init();
    mock_spi_hw_singleton = mock_spi_hw;

    spi_rw_data = new spi_rw_data_t;
    cppcut_assert_not_null(spi_rw_data);

    spi_reset();
}

void cut_teardown()
{
    mock_messages->check();
    mock_os->check();
    mock_spi_hw->check();

    mock_messages_singleton = nullptr;
    mock_os_singleton = nullptr;
    mock_spi_hw_singleton = nullptr;

    delete mock_messages;
    delete mock_os;
    delete mock_spi_hw;

    mock_messages = nullptr;
    mock_os = nullptr;
    mock_spi_hw = nullptr;

    delete spi_rw_data;
    spi_rw_data = nullptr;
}

static size_t expect_spi_transfers(size_t number_of_bytes)
{
    const size_t count =
        number_of_bytes / read_from_slave_spi_transfer_size +
        ((number_of_bytes % read_from_slave_spi_transfer_size) == 0 ? 0 : 1);

    for(size_t i = 0; i < count; ++i)
        mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    return count;
}

static int return_nops(int fd, const struct spi_ioc_transfer spi_transfer[],
                       size_t number_of_fragments)
{
    for(size_t f = 0; f < number_of_fragments; ++f)
    {
        void *dest_buffer = reinterpret_cast<void *>(spi_transfer[f].rx_buf);

        if(dest_buffer != nullptr)
            memset(dest_buffer, UINT8_MAX, spi_transfer[f].len);
    }

    return 0;
}

static void ensure_empty_read_buffer()
{
    static const struct timespec t1 = { .tv_sec = 1000, .tv_nsec = 0, };
    static const struct timespec t2 = { .tv_sec = 1000, .tv_nsec = 1, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t1);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_messages->expect_msg_error(0, LOG_NOTICE,
                                    "SPI read timeout, returning %zu of %zu bytes");

    uint8_t should_remain_untouched[1024] = { 0 };

    const ssize_t bytes =
        spi_read_buffer(expected_spi_fd, should_remain_untouched,
                        sizeof(should_remain_untouched), 0, nullptr);

    if(bytes > 0)
    {
        /* produce a useful failure message */
        cut_assert_equal_memory(should_remain_untouched, 0,
                                should_remain_untouched, bytes);
    }
}

/*!\test
 * Regular happy case: just read some data from SPI slave, no escape character.
 *
 * There is no protocol parsing done at this point. This is pure copying of
 * bytes (modulo special handling of NOP and escape bytes, added only to take
 * care of the characteristics of the physical layer, just like the request
 * GPIO) and juggling with buffers.
 */
void test_read_from_spi()
{
    std::array<uint8_t, 100> expected_content;
    for(size_t i = 0; i < expected_content.size(); ++i)
        expected_content[i] = i + DCP_ESCAPE_CHARACTER + 1;

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, expected_content);
    expect_spi_transfers(expected_content.size());

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 100> buffer;
    buffer.fill(0xab);

    cppcut_assert_equal(ssize_t(buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 500,
                                        nullptr));

    cut_assert_equal_memory(expected_content.data(), expected_content.size(),
                            buffer.data(), buffer.size());

    ensure_empty_read_buffer();
}

/*!\test
 * Read some data from SPI slave with escape characters inside.
 */
void test_read_escaped_data_from_spi()
{
    static const std::array<uint8_t, 27> escaped_data =
    {
        0x00,                                           /* regular byte */
        0x01,                                           /* regular byte */
        0x02,                                           /* regular byte */
        UINT8_MAX - 1U,                                 /* regular byte */
        UINT8_MAX,                                      /* NOP */

        DCP_ESCAPE_CHARACTER, 0x00,                     /* escaped regular */
        DCP_ESCAPE_CHARACTER, 0x01,                     /* escaped NOP */
        DCP_ESCAPE_CHARACTER, 0x02,                     /* escaped regular */
        DCP_ESCAPE_CHARACTER, DCP_ESCAPE_CHARACTER,     /* escaped escape */
        DCP_ESCAPE_CHARACTER, UINT8_MAX - 1U,           /* escaped regular */
        DCP_ESCAPE_CHARACTER, UINT8_MAX, 0x03,          /* escaped regular (NOP
                                                         *   is filtered) */
        UINT8_MAX, UINT8_MAX, UINT8_MAX,                /* evil sequence of */
        DCP_ESCAPE_CHARACTER, UINT8_MAX, UINT8_MAX,     /*   NOPs and escape */
        DCP_ESCAPE_CHARACTER, UINT8_MAX, UINT8_MAX,     /*   characters */
    };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, escaped_data);
    expect_spi_transfers(escaped_data.size());

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 11> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 500,
                                        nullptr));

    static const std::array<uint8_t, 11> expected_content =
    {
        0x00, 0x01, 0x02, UINT8_MAX - 1U,
        0x00, UINT8_MAX, 0x02, DCP_ESCAPE_CHARACTER, UINT8_MAX - 1U, 0x03,
        DCP_ESCAPE_CHARACTER,
    };

    cut_assert_equal_memory(expected_content.data(), expected_content.size(),
                            buffer.data(), buffer.size());

    ensure_empty_read_buffer();
}

/*!\test
 * Write data in two short SPI transfers so that first transfer ends with an
 * escape character.
 */
void test_read_escaped_data_from_spi_with_last_character_in_first_chunk_is_escape()
{
    /* 64 bytes, escape at offset 31 (last byte of first chunk), escaped data
     * at offset 32 (first byte of second chunk) */
    std::array<uint8_t, 2 * read_from_slave_spi_transfer_size> data = {0};
    data[read_from_slave_spi_transfer_size - 1] = DCP_ESCAPE_CHARACTER;
    data[read_from_slave_spi_transfer_size] = 0x01;

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, data);
    cppcut_assert_equal(size_t(2), expect_spi_transfers(data.size()));

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    /* should receive 63 bytes */
    std::array<uint8_t, 2 * read_from_slave_spi_transfer_size - 1> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 500,
                                        nullptr));

    std::array<uint8_t, 2 * read_from_slave_spi_transfer_size - 1> expected_content = {0};
    expected_content[read_from_slave_spi_transfer_size - 1] = UINT8_MAX;

    cut_assert_equal_memory(expected_content.data(), expected_content.size(),
                            buffer.data(), buffer.size());

    ensure_empty_read_buffer();
}

static void expect_spi_slave_gets_ready()
{
    /* let's assume get ready signal after one cycle */
    static const std::array<uint8_t, wait_for_slave_spi_transfer_size> slave_gets_ready =
    {
        UINT8_MAX,
    };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, slave_gets_ready, true);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
}

/*!\test
 * Regular happy case: just write some data to SPI slave, no escape character.
 */
void test_write_to_spi()
{
    expect_spi_slave_gets_ready();

    std::array<uint8_t, 100> expected_content;
    for(size_t i = 0; i < expected_content.size(); ++i)
        expected_content[i] = i + DCP_ESCAPE_CHARACTER + 1;

    spi_rw_data->set(expected_content);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_equal(SPI_SEND_RESULT_OK,
                        spi_send_buffer(expected_spi_fd,
                                        expected_content.data(),
                                        expected_content.size(), 500,
                                        nullptr));
}

/*!\test
 * Write some data to SPI slave with to-be-escaped characters inside.
 *
 * Note that the expected behavior is that all data are sent as-is! Escaping of
 * raw data must be done by calling #spi_fill_buffer_from_raw_data().
 */
void test_write_escaped_data_to_spi()
{
    expect_spi_slave_gets_ready();

    static const std::array<uint8_t, 6> raw_data =
    {
        0x00, 0x01, 0x02, DCP_ESCAPE_CHARACTER, UINT8_MAX - 1U, UINT8_MAX,
    };

    spi_rw_data->set(raw_data);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_equal(SPI_SEND_RESULT_OK,
                        spi_send_buffer(expected_spi_fd, raw_data.data(),
                                        raw_data.size(), 500, nullptr));
}

/*!\test
 * Escape data for DCP-over-SPI.
 */
void test_escape_data_for_dcp_over_spi()
{
    static const std::array<uint8_t, 6> raw_data =
    {
        0x00, 0x01, 0x02, DCP_ESCAPE_CHARACTER, UINT8_MAX - 1U, UINT8_MAX,
    };

    static const std::array<uint8_t, 8> expected_result =
    {
        0x00,                                           /* regular byte */
        0x01,                                           /* regular byte */
        0x02,                                           /* regular byte */
        DCP_ESCAPE_CHARACTER, DCP_ESCAPE_CHARACTER,     /* escaped escape */
        UINT8_MAX - 1U,                                 /* regular byte */
        DCP_ESCAPE_CHARACTER, 0x01,                     /* escaped NOP */
    };

    uint8_t buffer[64];

    cppcut_assert_equal(expected_result.size(),
                        spi_fill_buffer_from_raw_data(buffer, sizeof(buffer),
                                                      raw_data.data(),
                                                      raw_data.size()));
    cut_assert_equal_memory(expected_result.data(), expected_result.size(),
                            buffer, expected_result.size());
}

/*!\test
 * Destination buffer size is respected while escaping data.
 */
void test_too_small_buffer_for_escaped_data()
{
    static const std::array<uint8_t, 10> src_data =
    {
        0x05, 0x23, 0x42, 0x7f, 0x81, 0x00, 0xe5, 0x2b, 0xa9, 0x61
    };

    std::array<uint8_t, 12> dest_buffer;
    dest_buffer.fill(0x55);

    cppcut_assert_equal(size_t(8),
                        spi_fill_buffer_from_raw_data(dest_buffer.data() + 2, 8,
                                                      src_data.data(),
                                                      src_data.size()));

    cut_assert_equal_memory(dest_buffer.data() + 2, 8, src_data.data(), 8);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[0]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[1]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[10]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[11]);
}

/*!\test
 * Destination buffer size is respected while escaping last character.
 */
void test_too_small_buffer_for_escaped_data_with_last_char_is_escape()
{
    static const std::array<uint8_t, 5> src_data =
    {
        0x05, 0x23, 0x42, DCP_ESCAPE_CHARACTER, UINT8_MAX,
    };

    std::array<uint8_t, 8> dest_buffer;
    dest_buffer.fill(0x55);

    cppcut_assert_equal(size_t(4),
                        spi_fill_buffer_from_raw_data(dest_buffer.data() + 2, 4,
                                                      src_data.data(),
                                                      src_data.size()));

    cut_assert_equal_memory(dest_buffer.data() + 2, 4, src_data.data(), 4);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[0]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[1]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[6]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[7]);
}

/*!\test
 * Destination buffer size is respected while escaping last character.
 */
void test_too_small_buffer_for_escaped_data_with_last_char_is_uint8_max()
{
    static const std::array<uint8_t, 5> src_data =
    {
        0x05, 0x23, 0x42, UINT8_MAX, DCP_ESCAPE_CHARACTER,
    };

    std::array<uint8_t, 8> dest_buffer;
    dest_buffer.fill(0x55);

    cppcut_assert_equal(size_t(4),
                        spi_fill_buffer_from_raw_data(dest_buffer.data() + 2, 4,
                                                      src_data.data(),
                                                      src_data.size()));

    cut_assert_equal_memory(dest_buffer.data() + 2, 3, src_data.data(), 3);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[0]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[1]);
    cppcut_assert_equal(uint8_t(DCP_ESCAPE_CHARACTER), dest_buffer[5]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[6]);
    cppcut_assert_equal(uint8_t(0x55), dest_buffer[7]);
}

template <size_t N>
static void expect_buffer_content(std::array<uint8_t, N> &buffer,
                                  uint8_t expected_value)
{
    for(auto b : buffer)
        cppcut_assert_equal(expected_value, b);
}

/*!\test
 * Timeout during read due to extreme latency (context switch) between time
 * measurements.
 */
void test_timeout_without_any_read_is_not_possible()
{
    static const struct timespec t1 =
    {
        .tv_sec = 1,
        .tv_nsec = 0,
    };

    static const struct timespec t2 =
    {
        .tv_sec = 3,
        .tv_nsec = 0,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t2);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 1000,
                                        nullptr));
    expect_buffer_content(buffer, 0x55);
}

/*!\test
 * Timeout is measured in nanoseconds and therefore only limited by system
 * timer resolution.
 */
void test_read_timeout_is_precisely_measured()
{
    static const struct timespec t1 =
    {
        .tv_sec = 0,
        .tv_nsec = 0,
    };

    static const struct timespec t2 =
    {
        .tv_sec = 0,
        .tv_nsec = 800UL * 1000UL * 1000UL - 1UL,
    };

    static const struct timespec t3 =
    {
        .tv_sec = 1,
        .tv_nsec = 800UL * 1000UL * 1000UL,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t3);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 800,
                                        nullptr));
    expect_buffer_content(buffer, 0x55);
}

/*!\test
 * Timeout expiration is also computed correctly when overflowing the
 * nanoseconds in the \c timespec structure.
 */
void test_timeout_overflow_in_struct_timespec()
{
    static const struct timespec t1 =
    {
        .tv_sec = 0,
        .tv_nsec = 1000UL * 1000UL * 1000UL - 700UL * 1000UL,
    };

    /* 0.7 milliseconds later */
    static const struct timespec t2 =
    {
        .tv_sec = 1,
        .tv_nsec = 0,
    };

    /* 0.999 milliseconds later than t1 */
    static const struct timespec t3 =
    {
        .tv_sec = 1,
        .tv_nsec = 300UL * 1000UL - 1UL,
    };

    /* 1 millisecond later than t1 */
    static const struct timespec t4 =
    {
        .tv_sec = 1,
        .tv_nsec = 300UL * 1000UL,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t3);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t4);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 1,
                                        nullptr));
}

/*!\test
 * Timeout handling also works on a seconds scale.
 */
void test_long_timeout()
{
    static const struct timespec t1 =
    {
        .tv_sec = 0,
        .tv_nsec = 300UL * 1000UL * 1000UL,
    };

    /* 1.7 seconds later than t1 */
    static const struct timespec t2 =
    {
        .tv_sec = 2,
        .tv_nsec = 0,
    };

    /* 2.999999999 seconds later than t1 */
    static const struct timespec t3 =
    {
        .tv_sec = 3,
        .tv_nsec = 300UL * 1000UL * 1000UL - 1UL,
    };

    /* 3 seconds later than t1 */
    static const struct timespec t4 =
    {
        .tv_sec = 3,
        .tv_nsec = 300UL * 1000UL * 1000UL,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t3);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t4);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 3000,
                                        nullptr));
}

/*!\test
 * Two transactions initiated by the slave device.
 *
 * This test mimicks two SPI slave transfers by calling the involved low level
 * functions directly in the expected order. The slave incorrectly sends zeros
 * instead of NOPs after sending the actual command bytes. Our internal read
 * buffer is filled with these zeros, so #spi_new_transaction() must be called
 * upon begin of any slave request.
 *
 * \note
 *     Some refactoring of the production code should be done to make the real
 *     transaction code testable.
 */
void test_new_spi_slave_transaction_clears_internal_receive_buffer()
{
    static const std::array<uint8_t, read_from_slave_spi_transfer_size> slave_request_data[2] =
    {
        {
            0x02, 0x48, 0x01, 0x00, 0x28, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        },
        {
            0x02, 0x48, 0x01, 0x00, 0x26, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        }
    };

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    for(size_t i = 0;
        i < sizeof(slave_request_data) / sizeof(slave_request_data[0]);
        ++i)
    {
        if(i > 0)
            mock_messages->expect_msg_info_formatted("Discarding 27 bytes from SPI receive buffer");

        spi_new_transaction();

        auto &req(slave_request_data[i]);

        spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, req);

        /* read DCP header */
        mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
        mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

        std::array<uint8_t, 4> command_header_buffer;

        cppcut_assert_equal(ssize_t(command_header_buffer.size()),
                            spi_read_buffer(expected_spi_fd,
                                            command_header_buffer.data(),
                                            command_header_buffer.size(),
                                            10, nullptr));
        cut_assert_equal_memory(req.data(),
                                command_header_buffer.size(),
                                command_header_buffer.data(),
                                command_header_buffer.size());

        /* read DRCP command code (comes from internal buffer, no SPI transfer is
         * done here) */
        mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

        std::array<uint8_t, 1> drcp_command_buffer;

        cppcut_assert_equal(ssize_t(drcp_command_buffer.size()),
                            spi_read_buffer(expected_spi_fd,
                                            drcp_command_buffer.data(),
                                            drcp_command_buffer.size(),
                                            10, nullptr));
        cut_assert_equal_memory(req.data() + command_header_buffer.size(),
                                drcp_command_buffer.size(),
                                drcp_command_buffer.data(),
                                drcp_command_buffer.size());
    }

    mock_messages->expect_msg_info_formatted("Discarding 27 bytes from SPI receive buffer");
    spi_new_transaction();

    ensure_empty_read_buffer();
}

/*!\test
 * Timeout during write due to extreme latency (context switch) between time
 * measurements.
 */
void test_send_timeout_without_any_write_is_not_possible()
{
    static const struct timespec t1 =
    {
        .tv_sec = 1,
        .tv_nsec = 0,
    };

    static const struct timespec t2 =
    {
        .tv_sec = 3,
        .tv_nsec = 0,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(return_nops);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t2);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI write timeout, slave didn't get ready within 1000 ms");

    std::array<uint8_t, 10> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_TIMEOUT,
                        spi_send_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 1000,
                                        nullptr));
}

/*!\test
 * Before sending data, wait for slave until it gets ready, timeout does not
 * expire.
 */
void test_send_to_slave_waits_for_zero_byte_answer_before_sending()
{
    struct timespec t =
    {
        .tv_sec = 10,
        .tv_nsec = 0,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    /* some NOP transfers while waiting for slave, one short transfer per 2 ms
     * (that is, we do NOT take the real amount of delay into account here,
     * which would be #delay_between_slave_probes_ms!) */
    for(int i = 0; i < 10; ++i)
    {
        mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
        mock_os->expect_os_nanosleep(0, delay_between_slave_probes_ms);
        t.tv_nsec += 2UL * 1000UL * 1000UL;

        spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                           spi_rw_data_t::EXPECT_READ_NOPS,
                                                           true);
        mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    }

    /* slave signals it's ready */
    expect_spi_slave_gets_ready();

    /* send data bytes */
    static const std::array<uint8_t, 7> buffer =
    {
        0x90, 0x5a, 0xb7, 0xdb, 0xeb, 0x00, 0x4d,
    };

    spi_rw_data->set(buffer);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cppcut_assert_equal(SPI_SEND_RESULT_OK,
                        spi_send_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 100,
                                        nullptr));
}

/*!\test
 * Before sending data, wait for slave until timeout expires.
 */
void test_send_to_slave_may_fail_due_to_timeout()
{
    struct timespec t =
    {
        .tv_sec = 10,
        .tv_nsec = 0,
    };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    /* some NOP transfers while waiting for slave, one short transfer per 5 ms
     * (that is, we do NOT take the real amount of delay into account here,
     * which would be #delay_between_slave_probes_ms!) */
    for(int i = 0; i < 21; ++i)
    {
        mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

        /* the 21st sleep does not happen because of timeout */
        if(i < 20)
            mock_os->expect_os_nanosleep(0, delay_between_slave_probes_ms);

        t.tv_nsec += 5UL * 1000UL * 1000UL;

        spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                           spi_rw_data_t::EXPECT_READ_NOPS,
                                                           true);
        mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    }

    /* send data bytes */
    static const std::array<uint8_t, 7> not_sent_data =
    {
        0x90, 0x5a, 0xb7, 0xdb, 0xeb, 0x00, 0x4d,
    };

    /* no further SPI transfer takes place for the data, instead a log message
     * is emitted and an error code is returned */
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI write timeout, slave didn't get ready within 100 ms");

    cppcut_assert_equal(SPI_SEND_RESULT_TIMEOUT,
                        spi_send_buffer(expected_spi_fd,
                                        not_sent_data.data(),
                                        not_sent_data.size(), 100, nullptr));
}

/*!\test
 * Collisions are detected when the slave device sends non-NOP, non-zero bytes
 * when being polled (see #161).
 *
 * The test only checks if detection is done the right way. Collision handling
 * is done in the layer on top and should be tested somewhere else.
 */
void test_collision_detection_by_inspecting_poll_bytes()
{
    /* polling takes place because the request line is not asserted (not yet or
     * not anymore---we will never know) */
    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       spi_rw_data_t::EXPECT_READ_NON_ZERO,
                                                       true);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* the slave's data sent while polling can be received */
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 2> receive_buffer;

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cppcut_assert_equal(0xa5, static_cast<int>(receive_buffer[0]));
    cppcut_assert_equal(0xa5, static_cast<int>(receive_buffer[1]));

    ensure_empty_read_buffer();
}

/*!\test
 * Bytes read from slave while polling are not discarded after detecting a
 * collision in poll bytes (see #161).
 */
void test_collision_in_poll_bytes_does_not_discard_bytes_sent_by_slave()
{
    /* slave sends these bytes when being polled */
    static const std::array<uint8_t, 2> first_fragment  = { 0x01, 0x00, };

    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       first_fragment,
                                                       true);
    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* slave sends more data to complete its command */
    static const std::array<uint8_t, 5> second_fragment = { 0x66, 0x77, 0x88, 0xaa, 0xfe, };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, second_fragment);

    std::array<uint8_t, first_fragment.size() + second_fragment.size()> expected_data;

    std::copy(first_fragment.begin(), first_fragment.end(),
              expected_data.begin() + 0);
    std::copy(second_fragment.begin(), second_fragment.end(),
              expected_data.begin() + first_fragment.size());

    std::array<uint8_t, expected_data.size()> receive_buffer;

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cut_assert_equal_memory(expected_data.data(), expected_data.size(),
                            receive_buffer.data(), receive_buffer.size());

    ensure_empty_read_buffer();
}

/*!\test
 * Data read from slave while polling may contain escape sequence.
 */
void test_collision_in_poll_bytes_does_not_confuse_escape_sequences()
{
    /* slave sends these bytes when being polled: an escaped 0xff */
    static const std::array<uint8_t, 2> first_fragment  = { DCP_ESCAPE_CHARACTER, 0x01, };

    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       first_fragment,
                                                       true);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* slave sends more data to complete its command */
    static const std::array<uint8_t, 3> second_fragment = { 0x33, 0x09, 0x1f, };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, second_fragment);

    std::array<uint8_t, 1 + second_fragment.size()> expected_data;

    /* espace sequence was resolved */
    expected_data[0] = UINT8_MAX;
    std::copy(second_fragment.begin(), second_fragment.end(),
              expected_data.begin() + 1);

    std::array<uint8_t, expected_data.size()> receive_buffer;

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cut_assert_equal_memory(expected_data.data(), expected_data.size(),
                            receive_buffer.data(), receive_buffer.size());

    ensure_empty_read_buffer();
}

/*!\test
 * Data read from slave while polling may end with a escape character.
 */
void test_collision_in_poll_bytes_may_end_with_escape_character()
{
    /* slave sends these bytes when being polled */
    static const std::array<uint8_t, 2> first_fragment  = { 0x01, DCP_ESCAPE_CHARACTER, };

    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       first_fragment,
                                                       true);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* slave sends more data to complete its command: the 0x01 is escaped */
    static const std::array<uint8_t, 4> second_fragment = { 0x01, 0x80, 0x71, 0xba, };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, second_fragment);

    std::array<uint8_t, first_fragment.size() + second_fragment.size() - 1> expected_data;

    std::copy(first_fragment.begin(), first_fragment.end() - 1,
              expected_data.begin() + 0);
    expected_data[1] = 0xff;
    std::copy(second_fragment.begin() + 1, second_fragment.end(),
              expected_data.begin() + first_fragment.size());

    std::array<uint8_t, expected_data.size()> receive_buffer;

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cut_assert_equal_memory(expected_data.data(), expected_data.size(),
                            receive_buffer.data(), receive_buffer.size());

    ensure_empty_read_buffer();
}

/*!\test
 * Data read from slave while polling may be NOPs with a single escape
 * character as last byte.
 */
void test_collision_in_poll_bytes_may_end_with_escape_character_after_nops()
{
    /* slave sends these bytes when being polled */
    static const std::array<uint8_t, 2> first_fragment  = { UINT8_MAX, DCP_ESCAPE_CHARACTER, };

    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       first_fragment,
                                                       true);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* slave sends more data to complete its command */
    static const std::array<uint8_t, 3> second_fragment = { 0x01, 0x02, 0x03, };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, second_fragment);

    /* initial NOP should be removed, dangling espace sequence should be
     * resolved (0x01 turns into UINT8_MAX) */
    static const std::array<uint8_t, second_fragment.size()> expected_data =
    {
        UINT8_MAX, 0x02, 0x03,
    };

    std::array<uint8_t, expected_data.size()> receive_buffer;

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cut_assert_equal_memory(expected_data.data(), expected_data.size(),
                            receive_buffer.data(), receive_buffer.size());

    ensure_empty_read_buffer();
}

/*!\test
 * Data read from slave while polling may start with a NOP, which is filtered.
 */
void test_collision_in_poll_bytes_may_begin_with_nop()
{
    /* slave sends these bytes when being polled */
    static const std::array<uint8_t, 2> first_fragment  = { UINT8_MAX, 0x3a, };

    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       first_fragment,
                                                       true);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* the slave's data sent while polling can be received */
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 1> receive_buffer;

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cppcut_assert_equal(0x3a, static_cast<int>(receive_buffer[0]));

    ensure_empty_read_buffer();
}

/*!\test
 * Data read from slave while polling may end with a NOP, which is filtered.
 */
void test_collision_in_poll_bytes_may_end_with_nop()
{
    /* slave sends these bytes when being polled */
    static const std::array<uint8_t, 2> first_fragment  = { 0xc5, UINT8_MAX, };

    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       first_fragment,
                                                       true);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    std::array<uint8_t, 10> send_buffer;
    send_buffer.fill(0x55);

    cppcut_assert_equal(SPI_SEND_RESULT_COLLISION,
                        spi_send_buffer(expected_spi_fd,
                                        send_buffer.data(), send_buffer.size(),
                                        1000, nullptr));

    /* the slave's data sent while polling can be received */
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 1> receive_buffer;

    cppcut_assert_equal(ssize_t(receive_buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        receive_buffer.data(),
                                        receive_buffer.size(), 500, nullptr));

    cppcut_assert_equal(0xc5, static_cast<int>(receive_buffer[0]));

    ensure_empty_read_buffer();
}

};

/*!@}*/
