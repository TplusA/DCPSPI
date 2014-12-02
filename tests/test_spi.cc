#include <cppcutter.h>
#include <array>
#include <algorithm>

#include "spi.h"
#include "dcpdefs.h"

#include "mock_messages.hh"
#include "mock_spi_hw.hh"
#include "mock_os.hh"

/*!
 * \addtogroup spi_communication_tests Unit tests
 * \ingroup spi_communication
 *
 * SPI communication unit tests.
 */
/*!@{*/

namespace spi_communication_tests
{

static constexpr int expected_spi_fd = 42;
static constexpr size_t short_spi_transfer_size = 32;

class spi_rw_data_partial_t
{
  public:
    spi_rw_data_partial_t(const spi_rw_data_partial_t &) = delete;
    spi_rw_data_partial_t &operator=(const spi_rw_data_partial_t &) = delete;

    spi_rw_data_partial_t(spi_rw_data_partial_t &&) = default;

    const std::vector<uint8_t> expected_write_data_;
    const std::vector<uint8_t> read_data_;
    const size_t transfer_size_;
    const int errno_value_;
    const int return_value_;
    std::vector<uint8_t> written_data_;

    explicit spi_rw_data_partial_t(const std::vector<uint8_t> write_data,
                                   const std::vector<uint8_t> read_data,
                                   size_t transfer_size, int err, int ret):
        expected_write_data_(write_data),
        read_data_(read_data),
        transfer_size_(transfer_size),
        errno_value_(err),
        return_value_(ret)
    {}

    explicit spi_rw_data_partial_t(const std::vector<uint8_t> write_data,
                                   size_t transfer_size, int err, int ret):
        expected_write_data_(write_data),
        transfer_size_(transfer_size),
        errno_value_(err),
        return_value_(ret)
    {}
};

class spi_rw_data_t
{
  public:
    enum write_nops
    {
        EXPECT_WRITE_NOPS,
        EXPECT_WRITE_ZEROS,
    };

    enum read_nops
    {
        EXPECT_READ_NOPS,
        EXPECT_READ_ZEROS,
    };

    spi_rw_data_t(const spi_rw_data_t &) = delete;
    spi_rw_data_t &operator=(const spi_rw_data_t &) = delete;

    size_t fragment_;
    std::vector<spi_rw_data_partial_t> partial_;

    explicit spi_rw_data_t(): fragment_(0) {}

    /*!
     * One big write transfer, don't care about answer (rx buffer is NULL).
     */
    template <size_t N>
    void set(const std::array<uint8_t, N> &write_data)
    {
        set(write_data.data(), N, 0, 0);
    }

    /*!
     * Several small write transfers with constant answer from slave.
     */
    template <size_t N>
    void set(const std::array<uint8_t, N> &write_data, enum read_nops nops)
    {
        std::array<uint8_t, N> read_data;

        switch(nops)
        {
          case EXPECT_READ_NOPS:
            read_data.fill(UINT8_MAX);
            break;

          case EXPECT_READ_ZEROS:
            read_data.fill(0);
            break;
        }

        set_fragments(write_data.data(), read_data.data(), N);
    }

    /*!
     * Several small read transfers with constant value written to slave.
     */
    template <size_t N>
    void set(enum write_nops nops, const std::array<uint8_t, N> &read_data)
    {
        std::array<uint8_t, N> write_data;

        switch(nops)
        {
          case EXPECT_WRITE_NOPS:
            write_data.fill(UINT8_MAX);
            break;

          case EXPECT_WRITE_ZEROS:
            write_data.fill(0);
            break;
        }

        set_fragments(write_data.data(), read_data.data(), N);
    }

    /*!
     * Full expected transfer specification.
     */
    template <size_t N>
    void set(const std::array<uint8_t, N> &write_data,
             const std::array<uint8_t, N> &read_data)
    {
        set_fragments(write_data.data(), read_data.data(), N);
    }

  private:
    void set_fragments(const uint8_t *write_data, const uint8_t *read_data,
                       size_t transfer_size)
    {
        for(size_t offset = 0; offset < transfer_size; offset += short_spi_transfer_size)
        {
            const size_t count =
                std::min(short_spi_transfer_size, transfer_size - offset);

            if(count == short_spi_transfer_size)
                set(write_data + offset, read_data + offset,
                    short_spi_transfer_size, 0, 0);
            else
            {
                std::array<uint8_t, short_spi_transfer_size> last_write_data;
                std::array<uint8_t, short_spi_transfer_size> last_read_data;

                last_write_data.fill(UINT8_MAX);
                last_read_data.fill(UINT8_MAX);

                std::copy_n(write_data + offset, count, last_write_data.begin());
                std::copy_n(read_data + offset,  count, last_read_data.begin());

                set(last_write_data.data(), last_read_data.data(),
                    short_spi_transfer_size, 0, 0);
            }
        }
    }

    void set(const uint8_t *write_data,
             size_t transfer_size, int err, ssize_t ret)
    {
        partial_.push_back(spi_rw_data_partial_t(std::vector<uint8_t>(write_data,
                                                                      write_data + transfer_size),
                                                 transfer_size, err, ret));
    }

    void set(const uint8_t *write_data, const uint8_t *read_data,
             size_t transfer_size, int err, ssize_t ret)
    {
        partial_.push_back(spi_rw_data_partial_t(std::vector<uint8_t>(write_data,
                                                                      write_data + transfer_size),
                                                 std::vector<uint8_t>(read_data,
                                                                      read_data + transfer_size),
                                                 transfer_size, err, ret));
    }
};

static spi_rw_data_t *spi_rw_data;

static MockMessages *mock_messages;
static MockOs *mock_os;
static MockSPIHW *mock_spi_hw;

void cut_setup(void)
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

void cut_teardown(void)
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

static int mock_spi_transfer(int fd,
                             const struct spi_ioc_transfer spi_transfer[],
                             size_t number_of_fragments)
{
    cppcut_assert_equal(expected_spi_fd, fd);
    cppcut_assert_equal(size_t(1), number_of_fragments);
    cppcut_assert_operator(spi_rw_data->fragment_, <, spi_rw_data->partial_.size());

    const spi_rw_data_partial_t &partial(spi_rw_data->partial_[spi_rw_data->fragment_++]);

    if(partial.read_data_.size() > 0)
        cppcut_assert_not_equal(__u64(0), spi_transfer[0].rx_buf);
    else
        cppcut_assert_equal(__u64(0), spi_transfer[0].rx_buf);

    if(partial.expected_write_data_.size() > 0)
        cppcut_assert_not_equal(__u64(0), spi_transfer[0].tx_buf);
    else
        cppcut_assert_equal(__u64(0), spi_transfer[0].tx_buf);

    cppcut_assert_equal(partial.transfer_size_, size_t(spi_transfer[0].len));
    cppcut_assert_equal(partial.transfer_size_, partial.expected_write_data_.size());

    if(partial.read_data_.size() > 0)
    {
        cppcut_assert_equal(partial.transfer_size_, partial.read_data_.size());
        std::copy_n(partial.read_data_.begin(), partial.read_data_.size(),
                    reinterpret_cast<uint8_t *>(spi_transfer[0].rx_buf));
    }

    if(partial.expected_write_data_.size() > 0)
        cut_assert_equal_memory(partial.expected_write_data_.data(),
                                partial.expected_write_data_.size(),
                                reinterpret_cast<uint8_t *>(spi_transfer[0].tx_buf),
                                spi_transfer[0].len);

    errno = partial.errno_value_;

    return partial.return_value_;
}

static void expect_spi_transfers(size_t number_of_bytes)
{
    const size_t count =
        number_of_bytes / short_spi_transfer_size +
        ((number_of_bytes % short_spi_transfer_size) == 0 ? 0 : 1);

    for(size_t i = 0; i < count; ++i)
        mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
}

/*!\test
 * Regular happy case: just read some data from SPI slave, no escape character.
 */
void test_read_from_spi(void)
{
    std::array<uint8_t, 100> expected_content;
    for(size_t i = 0; i < expected_content.size(); ++i)
        expected_content[i] = i + DCP_ESCAPE_CHARACTER + 1;

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, expected_content);
    expect_spi_transfers(expected_content.size());

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 100> buffer;
    buffer.fill(0xab);

    cppcut_assert_equal(ssize_t(buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 500));

    cut_assert_equal_memory(expected_content.data(), expected_content.size(),
                            buffer.data(), buffer.size());
}

/*!\test
 * Read some data from SPI slave with escape characters inside.
 */
void test_read_escaped_data_from_spi(void)
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
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    std::array<uint8_t, 11> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(buffer.size()),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 500));

    static const std::array<uint8_t, 11> expected_content =
    {
        0x00, 0x01, 0x02, UINT8_MAX - 1U,
        0x00, UINT8_MAX, 0x02, DCP_ESCAPE_CHARACTER, UINT8_MAX - 1U, 0x03,
        DCP_ESCAPE_CHARACTER,
    };

    cut_assert_equal_memory(expected_content.data(), expected_content.size(),
                            buffer.data(), buffer.size());
}

static void expect_spi_slave_gets_ready(void)
{
    /* let's assume eight cycles */
    static const std::array<uint8_t, short_spi_transfer_size> slave_gets_ready =
    {
        UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX,
        UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX,
    };

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, slave_gets_ready);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
}

/*!\test
 * Regular happy case: just write some data to SPI slave, no escape character.
 */
void test_write_to_spi(void)
{
    expect_spi_slave_gets_ready();

    std::array<uint8_t, 100> expected_content;
    for(size_t i = 0; i < expected_content.size(); ++i)
        expected_content[i] = i + DCP_ESCAPE_CHARACTER + 1;

    spi_rw_data->set(expected_content);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_equal(0,
                        spi_send_buffer(expected_spi_fd,
                                        expected_content.data(),
                                        expected_content.size(), 500));
}

/*!\test
 * Write some data to SPI slave with to-be-escaped characters inside.
 *
 * Note that the expected behavior is that all data are sent as-is! Escaping of
 * raw data must be done by calling #spi_fill_buffer_from_raw_data().
 */
void test_write_escaped_data_to_spi(void)
{
    expect_spi_slave_gets_ready();

    static const std::array<uint8_t, 6> raw_data =
    {
        0x00, 0x01, 0x02, DCP_ESCAPE_CHARACTER, UINT8_MAX - 1U, UINT8_MAX,
    };

    spi_rw_data->set(raw_data);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    static const struct timespec t = { .tv_sec = 0, .tv_nsec = 0, };
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);

    cppcut_assert_equal(0,
                        spi_send_buffer(expected_spi_fd, raw_data.data(),
                                        raw_data.size(), 500));
}

/*!\test
 * Escape data for DCP-over-SPI.
 */
void test_escape_data_for_dcp_over_spi(void)
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
void test_too_small_buffer_for_escaped_data(void)
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
void test_too_small_buffer_for_escaped_data_with_last_char_is_escape(void)
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
void test_too_small_buffer_for_escaped_data_with_last_char_is_uint8_max(void)
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

static int read_nops(int fd, const struct spi_ioc_transfer spi_transfer[],
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
void test_timeout_without_any_read_is_not_possible(void)
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

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t2);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 1000));
    expect_buffer_content(buffer, 0x55);
}

/*!\test
 * Timeout is measured in nanoseconds and therefore only limited by system
 * timer resolution.
 */
void test_read_timeout_is_precisely_measured(void)
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

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t3);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 800));
    expect_buffer_content(buffer, 0x55);
}

/*!\test
 * Timeout expiration is also computed correctly when overflowing the
 * nanoseconds in the \c timespec structure.
 */
void test_timeout_overflow_in_struct_timespec(void)
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

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t3);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t4);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 1));
}

/*!\test
 * Timeout handling also works on a seconds scale.
 */
void test_long_timeout(void)
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

    /* 2.999999999 seconds seconds later than t1 */
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

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t3);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t4);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    std::array<uint8_t, 10> buffer;

    cppcut_assert_equal(ssize_t(0),
                        spi_read_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 3000));
}

/*!\test
 * Timeout during write due to extreme latency (context switch) between time
 * measurements.
 */
void test_send_timeout_without_any_write_is_not_possible(void)
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

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t2);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI write timeout, slave didn't get ready within 1000 ms");

    std::array<uint8_t, 10> buffer;
    buffer.fill(0x55);

    cppcut_assert_equal(-1,
                        spi_send_buffer(expected_spi_fd,
                                        buffer.data(), buffer.size(), 1000));
}

};

/*!@}*/
