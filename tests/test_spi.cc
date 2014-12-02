#include <cppcutter.h>

#include "spi.h"

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

    uint8_t buffer[10] = {0};

    cppcut_assert_equal(ssize_t(0), spi_read_buffer(expected_spi_fd, buffer, sizeof(buffer), 1000));
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
        .tv_nsec = 1000L * 1000UL * 1000UL * 1000UL - 1UL,
    };

    static const struct timespec t3 =
    {
        .tv_sec = 1,
        .tv_nsec = 0,
    };

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t1);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t2);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(read_nops);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t3);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "SPI read timeout, returning 0 of 10 bytes");

    uint8_t buffer[10] = {0};

    cppcut_assert_equal(ssize_t(0), spi_read_buffer(expected_spi_fd, buffer, sizeof(buffer), 1000));
}

};

/*!@}*/
