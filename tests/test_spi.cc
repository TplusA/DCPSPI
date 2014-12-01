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
    delete mock_spi_hw_singleton;

    mock_messages = nullptr;
    mock_os = nullptr;
    mock_spi_hw = nullptr;
}

static int fail_transfer(int fd, const struct spi_ioc_transfer spi_transfer[],
                         size_t number_of_fragments)
{
    cppcut_assert_equal(expected_spi_fd, fd);
    cppcut_assert_operator(number_of_fragments, >, size_t(0));
    return -1;
}

/*!\test
 * Hard errors returned by ioctl() are detected in #spi_read_buffer().
 */
void test_hard_error_on_interface_is_detected(void)
{
    static struct timespec t =
    {
        .tv_sec = 1,
        .tv_nsec = 0,
    };

    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(fail_transfer);
    mock_messages->expect_msg_error_formatted(0, LOG_EMERG,
                                              "Failed reading 32 bytes from SPI device fd 42");

    uint8_t buffer[10] = {0};

    cppcut_assert_equal(ssize_t(0), spi_read_buffer(expected_spi_fd, buffer, sizeof(buffer), 1000));
}

};

/*!@}*/
