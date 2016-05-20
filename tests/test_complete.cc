/*
 * Copyright (C) 2016  T+A elektroakustik GmbH & Co. KG
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

#include <cppcutter.h>
#include <array>

#include "dcpspi_process.h"
#include "spi.h"

#include "mock_messages.hh"
#include "mock_spi_hw.hh"
#include "mock_gpio.hh"
#include "mock_os.hh"
#include "spi_hw_data.hh"

ssize_t (*os_read)(int fd, void *dest, size_t count);
ssize_t (*os_write)(int fd, const void *buf, size_t count);
int (*os_poll)(struct pollfd *fds, nfds_t nfds, int timeout);

namespace complete_tests
{

class PollResult
{
  private:
    int retval_;
    int errno_;
    bool pending_;
    std::array<short, 2> revents;

    static constexpr const size_t GPIO_INDEX = 0;
    static constexpr const size_t DCPD_INDEX = 1;

  public:
    PollResult(const PollResult &) = delete;
    PollResult &operator=(const PollResult &) = delete;

    explicit PollResult() { reset(); }

    void reset()
    {
        retval_ = -1;
        errno_ = EFAULT;
        pending_ = false;
        revents.fill(0);
    }

    void check()
    {
        cut_assert_false(pending_);
    }

    PollResult &set_return_value(int retval)
    {
        retval_ = retval;
        pending_ = true;
        return *this;
    }

    PollResult &set_dcpd_events(short events)
    {
        revents[DCPD_INDEX] = events;
        pending_ = true;
        return *this;
    }

    PollResult &set_gpio_events(short events)
    {
        revents[GPIO_INDEX] = events;
        pending_ = true;
        return *this;
    }

    int finish(struct pollfd *fds, nfds_t nfds,
               int expected_gpio_fd, int expected_dcpd_fd)
    {
        cppcut_assert_not_null(fds);
        cppcut_assert_equal(nfds_t(revents.size()), nfds);
        cppcut_assert_equal(expected_gpio_fd,         fds[GPIO_INDEX].fd);
        cppcut_assert_equal(short(POLLPRI | POLLERR), fds[GPIO_INDEX].events);
        cppcut_assert_equal(expected_dcpd_fd,         fds[DCPD_INDEX].fd);
        cppcut_assert_equal(short(POLLIN),            fds[DCPD_INDEX].events);

        fds[GPIO_INDEX].revents = revents[GPIO_INDEX];
        fds[DCPD_INDEX].revents = revents[DCPD_INDEX];

        int ret = retval_;
        int err = errno_;

        reset();

        errno = err;

        return ret;
    }
};

class ProcessData
{
  public:
    struct gpio_handle *gpio;
    struct dcp_transaction transaction;
    struct buffer deferred_transaction_data;
    struct collision_check_data ccdata;
    bool prev_gpio_state;

    ProcessData(const ProcessData &) = delete;
    ProcessData &operator=(const ProcessData &) = delete;

    explicit ProcessData(int gpio_fd,
                         uint8_t (&dcp_buffer)[260],
                         uint8_t (&deferred_dcp_buffer)[260],
                         uint8_t (&spi_buffer)[260 * 2]):
        gpio(nullptr),
        prev_gpio_state(false)
    {
        memset(&transaction, 0, sizeof(transaction));
        memset(&deferred_transaction_data, 0, sizeof(deferred_transaction_data));
        memset(&ccdata, 0, sizeof(ccdata));

        memset(dcp_buffer, 0, sizeof(dcp_buffer));
        memset(deferred_dcp_buffer, 0, sizeof(deferred_dcp_buffer));
        memset(spi_buffer, 0, sizeof(spi_buffer));

        gpio = MockGPIO::get_handle(gpio_fd);

        transaction.dcp_buffer.buffer = dcp_buffer;
        transaction.dcp_buffer.size = sizeof(dcp_buffer);
        transaction.spi_buffer.buffer = spi_buffer;
        transaction.spi_buffer.size = sizeof(spi_buffer);
        reset_transaction_struct(&transaction);

        deferred_transaction_data.buffer = deferred_dcp_buffer;
        deferred_transaction_data.size = sizeof(deferred_dcp_buffer);

        ccdata.gpio = gpio;
    }

    ~ProcessData()
    {
        MockGPIO::close_handle(gpio);
    }
};

static const int expected_fifo_in_fd = 40;
static const int expected_fifo_out_fd = 50;
static const int expected_gpio_fd = 60;
static const int expected_spi_fd = 70;

static MockMessages *mock_messages;
static MockOs *mock_os;
static MockSPIHW *mock_spi_hw;
static MockGPIO *mock_gpio;

static PollResult poll_result;
static ProcessData *process_data;

static std::vector<char> os_write_buffer;

static ssize_t read_mock(int fd, void *dest, size_t count)
{
    cut_fail("read() called");
    return -1;
}

static ssize_t write_mock(int fd, const void *buf, size_t count)
{
    cppcut_assert_equal(expected_fifo_out_fd, fd);
    cppcut_assert_not_null(buf);
    cppcut_assert_operator(size_t(0), <, count);

    std::copy_n(static_cast<const char *>(buf), count,
                std::back_inserter<std::vector<char>>(os_write_buffer));

    return count;
}

static int poll_mock(struct pollfd *fds, nfds_t nfds, int timeout)
{
    cppcut_assert_equal(-1, timeout);

    return poll_result.finish(fds, nfds, expected_gpio_fd, expected_fifo_in_fd);
}

static const auto mock_spi_transfer = mock_spi_tx_template<expected_spi_fd>;

void cut_setup()
{
    os_read = read_mock;
    os_write = write_mock;
    os_poll = poll_mock;

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

    mock_gpio = new MockGPIO;
    cppcut_assert_not_null(mock_gpio);
    mock_gpio->init();
    mock_gpio_singleton = mock_gpio;

    os_write_buffer.clear();

    poll_result.reset();

    static uint8_t dcp_double_buffer[2][260];
    static uint8_t spi_backing_buffer[260 * 2];

    process_data = new ProcessData(expected_gpio_fd, dcp_double_buffer[0],
                                   dcp_double_buffer[1], spi_backing_buffer);
    cppcut_assert_not_null(process_data);

    spi_rw_data = new spi_rw_data_t;
    cppcut_assert_not_null(spi_rw_data);

    spi_reset();
}

void cut_teardown()
{
    poll_result.check();

    delete process_data;
    process_data = nullptr;

    os_write_buffer.clear();
    os_write_buffer.shrink_to_fit();

    mock_messages->check();
    mock_os->check();
    mock_spi_hw->check();
    mock_gpio->check();

    mock_messages_singleton = nullptr;
    mock_os_singleton = nullptr;
    mock_spi_hw_singleton = nullptr;
    mock_gpio_singleton = nullptr;

    delete mock_messages;
    delete mock_os;
    delete mock_spi_hw;
    delete mock_gpio;

    mock_messages = nullptr;
    mock_os = nullptr;
    mock_spi_hw = nullptr;
    mock_gpio = nullptr;

    delete spi_rw_data;
    spi_rw_data = nullptr;
}

static void expect_no_more_actions()
{
    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);

    for(int i = 0; i < 3; ++i)
    {
        mock_gpio->expect_gpio_is_active(false, process_data->gpio);
        poll_result.set_return_value(1);

        cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                       expected_spi_fd, expected_gpio_fd, true,
                                       &process_data->transaction,
                                       &process_data->deferred_transaction_data,
                                       &process_data->ccdata,
                                       &process_data->prev_gpio_state));

        cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    }

    cppcut_assert_equal(size_t(0), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(REQ_NOT_REQUESTED, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
}

/*!\test
 * Show how a write transaction from the SPI slave travels through the system.
 */
void test_single_slave_write_transaction()
{
    static const struct timespec dummy_time = { .tv_sec = 0, .tv_nsec = 0, };

    /* slave activates the request GPIO */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_gpio_events(POLLPRI).set_return_value(1);
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);

    /* slave sends write command for UPnP friendly name */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);
    std::array<uint8_t, 8> write_command { UINT8_MAX, 0x02, 0x58, 0x03, 0x00, 0x61, 0x62, 0x63 };
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, write_command);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_info_formatted(
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_SLAVE_WRITECMD_RECEIVING_DATA_FROM_SLAVE, process_data->transaction.state);

    /* process the rest of the SPI input buffer */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Slave write transaction: expecting 3 bytes from slave");
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_SLAVE_WRITECMD_FORWARDING_TO_DCPD, process_data->transaction.state);
    cut_assert_true(os_write_buffer.empty());

    /* send write command to DCPD */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Slave write transaction: send 7 bytes to DCPD");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cut_assert_equal_memory(&write_command.data()[1], write_command.size() - 1,
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();

    /* the poll(2) event for the deassearted request line is still pending, but
     * is handled only now (with no effect) */
    poll_result.check();
    poll_result.set_gpio_events(POLLPRI).set_return_value(1);
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    /* done */
    expect_no_more_actions();
}

}
