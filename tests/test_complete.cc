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
#include "dcpdefs.h"
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

static const struct timespec dummy_time = { .tv_sec = 0, .tv_nsec = 0, };

static MockMessages *mock_messages;
static MockOs *mock_os;
static MockSPIHW *mock_spi_hw;
static MockGPIO *mock_gpio;

static PollResult poll_result;
static ProcessData *process_data;

static std::vector<char> os_write_buffer;
static std::vector<char> os_read_buffer;

static ssize_t read_mock(int fd, void *dest, size_t count)
{
    cppcut_assert_equal(expected_fifo_in_fd, fd);
    cppcut_assert_not_null(dest);
    cppcut_assert_operator(size_t(0), <, count);

    if(count > os_read_buffer.size())
        count = os_read_buffer.size();

    std::copy_n(os_read_buffer.begin(), count, static_cast<char *>(dest));

    if(count < os_read_buffer.size())
        os_read_buffer.erase(os_read_buffer.begin(), os_read_buffer.begin() + count);
    else
        os_read_buffer.clear();

    return count;
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
    os_read_buffer.clear();

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

    cut_assert_true(os_write_buffer.empty());
    os_write_buffer.shrink_to_fit();

    cut_assert_true(os_read_buffer.empty());
    os_read_buffer.shrink_to_fit();

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

static void expect_wait_for_spi_slave(const struct timespec &t)
{
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, t);
    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       spi_rw_data_t::EXPECT_READ_ZEROS,
                                                       true);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
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
    static const std::array<uint8_t, 8> write_command
    {
        UINT8_MAX, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x58, 0x03, 0x00,
        0x61, 0x62, 0x63
    };
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

/*!\test
 * Show how a read transaction from the SPI slave travels through the system.
 */
void test_single_slave_read_transaction()
{
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
    mock_gpio->check();

    /* slave sends read command for device status register */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);
    static const std::array<uint8_t, 5> read_command
    {
        UINT8_MAX, DCP_COMMAND_READ_REGISTER, 0x11, 0x00, 0x00
    };
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, read_command);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_info_formatted(
        "Slave transaction: command header from SPI: 0x01 0x11 0x00 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_SLAVE_READCMD_FORWARDING_TO_DCPD, process_data->transaction.state);
    mock_messages->check();
    mock_gpio->check();

    /* send read command to DCPD */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Slave read transaction: send 4 bytes to DCPD");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_SLAVE_READCMD_RECEIVING_HEADER_FROM_DCPD, process_data->transaction.state);
    cut_assert_equal_memory(&read_command.data()[1], read_command.size() - 1,
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();

    /* DCPD sends the result back to us, header first */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_return_value(1);
    static const std::array<uint8_t, 6> dcpd_answer
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x11, 0x02, 0x00,
        0x12, 0x34
    };
    std::copy_n(dcpd_answer.begin(), dcpd_answer.size(), std::back_inserter(os_read_buffer));
    mock_messages->expect_msg_info_formatted(
        "Slave read transaction: command header from DCPD: 0x03 0x11 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(TR_SLAVE_READCMD_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2), os_read_buffer.size());
    mock_messages->check();
    mock_gpio->check();

    /* process data from DCPD remaining in pipe buffer */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_return_value(1);
    mock_messages->expect_msg_info_formatted(
        "Slave read transaction: expecting 2 bytes from DCPD");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(TR_SLAVE_READCMD_FORWARDING_TO_SLAVE, process_data->transaction.state);
    mock_messages->check();
    mock_gpio->check();

    /* send answer to SPI slave */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Slave read transaction: send 6 bytes over SPI");
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(dcpd_answer);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

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

/*!\test
 * Show how a write transaction from the master travels through the system.
 */
void test_single_master_write_transaction()
{
    /* DCPD sends something through its pipe */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_return_value(1);
    static const std::array<uint8_t, 6> network_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x32, 0x02, 0x00,
        0x02, 0x01
    };
    std::copy_n(network_status.begin(), network_status.size(), std::back_inserter(os_read_buffer));
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: command header from DCPD: 0x03 0x32 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2), os_read_buffer.size());
    mock_messages->check();
    mock_gpio->check();

    /* process data from DCPD remaining in pipe buffer */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_return_value(1);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: expecting 2 bytes from DCPD");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE, process_data->transaction.state);
    mock_messages->check();
    mock_gpio->check();

    /* send command to SPI slave */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: send 6 bytes over SPI");
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(network_status);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    /* done */
    expect_no_more_actions();
}

/*!\test
 * Collision: We tried to send something, but the slave tried as well.
 *
 * In this case, the slave transaction shall take priority over the master
 * transaction. The master transaction shall be suspended so that the slave
 * transaction can be processed. After the slave transaction has finished, the
 * master transaction shall be restarted.
 */
void test_collision_when_starting_to_send_to_slave()
{
    /* DCPD sends something through its pipe */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_return_value(1);
    static const std::array<uint8_t, 6> network_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x32, 0x02, 0x00,
        0x02, 0x01
    };
    std::copy_n(network_status.begin(), network_status.size(), std::back_inserter(os_read_buffer));
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: command header from DCPD: 0x03 0x32 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2), os_read_buffer.size());
    cppcut_assert_equal(size_t(4), process_data->transaction.dcp_buffer.pos);
    mock_messages->check();
    mock_gpio->check();

    /* SPI slave interrupts with a write command, DCPD's data still resides in
     * the pipe buffer and is processed before the collision is detected so
     * that the full transaction is there before going any further */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_gpio_events(POLLPRI).set_return_value(2);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: expecting 2 bytes from DCPD");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(size_t(0), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQ_NOT_REQUESTED, process_data->transaction.request_state);
    cppcut_assert_equal(network_status.size(), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    mock_messages->check();
    mock_gpio->check();

    /* try sending command to SPI slave, but collision is detected and the
     * current master transaction get temporarily replace by a slave
     * transaction */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: send 6 bytes over SPI");
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (interrupted by slave)");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(network_status.size(), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQ_ASSERTED, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    mock_messages->check();
    mock_gpio->check();

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);
    static const std::array<uint8_t, 8> write_command
    {
        UINT8_MAX, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x58, 0x03, 0x00,
        0x61, 0x62, 0x63
    };
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
    mock_messages->check();
    mock_gpio->check();

    /* slave transaction: process the rest of the SPI input buffer */
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
    mock_messages->check();
    mock_gpio->check();

    /* slave transaction: send write command to DCPD */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Slave write transaction: send 7 bytes to DCPD");
    mock_messages->expect_msg_info(
        "Continue processing deferred master transaction");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(size_t(0), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQ_NOT_REQUESTED, process_data->transaction.request_state);
    cppcut_assert_equal(network_status.size(), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(&write_command.data()[1], write_command.size() - 1,
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();

    /* nice, now continue with the master transaction: send command to SPI
     * slave */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: send 6 bytes over SPI");
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(network_status);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

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

/*!\test
 * Collision: We tried to send something, slave tries as well and asserts the
 * request line for a long time.
 */
void test_collision_slave_holds_request_line_until_after_end_of_transaction()
{
    /* DCPD sends something through its pipe */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_return_value(1);
    static const std::array<uint8_t, 6> network_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x32, 0x02, 0x00,
        0x02, 0x01
    };
    std::copy_n(network_status.begin(), network_status.size(), std::back_inserter(os_read_buffer));
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: command header from DCPD: 0x03 0x32 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(TR_MASTER_WRITECMD_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2), os_read_buffer.size());
    cppcut_assert_equal(size_t(4), process_data->transaction.dcp_buffer.pos);
    mock_messages->check();
    mock_gpio->check();

    /* SPI slave interrupts with a write command, DCPD's data still resides in
     * the pipe buffer and is processed before the collision is detected so
     * that the full transaction is there before going any further */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    poll_result.set_dcpd_events(POLLIN).set_gpio_events(POLLPRI).set_return_value(2);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: expecting 2 bytes from DCPD");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(size_t(0), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQ_NOT_REQUESTED, process_data->transaction.request_state);
    cppcut_assert_equal(network_status.size(), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    mock_messages->check();
    mock_gpio->check();

    /* try sending command to SPI slave, but collision is detected and the
     * current master transaction get temporarily replace by a slave
     * transaction */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: send 6 bytes over SPI");
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (interrupted by slave)");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));
    cppcut_assert_equal(network_status.size(), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_SLAVE_CMD_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQ_ASSERTED, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    mock_messages->check();
    mock_gpio->check();

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, CLOCK_MONOTONIC_RAW, dummy_time);
    static const std::array<uint8_t, 8> write_command
    {
        UINT8_MAX, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x58, 0x03, 0x00,
        0x61, 0x62, 0x63
    };
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
    mock_messages->check();
    mock_gpio->check();

    /* slave transaction: process the rest of the SPI input buffer */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
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
    mock_messages->check();
    mock_gpio->check();

    /* slave transaction: send write command to DCPD */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Slave write transaction: send 7 bytes to DCPD");
    mock_messages->expect_msg_info_formatted(
        "End of transaction, waiting for slave to release request line");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(network_status.size(), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_SLAVE_WAIT_FOR_REQUEST_DEASSERT, process_data->transaction.state);
    cppcut_assert_equal(REQ_ASSERTED, process_data->transaction.request_state);
    cppcut_assert_equal(write_command.size() - 1U, process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(write_command.size() - 1U, process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(&write_command.data()[1], write_command.size() - 1,
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();

    /* waiting for slave to release the GPIO */
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    poll_result.set_gpio_events(POLLPRI).set_return_value(1);
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info(
        "Continue processing deferred master transaction");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, expected_gpio_fd, true,
                                   &process_data->transaction,
                                   &process_data->deferred_transaction_data,
                                   &process_data->ccdata,
                                   &process_data->prev_gpio_state));

    cppcut_assert_equal(size_t(0), process_data->deferred_transaction_data.pos);
    cppcut_assert_equal(TR_MASTER_WRITECMD_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQ_NOT_REQUESTED, process_data->transaction.request_state);
    cppcut_assert_equal(network_status.size(), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();

    /* nice, now continue with the master transaction: send command to SPI
     * slave */
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_info_formatted(
        "Master write transaction: send 6 bytes over SPI");
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(network_status);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

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
