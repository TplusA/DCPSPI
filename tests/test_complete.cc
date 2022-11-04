/*
 * Copyright (C) 2016, 2018, 2019, 2022  T+A elektroakustik GmbH & Co. KG
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

#include "dcpspi_process.h"
#include "dcpdefs.h"
#include "spi.h"
#include "hexdump.h"

#include "mock_messages.hh"
#include "mock_spi_hw.hh"
#include "mock_gpio.hh"
#include "mock_os.hh"
#include "spi_hw_data.hh"

ssize_t (*os_read)(int fd, void *dest, size_t count);
ssize_t (*os_write)(int fd, const void *buf, size_t count);
int (*os_poll)(struct pollfd *fds, nfds_t nfds, int timeout);

/* Dummy implementation */
void hexdump_to_log(enum MessageVerboseLevel level,
                    const uint8_t *const buffer, size_t buffer_length,
                    const char *what)
{
    cppcut_assert_not_null(buffer);
    cppcut_assert_not_null(what);
}

namespace complete_tests
{

static constexpr unsigned long delay_between_slave_probes_ms = 5;

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
    PollResult(PollResult &&) = default;

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

    PollResult &set_errno(int error_number)
    {
        errno_ = error_number;
        return *this;
    }

    int finish(struct pollfd *fds, nfds_t nfds, int timeout,
               int expected_gpio_fd, int expected_dcpd_fd)
    {
        cppcut_assert_not_null(fds);
        cppcut_assert_equal(nfds_t(revents.size()), nfds);
        cppcut_assert_equal(expected_gpio_fd,         fds[GPIO_INDEX].fd);
        cppcut_assert_equal(short(POLLPRI | POLLERR), fds[GPIO_INDEX].events);
        cppcut_assert_equal(short(POLLIN),            fds[DCPD_INDEX].events);

        if(fds[DCPD_INDEX].fd != -1)
        {
            cppcut_assert_equal(expected_dcpd_fd, fds[DCPD_INDEX].fd);
            cppcut_assert_equal(-1, timeout);
        }
        else
        {
            cppcut_assert_equal(0, timeout);
            cppcut_assert_equal(short(0), revents[DCPD_INDEX]);
        }

        if(retval_ == 0)
            cppcut_assert_operator(0, <=, timeout);

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
    static constexpr const size_t expected_spi_buffer_size =
        (DCP_HEADER_SIZE + DCP_PAYLOAD_MAXSIZE) * 2;

    static constexpr const size_t expected_dcp_buffer_size =
        DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE + DCP_PAYLOAD_MAXSIZE;

    struct gpio_handle *gpio;
    struct dcp_transaction transaction;
    struct slave_request_and_lock_data rldata;

    ProcessData(const ProcessData &) = delete;
    ProcessData &operator=(const ProcessData &) = delete;

    explicit ProcessData(int gpio_fd,
                         uint8_t (&dcp_buffer)[expected_dcp_buffer_size],
                         uint8_t (&spi_buffer)[expected_spi_buffer_size]):
        gpio(MockGPIO::get_handle(gpio_fd)),
        rldata{true, gpio, gpio_fd, false}
    {
        memset(&transaction, 0, sizeof(transaction));

        memset(dcp_buffer, 0, sizeof(dcp_buffer));
        memset(spi_buffer, 0, sizeof(spi_buffer));

        transaction.dcp_buffer.buffer = dcp_buffer;
        transaction.dcp_buffer.size = sizeof(dcp_buffer);
        transaction.spi_buffer.buffer = spi_buffer;
        transaction.spi_buffer.size = sizeof(spi_buffer);
        reset_transaction_struct(&transaction, true);
    }

    ~ProcessData()
    {
        MockGPIO::close_handle(gpio);
    }
};

class PollResults
{
  private:
    std::vector<PollResult> expected_results_;
    size_t next_checked_expectation_;

  public:
    PollResults(const PollResults &) = delete;
    PollResults &operator=(const PollResults &) = delete;

    explicit PollResults():
        next_checked_expectation_(0)
    {}

    void expect(PollResult &&res)
    {
        expected_results_.emplace_back(std::move(res));
    }

    int finish_next(struct pollfd *fds, nfds_t nfds, int timeout,
                    int expected_gpio_fd, int expected_dcpd_fd)
    {
        cppcut_assert_operator(expected_results_.size(), >, next_checked_expectation_);

        return
            expected_results_[next_checked_expectation_++].finish(fds, nfds, timeout,
                                                                  expected_gpio_fd,
                                                                  expected_dcpd_fd);
    }

    void check()
    {
        cppcut_assert_equal(next_checked_expectation_, expected_results_.size());

        for(auto &pr : expected_results_)
            pr.check();

        clear();
    }

    void clear()
    {
        expected_results_.clear();
        expected_results_.shrink_to_fit();
        next_checked_expectation_ = 0;
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

static PollResults poll_results;

static ProcessData *process_data;

static std::vector<uint8_t> os_write_buffer;
static std::vector<uint8_t> os_read_buffer;

static constexpr char process_transaction_message[] =
    "Process transaction state %d, serial 0x%04x, lock state %d, pending size %u, flush pos %zu";

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
                std::back_inserter<std::vector<uint8_t>>(os_write_buffer));

    return count;
}

static int poll_mock(struct pollfd *fds, nfds_t nfds, int timeout)
{
    return poll_results.finish_next(fds, nfds, timeout,
                                    expected_gpio_fd, expected_fifo_in_fd);
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

    poll_results.clear();

    mock_messages->ignore_messages_with_level_or_above(MESSAGE_LEVEL_TRACE);

    static uint8_t dcp_backing_buffer[ProcessData::expected_dcp_buffer_size];
    static uint8_t spi_backing_buffer[ProcessData::expected_spi_buffer_size];

    process_data = new ProcessData(expected_gpio_fd, dcp_backing_buffer,
                                   spi_backing_buffer);
    cppcut_assert_not_null(process_data);

    spi_rw_data = new spi_rw_data_t;
    cppcut_assert_not_null(spi_rw_data);

    dcpspi_init();
    spi_reset();
}

void cut_teardown()
{
    poll_results.check();

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

/*!
 * Put data into buffer, add internal protocol header.
 */
static void wrap_data_into_protocol(std::vector<uint8_t> &buffer,
                                    char cmd, uint8_t ttl, uint16_t serial,
                                    const uint8_t *data = nullptr,
                                    size_t data_size = 0)
{
    cut_assert(cmd == 'a' || cmd == 'c' || cmd == 'n');

    if(cmd == 'a' || cmd == 'n')
    {
        cppcut_assert_null(data);
        cppcut_assert_equal(size_t(0), data_size);
    }
    else
    {
        cppcut_assert_not_null(data);
        cppcut_assert_operator(size_t(UINT16_MAX), >=, data_size);
    }

    cppcut_assert_not_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_INVALID), serial);
    cppcut_assert_not_equal(uint16_t(DCPSYNC_MASTER_SERIAL_INVALID), serial);

    buffer.push_back(cmd);
    buffer.push_back(ttl);
    buffer.push_back((serial >> 8) & UINT8_MAX);
    buffer.push_back((serial >> 0) & UINT8_MAX);
    buffer.push_back((data_size >> 8) & UINT8_MAX);
    buffer.push_back((data_size >> 0) & UINT8_MAX);
    std::copy_n(data, data_size, std::back_inserter(buffer));
}

static void expect_wait_for_spi_slave(const struct timespec &t)
{
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, t);
    spi_rw_data->set<wait_for_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                       spi_rw_data_t::EXPECT_READ_ZEROS,
                                                       true);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
}

static void expect_no_more_actions()
{
    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    cut_assert_true(os_write_buffer.empty());
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);

    for(int i = 0; i < 3; ++i)
    {
        poll_results.expect(std::move(PollResult().set_return_value(0)));
        poll_results.expect(std::move(PollResult().set_errno(EINTR).set_return_value(-1)));

        cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                       expected_spi_fd,
                                       &process_data->transaction,
                                       &process_data->rldata));

        cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    }

    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_INVALID), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
}

static void run_complete_single_slave_transaction(uint16_t expected_slave_serial,
                                                  bool expecting_extra_process_message,
                                                  bool check_for_true_idle,
                                                  bool slow_request_release = false)
{
    /* slave activates the request GPIO and sends write command for UPnP
     * friendly name */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    static const std::array<uint8_t, 8> write_command
    {
        UINT8_MAX, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x58, 0x03, 0x00,
        0x61, 0x62, 0x63
    };
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, write_command);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    if(expecting_extra_process_message)
        mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DEBUG, process_transaction_message);

    mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DEBUG, process_transaction_message);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_os->check();
    mock_spi_hw->check();
    poll_results.check();

    /* send write command to DCPD */
    if(slow_request_release)
        poll_results.expect(std::move(PollResult().set_return_value(0)));
    else
    {
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
        mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    }

    char expected_process_message[256];
    snprintf(expected_process_message, sizeof(expected_process_message),
             process_transaction_message,
             TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, expected_slave_serial,
             slow_request_release ? REQSTATE_LOCKED : REQSTATE_RELEASED,
             0, size_t(0));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG, expected_process_message);
    char expected_end_message[128];

    if(slow_request_release)
        snprintf(expected_end_message, sizeof(expected_end_message),
                 "About to end transaction 0x%04x in state %d, "
                 "waiting for slave to release request line",
                 expected_slave_serial, TR_SLAVE_COMMAND_FORWARDING_TO_DCPD);
    else
        snprintf(expected_end_message, sizeof(expected_end_message),
                "End of transaction 0x%04x in state %d, return to idle state",
                expected_slave_serial, TR_SLAVE_COMMAND_FORWARDING_TO_DCPD);

    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG, expected_end_message);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(slow_request_release ? TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT : TR_IDLE,
                        process_data->transaction.state);
    std::vector<uint8_t> wrapped_write_command;
    wrap_data_into_protocol(wrapped_write_command, 'c', 0, expected_slave_serial,
                            write_command.begin() + 1, write_command.size() - 1);
    cut_assert_equal_memory(wrapped_write_command.data(), wrapped_write_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* done */
    if(check_for_true_idle)
        expect_no_more_actions();
}

/*!\test
 * Show how a transaction initiated by the SPI slave travels through the
 * system.
 */
void test_single_slave_transaction()
{
    run_complete_single_slave_transaction(DCPSYNC_SLAVE_SERIAL_MIN, true, true);
}

/*!\test
 * Regular master write followed by a regular slave write, no collisions.
 */
void test_master_write_followed_by_slave_write()
{
    /* DCPD tells slave that the next external stream URL is empty */
    static const std::array<uint8_t, 4> next_appstream_empty
    {
        DCP_COMMAND_MULTI_WRITE_REGISTER, 0xef, 0x00, 0x00,
    };
    std::vector<uint8_t> wrapped_appstream;
    wrap_data_into_protocol(wrapped_appstream, 'c', UINT8_MAX, 0xeba9,
                            next_appstream_empty.begin(), next_appstream_empty.size());
    std::copy_n(wrapped_appstream.begin(), wrapped_appstream.size(),
                std::back_inserter(os_read_buffer));

    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 0, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Master transaction: command header from DCPD: 0x02 0xef 0x00 0x00");
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(next_appstream_empty);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    std::vector<uint8_t> master_command_ack;
    wrap_data_into_protocol(master_command_ack, 'a', 0, 0xeba9);
    cut_assert_equal_memory(master_command_ack.data(), master_command_ack.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();

    /* short writes succeed within a single call of #dcpspi_process(), so we
     * can process a slave request now */
    run_complete_single_slave_transaction(DCPSYNC_SLAVE_SERIAL_MIN, true, true);
}

/*!\test
 * Regular slave write followed by a regular master write, no collisions.
 */
void test_slave_write_followed_by_master_write()
{
    /* slave sends something */
    run_complete_single_slave_transaction(DCPSYNC_SLAVE_SERIAL_MIN, true, false);

    /* done, now DCPD tells slave that the next external stream URL is empty */
    static const std::array<uint8_t, 4> next_appstream_empty
    {
        DCP_COMMAND_MULTI_WRITE_REGISTER, 0xef, 0x00, 0x00,
    };
    std::vector<uint8_t> wrapped_appstream;
    wrap_data_into_protocol(wrapped_appstream, 'c', UINT8_MAX, 0xeba9,
                            next_appstream_empty.begin(), next_appstream_empty.size());
    std::copy_n(wrapped_appstream.begin(), wrapped_appstream.size(),
                std::back_inserter(os_read_buffer));

    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 0, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Master transaction: command header from DCPD: 0x02 0xef 0x00 0x00");
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(next_appstream_empty);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    std::vector<uint8_t> master_command_ack;
    wrap_data_into_protocol(master_command_ack, 'a', 0, 0xeba9);
    cut_assert_equal_memory(master_command_ack.data(), master_command_ack.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!\test
 * Two messages sent in fast succession, ending up together in the pipe buffer.
 */
void test_two_fast_master_transactions()
{
    /* DCPD sends something through its pipe */
    static const std::array<uint8_t, 6> network_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x32, 0x02, 0x00,
        0x02, 0x01
    };
    static const std::array<uint8_t, 6> device_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x11, 0x02, 0x00,
        0x24, 0x42
    };
    std::vector<uint8_t> wrapped_network_status;
    wrap_data_into_protocol(wrapped_network_status, 'c', UINT8_MAX, 0xc830,
                            network_status.begin(), network_status.size());
    std::vector<uint8_t> wrapped_device_status;
    wrap_data_into_protocol(wrapped_device_status, 'c', UINT8_MAX, 0xc831,
                            device_status.begin(), device_status.size());
    std::copy_n(wrapped_network_status.begin(), wrapped_network_status.size(),
                std::back_inserter(os_read_buffer));
    std::copy_n(wrapped_device_status.begin(), wrapped_device_status.size(),
                std::back_inserter(os_read_buffer));

    cppcut_assert_equal(wrapped_network_status.size() + wrapped_device_status.size(),
                        os_read_buffer.size());

    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 0, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Master transaction: command header from DCPD: 0x03 0x32 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2) + wrapped_device_status.size(), os_read_buffer.size());
    cppcut_assert_equal(size_t(DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE),
                        process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();

    /* process data from DCPD remaining in pipe buffer */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 2, serial 0xc830, lock state 0, pending size 2, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cut_assert_equal_memory(wrapped_network_status.data(), wrapped_network_status.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(uint16_t(0xc830), process_data->transaction.serial);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cppcut_assert_equal(wrapped_network_status.size(), os_read_buffer.size());
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* send command to SPI slave, send ACK to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(network_status);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 3, serial 0xc830, lock state 0, pending size 0, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    std::vector<uint8_t> master_command_ack;
    wrap_data_into_protocol(master_command_ack, 'a', 0, 0xc830);
    cut_assert_equal_memory(master_command_ack.data(), master_command_ack.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();

    /* done, second command is still in pipe buffer, so there will be a poll(2)
     * event for it */
    cppcut_assert_equal(wrapped_device_status.size(), os_read_buffer.size());

    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 0, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Master transaction: command header from DCPD: 0x03 0x11 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2), os_read_buffer.size());
    cppcut_assert_equal(size_t(DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE),
                        process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();

    /* process data from DCPD remaining in pipe buffer */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 2, serial 0xc831, lock state 0, pending size 2, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cut_assert_equal_memory(wrapped_device_status.data(), wrapped_device_status.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(uint16_t(0xc831), process_data->transaction.serial);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cppcut_assert_equal(size_t(0), os_read_buffer.size());
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* send command to SPI slave, send ACK to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(device_status);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 3, serial 0xc831, lock state 0, pending size 0, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    master_command_ack.clear();
    wrap_data_into_protocol(master_command_ack, 'a', 0, 0xc831);
    cut_assert_equal_memory(master_command_ack.data(), master_command_ack.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!
 * Set up some data to be used in #create_collision_state() and related tests.
 */
static std::tuple<const std::array<uint8_t, 6> &, const std::array<uint8_t, 6>, std::vector<uint8_t>>
prepare_for_collision(uint16_t master_serial, uint8_t ttl)
{
    static const std::array<uint8_t, 6> network_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x32, 0x02, 0x00,
        0x02, 0x01
    };
    std::vector<uint8_t> wrapped_network_status;
    wrap_data_into_protocol(wrapped_network_status, 'c', ttl, master_serial,
                            network_status.begin(), network_status.size());
    std::copy_n(wrapped_network_status.begin(), wrapped_network_status.size(),
                std::back_inserter(os_read_buffer));

    static const std::array<uint8_t, 8> interrupting_slave_command
    {
        UINT8_MAX, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x58, 0x03, 0x00,
        0x61, 0x62, 0x63
    };
    std::array<uint8_t, 2> interrupting_slave_command_prefix;
    std::array<uint8_t, 6> interrupting_slave_command_suffix;

    std::copy_n(interrupting_slave_command.begin(),
                interrupting_slave_command_prefix.size(),
                interrupting_slave_command_prefix.begin());
    std::copy_n(interrupting_slave_command.begin() + interrupting_slave_command_prefix.size(),
                interrupting_slave_command_suffix.size(),
                interrupting_slave_command_suffix.begin());

    std::vector<uint8_t> wrapped_interrupting_slave_command;
    wrap_data_into_protocol(wrapped_interrupting_slave_command, 'c', 0, DCPSYNC_SLAVE_SERIAL_MIN,
                            interrupting_slave_command.begin() + 1,
                            interrupting_slave_command.size() - 1);

    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, interrupting_slave_command_prefix, true);

    return std::make_tuple(network_status, interrupting_slave_command_suffix,
                           wrapped_interrupting_slave_command);
}

static const char *gpio_trace_to_ascii_art(const uint8_t trace,
                                           unsigned int number_of_bits)
{
    static char buffer[8];
    char *bufptr = buffer;

    cppcut_assert_operator(sizeof(buffer), >, size_t(number_of_bits));
    cppcut_assert_operator(0U, <, number_of_bits);

    for(uint8_t mask = 1U << (number_of_bits - 1); mask != 0; mask >>= 1)
        *bufptr++ = ((trace & mask) == 0) ? '_' : '~';

    *bufptr = '\0';

    return buffer;
}

static void expect_detection_of_interrupt_by_slave(uint16_t expected_master_serial)
{
    char expected_message_buffer[256];

    snprintf(expected_message_buffer, sizeof(expected_message_buffer),
             "Transaction 0x%04x interrupted by slave request", expected_master_serial);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG, expected_message_buffer);
}

static void expect_detection_of_pending_slave_request(uint16_t expected_master_serial)
{
    char expected_message_buffer[256];

    snprintf(expected_message_buffer, sizeof(expected_message_buffer),
             "Pending slave request while processing transaction 0x%04x",
             expected_master_serial);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG, expected_message_buffer);
}

enum class RequestPinBehavior
{
    UNCHANGED,
    ON,
    OFF,
    ON_OFF,
    OFF_ON,
};

/*!
 * Enter a state of SPI collision by GPIO request.
 *
 * In case of any collision, the interrupting slave transaction shall take
 * priority over the master transaction. The master transaction is rejected so
 * that the slave transaction can be processed. After the slave transaction has
 * finished, the master transaction should be restarted by DCPD.
 *
 * \pre
 *     A command whose transmission shall be \e interrupted by the SPI slave
 *     must be copied into the #os_read_buffer before calling this function.
 *
 * The length of the interrupted command must be passed via parameter (because
 * the buffer could contain more data than only the interrupted command).
 *
 * \pre
 *     The first \e two bytes of data sent by the SPI slave that are
 *     \e interrupting the master transaction must be placed into the
 *     #spi_rw_data object before calling this function (use
 *     #spi_rw_data_t::set() with \p is_slave_ready_probe set to true).
 *
 * At least one of the two interrupting bytes must be a non-NOP byte, otherwise
 * there will be no collision.
 */
static void create_collision_state(const size_t expected_bytes_in_read_buffer,
                                   const size_t size_of_interrupted_message,
                                   const uint16_t expected_master_serial,
                                   const RequestPinBehavior rpb_gpio_only_when_receiving_from_dcpd,
                                   const RequestPinBehavior rpb_combined_when_receiving_from_dcpd,
                                   const RequestPinBehavior rpb_when_sending_to_slave,
                                   const uint8_t expected_nack_ttl,
                                   const bool is_nack_expected = true)
{
    cppcut_assert_operator(size_t(DCP_HEADER_SIZE), <=, size_of_interrupted_message);
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);

    char expected_message_buffer[1024];

    /* DCPD sends something through its pipe */
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 0, pending size 0, flush pos 0");
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    snprintf(expected_message_buffer, sizeof(expected_message_buffer),
             "Master transaction: command header from DCPD: "
             "0x%02x 0x%02x 0x%02x 0x%02x",
             os_read_buffer[DCPSYNC_HEADER_SIZE + 0],
             os_read_buffer[DCPSYNC_HEADER_SIZE + 1],
             os_read_buffer[DCPSYNC_HEADER_SIZE + 2],
             os_read_buffer[DCPSYNC_HEADER_SIZE + 3]);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
                                              expected_message_buffer);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(expected_bytes_in_read_buffer - DCPSYNC_HEADER_SIZE - DCP_HEADER_SIZE,
                        os_read_buffer.size());
    cppcut_assert_equal(size_t(DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE),
                        process_data->transaction.dcp_buffer.pos);
    mock_messages->check();
    mock_gpio->check();

    /* SPI slave interrupts with a write command, DCPD's data still resides in
     * the pipe buffer and is processed before the collision is detected so
     * that the full transaction is there before going any further */
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);

    bool is_gpio_active = false;
    uint8_t gpio_trace = 0;
    uint8_t gpio_toggle_count = 0;
    bool pending_slave_request_detected = false;

    switch(rpb_gpio_only_when_receiving_from_dcpd)
    {
      case RequestPinBehavior::UNCHANGED:
        poll_results.expect(std::move(PollResult().set_return_value(0)));
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_trace = (gpio_trace << 1) | 0;
        break;

      case RequestPinBehavior::ON:
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI)
                                                  .set_return_value(1)));
        is_gpio_active = true;
        mock_gpio->expect_gpio_is_active(is_gpio_active, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_trace = (gpio_trace << 1) | 1;
        ++gpio_toggle_count;
        break;

      case RequestPinBehavior::ON_OFF:
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI)
                                                  .set_return_value(1)));
        mock_gpio->expect_gpio_is_active(is_gpio_active, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_toggle_count += 2;
        break;

      case RequestPinBehavior::OFF:
      case RequestPinBehavior::OFF_ON:
        /* these behaviors violate our assumption that the GPIO was inactive */
        cut_fail("Unsupported pin behavior");
        break;
    }

    switch(rpb_combined_when_receiving_from_dcpd)
    {
      case RequestPinBehavior::UNCHANGED:
        poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN)
                                                  .set_return_value(1)));
        gpio_trace = (gpio_trace << 1) | is_gpio_active;
        gpio_trace = (gpio_trace << 1) | is_gpio_active;
        break;

      case RequestPinBehavior::ON:
        cut_assert_false(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN)
                                                  .set_gpio_events(POLLPRI)
                                                  .set_return_value(2)));
        is_gpio_active = true;
        mock_gpio->expect_gpio_is_active(is_gpio_active, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_trace = (gpio_trace << 1) | 1;
        ++gpio_toggle_count;
        break;

      case RequestPinBehavior::OFF_ON:
        cut_assert_true(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN)
                                                  .set_gpio_events(POLLPRI)
                                                  .set_return_value(2)));
        mock_gpio->expect_gpio_is_active(is_gpio_active, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_toggle_count += 2;
        break;

      case RequestPinBehavior::OFF:
        cut_assert_true(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN)
                                                  .set_gpio_events(POLLPRI)
                                                  .set_return_value(2)));
        is_gpio_active = false;
        mock_gpio->expect_gpio_is_active(is_gpio_active, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_trace = (gpio_trace << 1) | 0;
        ++gpio_toggle_count;
        break;

      case RequestPinBehavior::ON_OFF:
        cut_assert_false(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN)
                                                  .set_gpio_events(POLLPRI)
                                                  .set_return_value(2)));
        mock_gpio->expect_gpio_is_active(is_gpio_active, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_toggle_count += 2;
        break;
    }

    if(gpio_trace != 0)
    {
        /* early collision */
        expect_detection_of_interrupt_by_slave(expected_master_serial);
    }

    if(!pending_slave_request_detected && gpio_toggle_count >= 3)
    {
        /* early pending second slave request */
        expect_detection_of_pending_slave_request(expected_master_serial);
        pending_slave_request_detected = true;
    }

    mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DEBUG, process_transaction_message);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    mock_messages->check();
    const enum transaction_request_state previous_request_state =
        process_data->transaction.request_state;

    cppcut_assert_equal(TR_MASTER_COMMAND_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(expected_bytes_in_read_buffer - DCPSYNC_HEADER_SIZE - size_of_interrupted_message,
                        os_read_buffer.size());
    cppcut_assert_equal(size_of_interrupted_message + DCPSYNC_HEADER_SIZE,
                        process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(expected_master_serial, process_data->transaction.serial);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();

    /* try sending command to SPI slave, but collision is detected and NACK is
     * immediately sent to dcpd; the master transaction is discarded and
     * replaced by a slave transaction */
    switch(rpb_when_sending_to_slave)
    {
      case RequestPinBehavior::UNCHANGED:
        poll_results.expect(std::move(PollResult().set_return_value(0)));
        gpio_trace = (gpio_trace << 1) | is_gpio_active;
        gpio_trace = (gpio_trace << 1) | is_gpio_active;
        break;

      case RequestPinBehavior::ON:
        cut_assert_false(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
        mock_gpio->expect_gpio_is_active(true, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_trace = (gpio_trace << 1) | 1;
        ++gpio_toggle_count;
        break;

      case RequestPinBehavior::OFF_ON:
        cut_assert_true(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
        mock_gpio->expect_gpio_is_active(true, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_toggle_count += 2;
        break;

      case RequestPinBehavior::OFF:
        cut_assert_true(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
        mock_gpio->expect_gpio_is_active(false, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_trace = (gpio_trace << 1) | 0;
        ++gpio_toggle_count;
        break;

      case RequestPinBehavior::ON_OFF:
        cut_assert_false(is_gpio_active);
        poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
        mock_gpio->expect_gpio_is_active(false, process_data->gpio);
        gpio_trace = (gpio_trace << 1) | 1;
        gpio_trace = (gpio_trace << 1) | 0;
        gpio_toggle_count += 2;
        break;
    }

    if((gpio_trace & (0x0f << 2)) == 0 && (gpio_trace & (0x03 << 0)) != 0)
    {
        /* didn't detect this before polling, so the message is emitted a bit
         * later */
        expect_detection_of_interrupt_by_slave(expected_master_serial);
    }

    if(!pending_slave_request_detected && gpio_toggle_count >= 3)
    {
        /* later pending second slave request */
        expect_detection_of_pending_slave_request(expected_master_serial);
        pending_slave_request_detected = true;
    }

    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DEBUG, process_transaction_message);
    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
                                              "Collision detected (got funny poll bytes)");

    if(!is_nack_expected)
    {
        snprintf(expected_message_buffer, sizeof(expected_message_buffer),
                 "Silently dropping 0x%04x", expected_master_serial);
        mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG, expected_message_buffer);
    }

    cppcut_assert_equal(TR_MASTER_COMMAND_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(expected_master_serial, process_data->transaction.serial);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    mock_messages->check();
    cppcut_assert_equal(TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);

    switch(gpio_trace)
    {
      case 0x00: /* ______ */
        /* slave is very late to raise the request pin: no activation at all */
        cppcut_assert_equal(REQSTATE_IDLE,         previous_request_state);
        cppcut_assert_equal(REQSTATE_IDLE,         process_data->transaction.request_state);
        break;

      case 0x03: /* ____~~ */
        /* late assertion: single activation, pending */
        cppcut_assert_equal(REQSTATE_IDLE,         previous_request_state);
        cppcut_assert_equal(REQSTATE_LOCKED,       process_data->transaction.request_state);
        break;

      case 0x3f: /* ~~~~~~ */
      case 0x0f: /* __~~~~ */
        /* early long assertion: single activation, pending */
        cppcut_assert_equal(REQSTATE_LOCKED,       previous_request_state);
        cppcut_assert_equal(REQSTATE_LOCKED,       process_data->transaction.request_state);
        break;

      case 0x02: /* ____~_ */
        /* late short peak: single complete activation */
        cppcut_assert_equal(REQSTATE_IDLE,         previous_request_state);
        cppcut_assert_equal(REQSTATE_RELEASED,     process_data->transaction.request_state);
        break;

      case 0x20: /* ~_____ */
      case 0x08: /* __~___ */
      case 0x30: /* ~~____ */
        /* early short peak: single complete activation */
        cppcut_assert_equal(REQSTATE_RELEASED,     previous_request_state);
        cppcut_assert_equal(REQSTATE_RELEASED,     process_data->transaction.request_state);
        break;

      case 0x3c: /* ~~~~__ */
      case 0x0c: /* __~~__ */
        /* early long peak: single complete activation */
        cppcut_assert_equal(REQSTATE_LOCKED,       previous_request_state);
        cppcut_assert_equal(REQSTATE_RELEASED,     process_data->transaction.request_state);
        break;

      case 0x23: /* ~___~~ */
      case 0x33: /* ~~__~~ */
      case 0x0b: /* __~_~~ */
        /* rather early short peak, second assertion follows: one complete activation,
         * second pending */
        cppcut_assert_equal(REQSTATE_RELEASED,     previous_request_state);
        cppcut_assert_equal(REQSTATE_NEXT_PENDING, process_data->transaction.request_state);
        break;

      case 0x31: /* ~~___~ */
      case 0x3d: /* ~~~~_~ */
      case 0x0d: /* __~~_~ */
        /* rather early long peak, second assertion follows: one complete activation,
         * second pending */
        cppcut_assert_equal(REQSTATE_LOCKED,       previous_request_state);
        cppcut_assert_equal(REQSTATE_NEXT_PENDING, process_data->transaction.request_state);
        break;

      case 0x2f: /* ~_~~~~ */
      case 0x37: /* ~~_~~~ */
        /* early short peak, second assertion follows quickly: one complete activation,
         * second pending */
        cppcut_assert_equal(REQSTATE_NEXT_PENDING, previous_request_state);
        cppcut_assert_equal(REQSTATE_NEXT_PENDING, process_data->transaction.request_state);
        break;

      case 0x22: /* ~___~_ */
      case 0x28: /* ~_~___ */
        /* early short peak, second short peak follows: two complete activations */
        cppcut_assert_equal(REQSTATE_RELEASED,     previous_request_state);
        cppcut_assert_equal(REQSTATE_MISSED,       process_data->transaction.request_state);
        break;

      default:
        {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Invalid GPIO trace %s (0x%02x)",
                     gpio_trace_to_ascii_art(gpio_trace, 6), gpio_trace);
            cut_fail("%s", buffer);
        }

        break;
    }

    cppcut_assert_equal(size_of_interrupted_message + DCPSYNC_HEADER_SIZE,
                        process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.serial);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);

    if(is_nack_expected)
    {
        std::vector<uint8_t> master_command_nack;
        wrap_data_into_protocol(master_command_nack, 'n',
                                expected_nack_ttl, expected_master_serial);
        cut_assert_equal_memory(master_command_nack.data(), master_command_nack.size(),
                                os_write_buffer.data(), os_write_buffer.size());
        os_write_buffer.clear();
    }
    else
        cut_assert_true(os_write_buffer.empty());

    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();
}

/*!\test
 * Collision: Slave sends data bytes when being polled for ready state, GPIO
 * request present, but not done before data is fetched from slave.
 *
 * Several variations with respect to timing are possible and are sometimes
 * handled slightly differently, but the outcome should always be the same.
 *
 * See also ticket #161.
 */
static void
collision_with_open_transaction_request(const RequestPinBehavior rpb_gpio_only_when_receiving_from_dcpd,
                                        const RequestPinBehavior rpb_combined_when_receiving_from_dcpd,
                                        const RequestPinBehavior rpb_when_sending_to_slave)
{
    const auto &prepared_data(prepare_for_collision(0xb934, UINT8_MAX));
    const auto &network_status(std::get<0>(prepared_data));
    const auto &interrupting_slave_command_suffix(std::get<1>(prepared_data));
    const auto &wrapped_interrupting_slave_command(std::get<2>(prepared_data));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE,
                           network_status.size(), 0xb934,
                           rpb_gpio_only_when_receiving_from_dcpd,
                           rpb_combined_when_receiving_from_dcpd,
                           rpb_when_sending_to_slave, 9);

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name, request pin still active */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, interrupting_slave_command_suffix);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 1, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN), process_data->transaction.serial);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();

    /* slave transaction: send write command to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0001, lock state 1, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "About to end transaction 0x0001 in state 8, waiting for slave to release request line");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);
    cppcut_assert_equal(wrapped_interrupting_slave_command.size(),
                        process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(wrapped_interrupting_slave_command.size()),
                        process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* nice, now the SPI slave releases its request line, a bit slow today */
    cppcut_assert_equal(size_t(13), wrapped_interrupting_slave_command.size());
    mock_messages->expect_msg_info_formatted("Waiting for slave to deassert the request pin");
    mock_messages->expect_msg_info_formatted("Slave has deasserted the request pin");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 9, serial 0x0001, lock state 2, pending size 0, flush pos 13");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 9, return to idle state");
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!
 * Collision: Slave sends data bytes when being polled for ready state, GPIO
 * request present and done before data is fetched from slave.
 *
 * Several variations with respect to timing are possible and are sometimes
 * handled slightly differently, but the outcome should always be the same.
 */
static void
collision_with_full_transaction_request(const RequestPinBehavior rpb_gpio_only_when_receiving_from_dcpd,
                                        const RequestPinBehavior rpb_combined_when_receiving_from_dcpd,
                                        const RequestPinBehavior rpb_when_sending_to_slave)
{
    const auto &prepared_data(prepare_for_collision(0xa678, UINT8_MAX));
    const auto &network_status(std::get<0>(prepared_data));
    const auto &interrupting_slave_command_suffix(std::get<1>(prepared_data));
    const auto &wrapped_interrupting_slave_command(std::get<2>(prepared_data));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE,
                           network_status.size(), 0xa678,
                           rpb_gpio_only_when_receiving_from_dcpd,
                           rpb_combined_when_receiving_from_dcpd,
                           rpb_when_sending_to_slave, 9);

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name, request pin not active anymore */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, interrupting_slave_command_suffix);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN), process_data->transaction.serial);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();

    /* slave transaction: send write command to DCPD, and we are done */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0001, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 8, return to idle state");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_INVALID), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!
 * Collision: Slave sends data, request is closed and followed by next, open
 * request.
 *
 * Happens if the slave decides to interrupt us with a burst of transactions
 * and behaves civilized.
 *
 * Several variations with respect to timing are possible and are sometimes
 * handled slightly differently, but the outcome should always be the same.
 */
static void collision_with_full_request_followed_by_open_request(
        const RequestPinBehavior rpb_gpio_only_when_receiving_from_dcpd,
        const RequestPinBehavior rpb_combined_when_receiving_from_dcpd,
        const RequestPinBehavior rpb_when_sending_to_slave)
{
    const auto &prepared_data(prepare_for_collision(0xdf86, UINT8_MAX));
    const auto &network_status(std::get<0>(prepared_data));
    const auto &interrupting_slave_command_suffix(std::get<1>(prepared_data));
    const auto &wrapped_interrupting_slave_command(std::get<2>(prepared_data));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE,
                           network_status.size(), 0xdf86,
                           rpb_gpio_only_when_receiving_from_dcpd,
                           rpb_combined_when_receiving_from_dcpd,
                           rpb_when_sending_to_slave, 9);

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name, request pin still active */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, interrupting_slave_command_suffix);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 3, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN), process_data->transaction.serial);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();

    /* slave transaction: send write command to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0001, lock state 3, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 8, slave request pending");
    mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DIAG,
                                    "Processing pending slave transaction");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 1, pending size 0, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_INVALID), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* pending slave transaction: slave sends write command for DRC command */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    static const std::array<uint8_t, 5> drcp_store_favorites
    {
        DCP_COMMAND_MULTI_WRITE_REGISTER, 0x48, 0x01, 0x00, 0x2c
    };
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, drcp_store_favorites);
    expect_wait_for_spi_slave(dummy_time);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 1, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x48 0x01 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN + 1), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* pending slave transaction: slave deasserts request pin, we forward the
     * command to DCPD */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0002, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0002 in state 8, return to idle state");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    std::vector<uint8_t> wrapped_drcp_store_favorites;
    wrap_data_into_protocol(wrapped_drcp_store_favorites, 'c', 0,
                            DCPSYNC_SLAVE_SERIAL_MIN + 1,
                            drcp_store_favorites.begin(), drcp_store_favorites.size());
    cut_assert_equal_memory(wrapped_drcp_store_favorites.data(),
                            wrapped_drcp_store_favorites.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!\test
 * Collision: Slave sends data bytes, request is very early and stays asserted.
 */
void test_collision_with_slow_early_request()
{
    collision_with_open_transaction_request(RequestPinBehavior::ON,
                                            RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data bytes, request is early and stays asserted.
 */
void test_collision_with_slow_later_request()
{
    collision_with_open_transaction_request(RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::ON,
                                            RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data bytes, request is late and stays asserted.
 */
void test_collision_with_slow_late_request()
{
    collision_with_open_transaction_request(RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::ON);
}

/*!\test
 * Collision: Slave sends data bytes, request is early and long.
 */
void test_collision_with_fast_early_request_release()
{
    collision_with_full_transaction_request(RequestPinBehavior::ON,
                                            RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::OFF);
}

/*!\test
 * Collision: Slave sends data bytes, request is early and not so long.
 */
void test_collision_with_faster_early_request_release()
{
    collision_with_full_transaction_request(RequestPinBehavior::ON,
                                            RequestPinBehavior::OFF,
                                            RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data bytes, request is later and long.
 */
void test_collision_with_fast_later_request_release()
{
    collision_with_full_transaction_request(RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::ON,
                                            RequestPinBehavior::OFF);
}

/*!\test
 * Collision: Slave sends data bytes, request is early and short.
 */
void test_collision_with_very_fast_early_request_release()
{
    collision_with_full_transaction_request(RequestPinBehavior::ON_OFF,
                                            RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data bytes, request is later and short.
 */
void test_collision_with_very_fast_later_request_release()
{
    collision_with_full_transaction_request(RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::ON_OFF,
                                            RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data bytes, request is late and short.
 */
void test_collision_with_very_fast_late_request_release()
{
    collision_with_full_transaction_request(RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::UNCHANGED,
                                            RequestPinBehavior::ON_OFF);
}

/*!\test
 * Collision: Slave sends data, full request is followed by open request.
 */
void test_collision_with_follow_up_request_simple()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::ON,
                                                         RequestPinBehavior::OFF,
                                                         RequestPinBehavior::ON);
}

/*!\test
 * Collision: Slave sends data, requests transaction, quickly requests next.
 */
void test_collision_with_follow_up_request_new_request_early_just_in_time()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::ON,
                                                         RequestPinBehavior::OFF_ON,
                                                         RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data, requests transaction, quickly requests next.
 */
void test_collision_with_follow_up_request_new_request_late_just_in_time()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::ON,
                                                         RequestPinBehavior::UNCHANGED,
                                                         RequestPinBehavior::OFF_ON);
}

/*!\test
 * Collision: Slave sends data, requests transaction, quickly requests next.
 */
void test_collision_with_follow_up_request_new_request_early()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::ON_OFF,
                                                         RequestPinBehavior::ON,
                                                         RequestPinBehavior::UNCHANGED);
}

/*!\test
 * Collision: Slave sends data, requests transaction, quickly requests next.
 */
void test_collision_with_follow_up_request_new_request_late()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::ON_OFF,
                                                         RequestPinBehavior::UNCHANGED,
                                                         RequestPinBehavior::ON);
}

/*!\test
 * Collision: Slave sends data, requests transaction, quickly requests next.
 */
void test_collision_with_follow_up_request_first_request_late()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::UNCHANGED,
                                                         RequestPinBehavior::ON_OFF,
                                                         RequestPinBehavior::ON);
}

/*!\test
 * Collision: Slave sends data, requests transaction, quickly requests next.
 */
void test_collision_with_follow_up_request_first_request_late_new_request_just_in_time()
{
    collision_with_full_request_followed_by_open_request(RequestPinBehavior::UNCHANGED,
                                                         RequestPinBehavior::ON,
                                                         RequestPinBehavior::OFF_ON);
}

/*!
 * Collision: Slave requests transaction, then requests the next one and sends
 * the full second request together with the first one.
 *
 * This situation occurs in case the load on the Streaming Board is high and
 * the slave is sending two subsequent requests quickly one after the other
 * while a master transaction is about to start. In this case, dcpspi may not
 * even see the first request and runs into an SPI collision. Two bytes are
 * read by dcpspi, signaling to the slave that its GPIO request has been seen
 * (even though it hasn't yet). The DCP header isn't fully there yet, so the
 * next read from SPI slave is for a full chunk of 32 bytes. In the meantime,
 * the slave has already prepared the next request and asserted the request pin
 * again. It sends the rest of the first request and then sends the second
 * request right after it.
 *
 * In theory (and rarely in practice), this effect can stack, meaning that
 * multiple small requests may reside in the SPI input buffer for very unlucky
 * timings and successive collisions. The communication temporarily falls out
 * of sync as transaction requests are lost while processing some transaction
 * because the Streaming Board fails to keep up with the speed of the SPI
 * slave.
 *
 * The solution that dcpspi implements is that it peeks into the SPI input
 * buffer after being done handling any requests performed after a collision if
 * the request GPIO has been toggled in the meantime in such a way that lost
 * transaction requests are indicated. If there is any pending request data, it
 * will be processed. The input buffer must begin with one or two zeros (the
 * ready signal) for this to work, otherwise it must be completely discarded
 * (we cannot actively search for DCP messages in arbitrary buffers because the
 * protocol lacks sync marks, and the binary nature of the protocol makes
 * guessing risky).
 */
void test_collision_with_lost_and_found_slave_request()
{
    const auto &prepared_data(prepare_for_collision(0x920b, UINT8_MAX));
    const auto &network_status(std::get<0>(prepared_data));
    const auto &interrupting_slave_command_suffix(std::get<1>(prepared_data));
    const auto &wrapped_interrupting_slave_command(std::get<2>(prepared_data));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE,
                           network_status.size(), 0x920b,
                           RequestPinBehavior::ON,
                           RequestPinBehavior::OFF,
                           RequestPinBehavior::UNCHANGED, 9);

    /* next command tacked to the end of the first one, sent by slave because
     * of unlucky timing; there could be a UINT8_MAX because first poll byte is
     * usually that, and the 0x00 (sometimes 2 of them instead of UINT8_MAX) is
     * the ready signal because the slave thought it's being polled */
    static const std::array<uint8_t, 17> follow_up_slave_command
    {
        UINT8_MAX, 0x00, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x79, 0x0b, 0x00,
        0x51, 0xc3, 0x01, 0xa9, 0xe4, 0x2e, 0x03, 0x3a, 0x02, 0x01, 0xfb,
    };
    cppcut_assert_equal(size_t(6), interrupting_slave_command_suffix.size());
    std::array<uint8_t, 6 + follow_up_slave_command.size()> two_commands;

    std::copy_n(interrupting_slave_command_suffix.data(),
                interrupting_slave_command_suffix.size(),
                two_commands.begin());
    std::copy_n(follow_up_slave_command.data(),
                follow_up_slave_command.size(),
                two_commands.begin() + interrupting_slave_command_suffix.size());

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, two_commands);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN), process_data->transaction.serial);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();

    /* slave transaction: slave has prepared the next request and sends it
     * while we are sending the first command to DCPD */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Pending slave request while processing transaction 0x0001");
    mock_messages->expect_msg_error_formatted(0, LOG_WARNING,
        "Lost slave request while processing transaction 0x0001");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0001, lock state 4, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 8, looking for missed transactions");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 4, pending size 0, flush pos 0");
    mock_messages->expect_msg_info_formatted("Possibly found lost packet(s) in SPI input buffer");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x79 0x0b 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    std::vector<uint8_t> wrapped_second_slave_command;
    wrap_data_into_protocol(wrapped_second_slave_command, 'c', 0, DCPSYNC_SLAVE_SERIAL_MIN + 1,
                            follow_up_slave_command.begin() + 2,
                            follow_up_slave_command.size() - 2);
    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN + 1), process_data->transaction.serial);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    cut_assert_equal_memory(wrapped_second_slave_command.data(),
                            wrapped_second_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();

    /* second slave transaction: send to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0002, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0002 in state 8, return to idle state");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_INVALID), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(wrapped_second_slave_command.data(),
                            wrapped_second_slave_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!\test
 * Collision occurs, but DCPD doesn't want to know (anymore).
 */
void test_collision_with_command_ttl_of_0()
{
    const auto &prepared_data(prepare_for_collision(0x81f7, 0));
    const auto &network_status(std::get<0>(prepared_data));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE,
                           network_status.size(), 0x81f7,
                           RequestPinBehavior::ON,
                           RequestPinBehavior::UNCHANGED,
                           RequestPinBehavior::OFF,
                           23, false);

    /* no NACK was sent, slave transaction should be processed next (we do not
     * do this here, though, to keep the test small */
}

/*!\test
 * Collision occurs, DCPD should not retry anymore.
 */
void test_collision_with_command_ttl_of_1()
{
    const auto &prepared_data(prepare_for_collision(0xf094, 1));
    const auto &network_status(std::get<0>(prepared_data));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE,
                           network_status.size(), 0xf094,
                           RequestPinBehavior::ON,
                           RequestPinBehavior::UNCHANGED,
                           RequestPinBehavior::OFF,
                           0);

    /* final NACK was sent, slave transaction should be processed next (we do
     * not do this here, though, to keep the test small */
}

/*!\test
 * Collision occurs, master has sent a burst of packets without waiting for
 * ACKs.
 *
 * This can happen because DCPD may want to push multiple packets into the pipe
 * buffer in the hope of higher throughput as long as no collision occurs.
 * In case of a collision with a slave transaction, however, the most recent
 * master transaction is going to be rejected and DCPD must send it again.
 *
 * Note that this will lead to reordering of packets, leading to messed up data
 * in case of long data fragmented over multiple packets (such as DRC data)>.
 * Consequently, DCPD must make sure to synchronize on ACKs to avoid this where
 * necessary.
 *
 * This test is a bit like #test_two_fast_master_transactions(), but with a
 * collision strayed in.
 */
void test_collision_with_multiple_master_transactions_in_pipe_buffer()
{
    const auto &prepared_data(prepare_for_collision(0xc0e5, UINT8_MAX));
    const auto &network_status(std::get<0>(prepared_data));
    const auto &interrupting_slave_command_suffix(std::get<1>(prepared_data));
    const auto &wrapped_interrupting_slave_command(std::get<2>(prepared_data));

    static const std::array<uint8_t, 6> device_status
    {
        DCP_COMMAND_MULTI_READ_REGISTER, 0x11, 0x02, 0x00,
        0x24, 0x42
    };
    std::vector<uint8_t> wrapped_device_status;
    wrap_data_into_protocol(wrapped_device_status, 'c', UINT8_MAX, 0xc0e6,
                            device_status.begin(), device_status.size());
    std::copy_n(wrapped_device_status.begin(), wrapped_device_status.size(),
                std::back_inserter(os_read_buffer));

    create_collision_state(network_status.size() + DCPSYNC_HEADER_SIZE + wrapped_device_status.size(),
                           network_status.size(), 0xc0e5,
                           RequestPinBehavior::ON,
                           RequestPinBehavior::UNCHANGED,
                           RequestPinBehavior::OFF,
                           9);

    /* switch over to slave transaction: slave sends write command for UPnP
     * friendly name, request pin not active anymore */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, interrupting_slave_command_suffix);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_MIN), process_data->transaction.serial);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();

    /* slave transaction: send write command to DCPD, and we are done */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0001, lock state 2, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 8, return to idle state");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(DCPSYNC_SLAVE_SERIAL_INVALID), process_data->transaction.serial);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cut_assert_equal_memory(wrapped_interrupting_slave_command.data(),
                            wrapped_interrupting_slave_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    cppcut_assert_equal(size_t(0), process_data->transaction.dcp_buffer.pos);
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* slave transaction done, but there is another master transaction sitting
     * in the pipe buffer */
    cppcut_assert_equal(wrapped_device_status.size(), os_read_buffer.size());

    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 0, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Master transaction: command header from DCPD: 0x03 0x11 0x02 0x00");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_RECEIVING_DATA_FROM_DCPD, process_data->transaction.state);
    cppcut_assert_equal(size_t(2), os_read_buffer.size());
    cppcut_assert_equal(size_t(DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE),
                        process_data->transaction.dcp_buffer.pos);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();

    /* process data from DCPD remaining in pipe buffer */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    poll_results.expect(std::move(PollResult().set_dcpd_events(POLLIN).set_return_value(1)));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 2, serial 0xc0e6, lock state 0, pending size 2, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_MASTER_COMMAND_FORWARDING_TO_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_IDLE, process_data->transaction.request_state);
    cut_assert_equal_memory(wrapped_device_status.data(), wrapped_device_status.size(),
                            process_data->transaction.dcp_buffer.buffer,
                            process_data->transaction.dcp_buffer.pos);
    cppcut_assert_equal(uint16_t(0xc0e6), process_data->transaction.serial);
    cppcut_assert_equal(size_t(0), process_data->transaction.spi_buffer.pos);
    cppcut_assert_equal(uint16_t(0), process_data->transaction.pending_size_of_transaction);
    cppcut_assert_equal(size_t(0), process_data->transaction.flush_to_dcpd_buffer_pos);
    cppcut_assert_equal(size_t(0), os_read_buffer.size());
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* send command to SPI slave, send ACK to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));
    expect_wait_for_spi_slave(dummy_time);
    spi_rw_data->set(device_status);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 3, serial 0xc0e6, lock state 0, pending size 0, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    std::vector<uint8_t> master_command_ack;
    wrap_data_into_protocol(master_command_ack, 'a', 0, 0xc0e6);
    cut_assert_equal_memory(master_command_ack.data(), master_command_ack.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    mock_spi_hw->check();
    poll_results.check();

    /* done */
    expect_no_more_actions();
}

/*!\test
 * In case the SPI slave sends junk for whatever reason, then we simply forward
 * it to DCPD to deal with it.
 */
void test_junk_is_forwarded_to_dcpd()
{
    /* slave activates the request GPIO and sends junk */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    static const std::array<uint8_t, 10> junk_bytes
    {
        0x80, 0x01, 0xfe, 0x9a, 0x77, 0x4c, 0xe3, 0x00, 0x6a, 0xdb,
    };
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, junk_bytes);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 1, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 1, pending size 0, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x80 0x01 0xfe 0x9a");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_os->check();
    mock_spi_hw->check();
    poll_results.check();

    /* send junk to DCPD */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 8, serial 0x0001, lock state 2, pending size 10, flush pos 0");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 8, return to idle state");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    std::array<uint8_t, DCP_HEADER_SIZE> junk_bytes_prefix;
    std::copy_n(junk_bytes.begin(), junk_bytes_prefix.size(), junk_bytes_prefix.begin());
    std::vector<uint8_t> wrapped_junk_bytes_prefix;
    wrap_data_into_protocol(wrapped_junk_bytes_prefix, 'c', 0, DCPSYNC_SLAVE_SERIAL_MIN,
                            junk_bytes_prefix.begin(), junk_bytes_prefix.size());
    cut_assert_equal_memory(wrapped_junk_bytes_prefix.data(),
                            DCPSYNC_HEADER_SIZE + DCP_HEADER_SIZE,
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    /* see what's going on with the junk that is still in the SPI input buffer
     * on next transaction */
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 1, pending size 0, flush pos 0");
    mock_messages->expect_msg_info_formatted(
        "Discarding 6 bytes from SPI receive buffer");

    run_complete_single_slave_transaction(0x0002, false, true);
}

/*!\test
 * In case the slave tries to send two successive messages, but is sloppy with
 * the request signal, then we may lose a message (depending on implementation
 * on slave side) and end up recovering via timeout.
 *
 * Processing continues after our internal timeout has expired.
 */
void test_lost_transaction_and_timeout_if_slave_deasserts_request_line_too_soon()
{
    run_complete_single_slave_transaction(DCPSYNC_SLAVE_SERIAL_MIN, true, false, true);

    /* at this point, we have a transaction in a state waiting for deasserting
     * the request pin; let's assume the SPI slave toggles the request line too
     * quick for us to see the deasserted state, and the next read-out yields
     * an asserted state again; we can actually detect this situation and react
     * accordingly */
    cppcut_assert_equal(TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);

    mock_messages->expect_msg_info_formatted("Waiting for slave to deassert the request pin");
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_info_formatted("Slave has deasserted the request pin (and has asserted it again)");
    expect_detection_of_pending_slave_request(DCPSYNC_SLAVE_SERIAL_MIN);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 9, serial 0x0001, lock state 3, pending size 0, flush pos 13");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 9, slave request pending");
    mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DIAG,
                                    "Processing pending slave transaction");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 1, pending size 0, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    mock_messages->check();
    mock_gpio->check();
    mock_os->check();

    /* we know there is (or at least, was) a pending request and we are going
     * to process it */
    cppcut_assert_equal(TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);

    /* the slave has already released the GPIO pin in the meantime (which must
     * be considered a bug in the implementation on slave side) and doesn't
     * have anything for us anymore */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_error_formatted(0, LOG_CRIT,
        "APPLIANCE BUG: Transaction was requested by slave, but request pin is deasserted now. "
        "We will try to process this pending transaction anyway.");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 2, pending size 0, flush pos 0");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    spi_rw_data->set<read_from_slave_spi_transfer_size>(spi_rw_data_t::EXPECT_WRITE_NOPS,
                                                        spi_rw_data_t::EXPECT_READ_NOPS,
                                                        false);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    mock_os->expect_os_nanosleep(0, delay_between_slave_probes_ms);
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    /* expire multiple timeouts */
    struct timespec expired_time = dummy_time;
    for(int i = 0; i < 4; ++i)
    {
        spi_rw_data->set<read_from_slave_spi_transfer_size>(
                                            spi_rw_data_t::EXPECT_WRITE_NOPS,
                                            spi_rw_data_t::EXPECT_READ_NOPS,
                                            false);
        mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
        ++expired_time.tv_sec;
        mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, expired_time);
        mock_os->expect_os_nanosleep(0, delay_between_slave_probes_ms);
        mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, expired_time);
    }
    spi_rw_data->set<read_from_slave_spi_transfer_size>(
                                            spi_rw_data_t::EXPECT_WRITE_NOPS,
                                            spi_rw_data_t::EXPECT_READ_NOPS,
                                            false);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    ++expired_time.tv_sec;
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, expired_time);

    mock_messages->expect_msg_error_formatted(0, LOG_NOTICE,
        "SPI read timeout, returning 0 of 4 bytes");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0000 in state 6, return to idle state");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));
    mock_messages->check();
    mock_gpio->check();
    mock_os->check();

    expect_no_more_actions();
}

/*!\test
 * In case the slave tries to send two successive messages, but is sloppy with
 * the request signal, then we may still receive a message (depending on
 * implementation on slave side) we can process.
 */
void test_new_transaction_even_if_slave_deasserts_request_line_too_soon()
{
    run_complete_single_slave_transaction(DCPSYNC_SLAVE_SERIAL_MIN, true, false, true);

    /* at this point, we have a transaction in a state waiting for deasserting
     * the request pin; let's assume the SPI slave toggles the request line too
     * quick for us to see the deasserted state, and the next read-out yields
     * an asserted state again; we can actually detect this situation and react
     * accordingly */
    cppcut_assert_equal(TR_SLAVE_COMMAND_WAIT_FOR_REQUEST_DEASSERT, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);

    mock_messages->expect_msg_info_formatted("Waiting for slave to deassert the request pin");
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(true, process_data->gpio);
    mock_messages->expect_msg_info_formatted("Slave has deasserted the request pin (and has asserted it again)");
    expect_detection_of_pending_slave_request(DCPSYNC_SLAVE_SERIAL_MIN);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 9, serial 0x0001, lock state 3, pending size 0, flush pos 13");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "End of transaction 0x0001 in state 9, slave request pending");
    mock_messages->expect_msg_vinfo(MESSAGE_LEVEL_DIAG,
                                    "Processing pending slave transaction");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 0, serial 0x0000, lock state 1, pending size 0, flush pos 0");

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    /* we know there is (or at least, was) a pending request and we are going
     * to process it */
    cppcut_assert_equal(TR_SLAVE_COMMAND_RECEIVING_HEADER_FROM_SLAVE, process_data->transaction.state);
    cppcut_assert_equal(REQSTATE_LOCKED, process_data->transaction.request_state);

    /* the slave has already released the GPIO pin in the meantime (which must
     * be considered a bug in the implementation on slave side), but there are
     * still data the slave is willing to give to us */
    poll_results.expect(std::move(PollResult().set_gpio_events(POLLPRI).set_return_value(1)));
    mock_gpio->expect_gpio_is_active(false, process_data->gpio);
    mock_messages->expect_msg_error_formatted(0, LOG_CRIT,
        "APPLIANCE BUG: Transaction was requested by slave, but request pin is deasserted now. "
        "We will try to process this pending transaction anyway.");
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG,
        "Process transaction state 6, serial 0x0000, lock state 2, pending size 0, flush pos 0");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);
    static const std::array<uint8_t, 8> write_command
    {
        UINT8_MAX, DCP_COMMAND_MULTI_WRITE_REGISTER, 0x58, 0x03, 0x00,
        0x61, 0x62, 0x63
    };
    spi_rw_data->set(spi_rw_data_t::EXPECT_WRITE_NOPS, write_command);
    mock_spi_hw->expect_spi_hw_do_transfer_callback(mock_spi_transfer);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DIAG,
        "Slave transaction: command header from SPI: 0x02 0x58 0x03 0x00");
    mock_os->expect_os_clock_gettime(0, 0, CLOCK_MONOTONIC_RAW, dummy_time);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    static const uint16_t expected_slave_serial = DCPSYNC_SLAVE_SERIAL_MIN + 1;
    cppcut_assert_equal(TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, process_data->transaction.state);
    cppcut_assert_equal(expected_slave_serial, process_data->transaction.serial);
    cut_assert_true(os_write_buffer.empty());
    mock_messages->check();
    mock_gpio->check();
    mock_os->check();
    mock_spi_hw->check();
    poll_results.check();

    /* send write command to DCPD */
    poll_results.expect(std::move(PollResult().set_return_value(0)));

    char expected_process_message[256];
    snprintf(expected_process_message, sizeof(expected_process_message),
             process_transaction_message,
             TR_SLAVE_COMMAND_FORWARDING_TO_DCPD, expected_slave_serial,
             REQSTATE_RELEASED, 0, size_t(0));
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG, expected_process_message);
    char expected_end_message[128];
    snprintf(expected_end_message, sizeof(expected_end_message),
             "End of transaction 0x%04x in state %d, return to idle state",
             expected_slave_serial, TR_SLAVE_COMMAND_FORWARDING_TO_DCPD);
    mock_messages->expect_msg_vinfo_formatted(MESSAGE_LEVEL_DEBUG, expected_end_message);

    cut_assert_true(dcpspi_process(expected_fifo_in_fd, expected_fifo_out_fd,
                                   expected_spi_fd, &process_data->transaction,
                                   &process_data->rldata));

    cppcut_assert_equal(TR_IDLE, process_data->transaction.state);
    std::vector<uint8_t> wrapped_write_command;
    wrap_data_into_protocol(wrapped_write_command, 'c', 0, expected_slave_serial,
                            write_command.begin() + 1, write_command.size() - 1);
    cut_assert_equal_memory(wrapped_write_command.data(), wrapped_write_command.size(),
                            os_write_buffer.data(), os_write_buffer.size());
    os_write_buffer.clear();
    mock_messages->check();
    mock_gpio->check();
    poll_results.check();

    expect_no_more_actions();
}

}
