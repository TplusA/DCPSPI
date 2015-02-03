/*
 * Copyright (C) 2015  T+A elektroakustik GmbH & Co. KG
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

#include <cppcutter.h>

#include <vector>
#include <string>
#include <memory>
#include <cstdarg>
#include <cstring>
#include <cstdio>

#include "mock_messages.hh"

class MockMessages::Expectation
{
  private:
    Expectation(const Expectation &);
    Expectation &operator=(const Expectation &);

  public:
    const int error_code_;
    const int priority_;

    const bool is_format_string_;
    const bool is_complete_string_;
    const std::string string_;
    const std::string suffix_;

    explicit Expectation(int error_code, int priority,
                         const char *string, bool is_format_string):
        error_code_(error_code),
        priority_(priority),
        is_format_string_(is_format_string),
        is_complete_string_(true),
        string_(string)
    {}

    explicit Expectation(int error_code, int priority,
                         const char *prefix, const char *suffix,
                         bool is_format_string):
        error_code_(error_code),
        priority_(priority),
        is_format_string_(is_format_string),
        is_complete_string_(false),
        string_(prefix),
        suffix_(suffix)
    {}

    Expectation(Expectation &&) = default;
};


MockMessages::MockMessages():
    ignore_all_(false)
{
    expectations_ = new MockExpectations();
}

MockMessages::~MockMessages()
{
    delete expectations_;
}

void MockMessages::init()
{
    cppcut_assert_not_null(expectations_);
    expectations_->init();
}

void MockMessages::check() const
{
    cppcut_assert_not_null(expectations_);
    expectations_->check();
}

void MockMessages::expect_msg_error_formatted(int error_code, int priority,
                                              const char *string)
{
    expectations_->add(Expectation(error_code, priority, string, false));
}

void MockMessages::expect_msg_error_formatted(int error_code, int priority,
                                              const char *prefix, const char *suffix)
{
    expectations_->add(Expectation(error_code, priority, prefix, suffix, false));
}

void MockMessages::expect_msg_error(int error_code, int priority,
                                    const char *string)
{
    expectations_->add(Expectation(error_code, priority, string, true));
}

void MockMessages::expect_msg_info_formatted(const char *string)
{
    expectations_->add(Expectation(0, LOG_INFO, string, false));
}

void MockMessages::expect_msg_info(const char *string)
{
    expectations_->add(Expectation(0, LOG_INFO, string, true));
}


MockMessages *mock_messages_singleton = nullptr;

static void check_prefix_and_suffix(const std::string &expected_prefix,
                                    const std::string &expected_suffix,
                                    const char *string)
{
    size_t string_length = strlen(string);

    cppcut_assert_operator(expected_prefix.length() + expected_suffix.length(), <=, string_length);
    cppcut_assert_equal(expected_prefix, std::string(string, expected_prefix.length()));
    cppcut_assert_equal(expected_suffix, std::string(string + string_length - expected_suffix.length(),
                                                     expected_suffix.length()));
}

static void check_message_expectation(int error_code, int priority,
                                      const char *format_string, va_list va)
{
    const auto &expect(mock_messages_singleton->expectations_->get_next_expectation(format_string));

    cppcut_assert_equal(expect.error_code_, error_code);
    cppcut_assert_equal(expect.priority_, priority);

    if(expect.is_format_string_)
    {
        if(expect.is_complete_string_)
            cppcut_assert_equal(expect.string_, std::string(format_string));
        else
            check_prefix_and_suffix(expect.string_, expect.suffix_, format_string);

        return;
    }

    char buffer[512];
    size_t len = vsnprintf(buffer, sizeof(buffer), format_string, va);

    if(error_code != 0 && len < sizeof(buffer))
        snprintf(buffer + len, sizeof(buffer) - len,
                 " (%s)", strerror(error_code));

    if(expect.is_complete_string_)
        cppcut_assert_equal(expect.string_, std::string(buffer));
    else
        check_prefix_and_suffix(expect.string_, expect.suffix_, buffer);
}

void msg_enable_syslog(bool enable_syslog) {}

void msg_error(int error_code, int priority, const char *error_format, ...)
{
    if(mock_messages_singleton->ignore_all_)
        return;

    va_list va;

    va_start(va, error_format);
    check_message_expectation(error_code, priority, error_format, va);
    va_end(va);
}

void msg_info(const char *format_string, ...)
{
    if(mock_messages_singleton->ignore_all_)
        return;

    va_list va;

    va_start(va, format_string);
    check_message_expectation(0, LOG_INFO, format_string, va);
    va_end(va);
}
