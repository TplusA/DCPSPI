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
    const std::string string_;

    explicit Expectation(int error_code, int priority,
                         const char *string, bool is_format_string):
        error_code_(error_code),
        priority_(priority),
        is_format_string_(is_format_string),
        string_(string)
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

static void check_message_expectation(int error_code, int priority,
                                      const char *format_string, va_list va)
{
    const auto &expect(mock_messages_singleton->expectations_->get_next_expectation(format_string));

    cppcut_assert_equal(expect.error_code_, error_code);
    cppcut_assert_equal(expect.priority_, priority);

    if(expect.is_format_string_)
    {
        cppcut_assert_equal(expect.string_, std::string(format_string));
        return;
    }

    char buffer[512];
    size_t len = vsnprintf(buffer, sizeof(buffer), format_string, va);

    if(error_code != 0 && len < sizeof(buffer))
        snprintf(buffer + len, sizeof(buffer) - len,
                 " (%s)", strerror(error_code));

    cppcut_assert_equal(expect.string_, std::string(buffer));
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
