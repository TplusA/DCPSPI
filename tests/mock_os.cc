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

#include "mock_os.hh"

enum class OsFn
{
    os_clock_gettime_fn,
    os_nanosleep_fn,
    stdlib_abort,

    first_valid_os_fn_id = os_clock_gettime_fn,
    last_valid_os_fn_id = stdlib_abort,
};


static std::ostream &operator<<(std::ostream &os, const OsFn id)
{
    if(id < OsFn::first_valid_os_fn_id ||
       id > OsFn::last_valid_os_fn_id)
    {
        os << "INVALID";
        return os;
    }

    switch(id)
    {
      case OsFn::os_clock_gettime_fn:
        os << "os_clock_gettime";
        break;

      case OsFn::os_nanosleep_fn:
        os << "os_nanosleep";
        break;

      case OsFn::stdlib_abort:
        os << "abort";
        break;
    }

    os << "()";

    return os;
}

class MockOs::Expectation
{
  public:
    const OsFn function_id_;

    const int ret_code_;
    clockid_t arg_clk_id_;
    const struct timespec timespec_;
    os_clock_gettime_callback_t os_clock_gettime_callback_;

    Expectation(const Expectation &) = delete;
    Expectation &operator=(const Expectation &) = delete;

    explicit Expectation(int ret, clockid_t clk_id, const struct timespec &ret_tp):
        function_id_(OsFn::os_clock_gettime_fn),
        ret_code_(ret),
        arg_clk_id_(clk_id),
        timespec_(ret_tp),
        os_clock_gettime_callback_(nullptr)
    {}

    explicit Expectation(os_clock_gettime_callback_t fn):
        function_id_(OsFn::os_clock_gettime_fn),
        ret_code_(-5),
        arg_clk_id_(CLOCK_REALTIME_COARSE),
        timespec_({0}),
        os_clock_gettime_callback_(fn)
    {}

    explicit Expectation(const struct timespec &tp):
        function_id_(OsFn::os_nanosleep_fn),
        ret_code_(-5),
        arg_clk_id_(CLOCK_REALTIME_COARSE),
        timespec_(tp),
        os_clock_gettime_callback_(nullptr)
    {}

    explicit Expectation(OsFn fn_):
        function_id_(fn_),
        ret_code_(-5),
        arg_clk_id_(CLOCK_REALTIME_COARSE),
        timespec_({0}),
        os_clock_gettime_callback_(nullptr)
    {}

    Expectation(Expectation &&) = default;
};

MockOs::MockOs()
{
    expectations_ = new MockExpectations();
}

MockOs::~MockOs()
{
    delete expectations_;
}

void MockOs::init()
{
    cppcut_assert_not_null(expectations_);
    expectations_->init();
}

void MockOs::check() const
{
    cppcut_assert_not_null(expectations_);
    expectations_->check();
}

void MockOs::expect_os_clock_gettime(int ret, clockid_t clk_id,
                                     const struct timespec &ret_tp)
{
    expectations_->add(Expectation(ret, clk_id, ret_tp));
}

void MockOs::expect_os_clock_gettime_callback(os_clock_gettime_callback_t fn)
{
    expectations_->add(Expectation(fn));
}

void MockOs::expect_os_nanosleep(const struct timespec *tp)
{
    expectations_->add(Expectation(*tp));
}

void MockOs::expect_os_nanosleep(long milliseconds)
{
    const struct timespec tp =
    {
        .tv_sec = milliseconds / 1000L,
        .tv_nsec = 1000L * 1000L * (milliseconds % 1000L),
    };

    expectations_->add(Expectation(tp));
}

void MockOs::expect_os_abort(void)
{
    expectations_->add(Expectation(OsFn::stdlib_abort));
}


MockOs *mock_os_singleton = nullptr;

int os_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    const auto &expect(mock_os_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, OsFn::os_clock_gettime_fn);

    if(expect.os_clock_gettime_callback_ != nullptr)
        return expect.os_clock_gettime_callback_(clk_id, tp);

    cppcut_assert_equal(expect.arg_clk_id_, clk_id);
    *tp = expect.timespec_;
    return expect.ret_code_;
}

void os_nanosleep(const struct timespec *tp)
{
    const auto &expect(mock_os_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, OsFn::os_nanosleep_fn);

    cppcut_assert_equal(expect.timespec_.tv_sec, tp->tv_sec);
    cppcut_assert_equal(expect.timespec_.tv_nsec, tp->tv_nsec);
}

void os_abort(void)
{
    const auto &expect(mock_os_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_equal(expect.function_id_, OsFn::stdlib_abort);
}
