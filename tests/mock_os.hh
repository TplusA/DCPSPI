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

#ifndef MOCK_OS_HH
#define MOCK_OS_HH

#include "os.h"
#include "mock_expectation.hh"

class MockOs
{
  public:
    MockOs(const MockOs &) = delete;
    MockOs &operator=(const MockOs &) = delete;

    class Expectation;
    typedef MockExpectationsTemplate<Expectation> MockExpectations;
    MockExpectations *expectations_;

    explicit MockOs();
    ~MockOs();

    void init();
    void check() const;

    typedef int (*os_clock_gettime_callback_t)(clockid_t clk_id, struct timespec *tp);
    void expect_os_clock_gettime(int ret, clockid_t clk_id, const struct timespec &ret_tp);
    void expect_os_clock_gettime_callback(os_clock_gettime_callback_t fn);

    void expect_os_nanosleep(const struct timespec *tp);
    void expect_os_nanosleep(long milliseconds);

    void expect_os_abort(void);
};

extern MockOs *mock_os_singleton;

#endif /* !MOCK_OS_HH */
