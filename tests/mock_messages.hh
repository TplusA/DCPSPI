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

#ifndef MOCK_MESSAGES_HH
#define MOCK_MESSAGES_HH

#include "messages.h"
#include "mock_expectation.hh"

class MockMessages
{
  private:
    MockMessages(const MockMessages &);
    MockMessages &operator=(const MockMessages &);

  public:
    class Expectation;
    typedef MockExpectationsTemplate<Expectation> MockExpectations;
    MockExpectations *expectations_;

    bool ignore_all_;

    explicit MockMessages();
    ~MockMessages();

    void init();
    void check() const;

    void expect_msg_error_formatted(int error_code, int priority, const char *string);
    void expect_msg_error_formatted(int error_code, int priority, const char *prefix, const char *suffix);
    void expect_msg_error(int error_code, int priority, const char *string);
    void expect_msg_info_formatted(const char *string);
    void expect_msg_info(const char *string);
};

/*!
 * One messages mock to rule them all...
 *
 * This is necessary because there are only free C function tested by this
 * mock, and there is no simple way for these functions to pick a suitable mock
 * object from a set of those. It should be possible to use TLD for this
 * purpose, but for now let's go with a simpler version.
 *
 * \note Having this singleton around means that running tests in multiple
 *     threads in NOT possible.
 */
extern MockMessages *mock_messages_singleton;

#endif /* !MOCK_MESSAGES_HH */
