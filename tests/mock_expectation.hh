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

#ifndef MOCK_EXPECTATION_HH
#define MOCK_EXPECTATION_HH

#include <cppcutter.h>
#include <vector>

template <typename E>
class MockExpectationsTemplate
{
  private:
    MockExpectationsTemplate(const MockExpectationsTemplate &);
    MockExpectationsTemplate &operator=(const MockExpectationsTemplate &);

    std::vector<E> expectations_;
    size_t next_checked_expectation_;

  public:
    explicit MockExpectationsTemplate() {}

    void init()
    {
        expectations_.clear();
        next_checked_expectation_ = 0;
    }

    void check() const
    {
        cppcut_assert_equal(next_checked_expectation_, expectations_.size(),
                            cut_message("In %s:\nHave %zu expectation%s, but only %zu %s checked",
                                        __PRETTY_FUNCTION__,
                                        expectations_.size(),
                                        (expectations_.size() == 1) ? "" : "s",
                                        next_checked_expectation_,
                                        (next_checked_expectation_ == 1) ? "was" : "were"));
    }

    void add(E &&expectation)
    {
        expectations_.push_back(std::move(expectation));
    }

    const E &get_next_expectation(const char *string)
    {
        cppcut_assert_operator(next_checked_expectation_, <, expectations_.size(),
                               cut_message("Missing expectation for \"%s\"",
                                           string));

        return expectations_[next_checked_expectation_++];
    }
};


#endif /* !MOCK_EXPECTATION_HH */
