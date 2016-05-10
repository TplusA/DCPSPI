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

#ifndef MOCK_GPIO_HH
#define MOCK_GPIO_HH

#include "gpio.h"
#include "mock_expectation.hh"

struct gpio_handle;

class MockGPIO
{
  public:
    MockGPIO(const MockGPIO &) = delete;
    MockGPIO &operator=(const MockGPIO &) = delete;

    class Expectation;
    typedef MockExpectationsTemplate<Expectation> MockExpectations;
    MockExpectations *expectations_;

    explicit MockGPIO();
    ~MockGPIO();

    void init();
    void check() const;

    static struct gpio_handle *get_handle(int expected_fd);
    static void close_handle(struct gpio_handle *&gpio);

    void expect_gpio_is_active(bool ret, const struct gpio_handle *gpio);
};

extern MockGPIO *mock_gpio_singleton;

#endif /* !MOCK_GPIO_HH */
