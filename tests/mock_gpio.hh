/*
 * Copyright (C) 2016, 2019  T+A elektroakustik GmbH & Co. KG
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
