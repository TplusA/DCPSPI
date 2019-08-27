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

#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdlib.h>

#include "mock_gpio.hh"

enum class GpioFn
{
    is_active,

    first_valid_gpio_fn_id = is_active,
    last_valid_gpio_fn_id = is_active,
};

static std::ostream &operator<<(std::ostream &os, const GpioFn id)
{
    if(id < GpioFn::first_valid_gpio_fn_id ||
       id > GpioFn::last_valid_gpio_fn_id)
    {
        os << "INVALID";
        return os;
    }

    switch(id)
    {
      case GpioFn::is_active:
        os << "gpio_is_active";
        break;
    }

    os << "()";

    return os;
}

class MockGPIO::Expectation
{
  public:
    struct Data
    {
        const GpioFn function_id_;

        bool ret_bool_;
        const struct gpio_handle *arg_gpio_;

        explicit Data(GpioFn fn):
            function_id_(fn),
            ret_bool_(false),
            arg_gpio_(nullptr)
        {}
    };

    const Data d;

  private:
    /* writable reference for simple ctor code */
    Data &data_ = *const_cast<Data *>(&d);

  public:
    Expectation(const Expectation &) = delete;
    Expectation &operator=(const Expectation &) = delete;

    explicit Expectation(GpioFn fn, bool ret, const struct gpio_handle *gpio):
        d(fn)
    {
        data_.ret_bool_ = ret;
        data_.arg_gpio_ = gpio;
    }

    Expectation(Expectation &&) = default;
};

MockGPIO::MockGPIO()
{
    expectations_ = new MockExpectations();
}

MockGPIO::~MockGPIO()
{
    delete expectations_;
}

void MockGPIO::init()
{
    cppcut_assert_not_null(expectations_);
    expectations_->init();
}

void MockGPIO::check() const
{
    cppcut_assert_not_null(expectations_);
    expectations_->check();
}


struct gpio_handle
{
    int fd;
};

struct gpio_handle *MockGPIO::get_handle(int expected_fd)
{
    struct gpio_handle *gpio = new gpio_handle;
    cppcut_assert_not_null(gpio);

    gpio->fd = expected_fd;

    return gpio;
}

void MockGPIO::close_handle(struct gpio_handle *&gpio)
{
    if(gpio != nullptr)
    {
        delete gpio;
        gpio = nullptr;
    }
}

void MockGPIO::expect_gpio_is_active(bool ret, const struct gpio_handle *gpio)
{
    expectations_->add(Expectation(GpioFn::is_active, ret, gpio));
}


MockGPIO *mock_gpio_singleton = nullptr;

bool gpio_is_active(const struct gpio_handle *gpio)
{
    const auto &expect(mock_gpio_singleton->expectations_->get_next_expectation(__func__));

    cppcut_assert_not_null(gpio);
    cppcut_assert_equal(expect.d.function_id_, GpioFn::is_active);

    return expect.d.ret_bool_;
}
