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

#ifndef GPIO_H
#define GPIO_H

struct gpio_handle;

#ifdef __cplusplus
extern "C" {
#endif

struct gpio_handle *gpio_open(unsigned int gpio_num, bool is_active_low);
void gpio_close(struct gpio_handle *gpio);
int gpio_get_poll_fd(const struct gpio_handle *gpio);
bool gpio_is_active(const struct gpio_handle *gpio);
void gpio_enable_debouncing(struct gpio_handle *gpio);

#ifdef __cplusplus
}
#endif

#endif /* !GPIO_H */
