/*
 * Copyright (C) 2015, 2019, 2022  T+A elektroakustik GmbH & Co. KG
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>

#include "gpio.h"
#include "os.h"
#include "messages.h"

struct gpio_handle
{
    bool is_in_use;
    bool is_active_low;
    bool is_debouncing_enabled;
    unsigned int gpio_num;
    int value_fd;
};

static struct gpio_handle the_gpio;

static int write_to_gpio_file(const char *filename, const char *buffer,
                              size_t buffer_length)
{
    int fd;

    while((fd = open(filename, O_WRONLY)) == -1 && errno == EINTR)
        ;

    if(fd < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed opening GPIO file \"%s\"", filename);
        return -1;
    }

    int temp;
    int ret = 0;

    while((temp = write(fd, buffer, buffer_length)) == -1 && errno == EINTR)
        ;

    if(temp < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed writing GPIO file \"%s\"", filename);
        ret = -1;
    }

    os_file_close(fd);

    if(temp < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed closing GPIO file \"%s\"", filename);
        ret = -1;
    }

    return ret;
}

static const char *mk_gpio_name(const char *prefix, const char *filename)
{
    static char buffer[64];

    snprintf(buffer, sizeof(buffer), "%s/%s", prefix, filename);
    return buffer;
}

static int wait_for_path(const char *path, int tries,
                         unsigned int min_sleep_ms_between_tries,
                         unsigned int max_sleep_ms_between_tries,
                         bool show_error)
{
    int tries_left = tries;
    unsigned int sleep_ms_between_tries = 0;
    struct timespec tp = { 0 };

    while(1)
    {
        if(access(path, W_OK) == 0)
        {
            const int tried = tries - tries_left + 1;
            msg_vinfo(MESSAGE_LEVEL_DIAG,
                      "Path \"%s\" accessible after %d %s",
                      path, tried, tried == 1 ? "try" : "tries");

            return 0;
        }

        if(--tries_left <= 0)
        {
            if(show_error)
                msg_error(errno, LOG_EMERG,
                          "Path \"%s\" not accessible after %d tries",
                          path, tries);

            return -1;
        }

        if(sleep_ms_between_tries < max_sleep_ms_between_tries)
        {
            if(sleep_ms_between_tries < min_sleep_ms_between_tries)
                sleep_ms_between_tries = min_sleep_ms_between_tries;
            else
                sleep_ms_between_tries *= 2;

            if(sleep_ms_between_tries > max_sleep_ms_between_tries)
                sleep_ms_between_tries = max_sleep_ms_between_tries;

            tp.tv_sec = sleep_ms_between_tries / 1000U;
            tp.tv_nsec = (sleep_ms_between_tries % 1000) * 1000UL * 1000UL;
        }

        os_nanosleep(&tp);
    }
}

static int hook_to_gpio(unsigned int num)
{
    static const char sys_class_gpio[] = "/sys/class/gpio";

    char export_dir[32];
    snprintf(export_dir, sizeof(export_dir), "%s/gpio%u", sys_class_gpio, num);

    char buffer[10];
    size_t buffer_length;
    const char *filename;

    if(wait_for_path(export_dir, 1, 0, 0, false) < 0)
    {
        buffer_length = snprintf(buffer, sizeof(buffer), "%u", num);

        filename = mk_gpio_name(sys_class_gpio, "export");
        if(write_to_gpio_file(filename, buffer, buffer_length) < 0)
            return -1;

        os_sync_dir(sys_class_gpio);
    }

    filename = mk_gpio_name(export_dir, "direction");
    if(wait_for_path(filename, 1000, 10, 200, true) < 0)
        return -1;

    buffer_length = snprintf(buffer, sizeof(buffer), "%s", "in");
    if(write_to_gpio_file(filename, buffer, buffer_length) < 0)
        return -1;

    filename = mk_gpio_name(export_dir, "edge");
    if(wait_for_path(filename, 1000, 10, 200, true) < 0)
        return -1;

    buffer_length = snprintf(buffer, sizeof(buffer), "%s", "both");
    if(write_to_gpio_file(filename, buffer, buffer_length) < 0)
        return -1;

    filename = mk_gpio_name(export_dir, "value");

    int fd;

    while((fd = open(filename, O_RDONLY)) == -1 && errno == EINTR)
        ;

    if(fd < 0)
    {
        msg_error(errno, LOG_EMERG,
                  "Failed opening GPIO value file \"%s\"", filename);
        return -1;
    }

    /* dummy read to flush the current value */
    while(read(fd, buffer, sizeof(buffer)) == -1 && errno == EINTR)
        ;
    lseek(fd, 0, SEEK_SET);

    return fd;
}

static void unhook_from_gpio(struct gpio_handle *gpio)
{
    int ret;

    while((ret = close(gpio->value_fd)) < 0 && errno == EINTR)
        ;

    if(ret < 0)
        msg_error(errno, LOG_ERR,
                  "Failed closing GPIO value fd %d", gpio->value_fd);

    gpio->value_fd = -1;

    char buffer[10];
    size_t buffer_length =
        snprintf(buffer, sizeof(buffer), "%u", gpio->gpio_num);

    (void)write_to_gpio_file("/sys/class/gpio/unexport", buffer, buffer_length);
}

struct gpio_handle *gpio_open(unsigned int gpio_num, bool is_active_low)
{
    if(the_gpio.is_in_use)
        return NULL;

    the_gpio.value_fd = hook_to_gpio(gpio_num);
    if(the_gpio.value_fd < 0)
        return NULL;

    the_gpio.is_active_low = is_active_low;
    the_gpio.is_debouncing_enabled = false;
    the_gpio.gpio_num = gpio_num;
    the_gpio.is_in_use = true;

    return &the_gpio;
}

void gpio_close(struct gpio_handle *gpio)
{
    if(!gpio->is_in_use)
        return;

    unhook_from_gpio(gpio);

    gpio->is_in_use = false;
}

int gpio_get_poll_fd(const struct gpio_handle *gpio)
{
    return gpio->value_fd;
}

static bool sample_gpio_value(const struct gpio_handle *gpio)
{
    ssize_t bytes;
    char buffer[10];

    while((bytes = read(gpio->value_fd, buffer, sizeof(buffer))) == -1 && errno == EINTR)
        ;
    if(lseek(gpio->value_fd, 0, SEEK_SET) == (off_t)-1)
        msg_error(errno, LOG_EMERG,
                  "lseek() failed for GPIO value fd %d", gpio->value_fd);

    if(bytes != 2 || (buffer[0] != '0' && buffer[0] != '1'))
    {
        msg_error(0, LOG_EMERG,
                  "Got unexpected input from GPIO value fd %d", gpio->value_fd);
        return false;
    }

    return buffer[0] == '1';
}

bool gpio_is_active(const struct gpio_handle *gpio)
{
    bool value = sample_gpio_value(gpio);

    if(gpio->is_debouncing_enabled)
    {
        static const int debounce_count = 10;

        int debounce = debounce_count;

        while(--debounce > 0)
        {
            usleep(5000);

            bool temp = sample_gpio_value(gpio);
            if(temp != value)
            {
                value = temp;
                debounce = debounce_count;
            }
        }
    }

    return gpio->is_active_low ? !value : value;
}

void gpio_enable_debouncing(struct gpio_handle *gpio)
{
    gpio->is_debouncing_enabled = true;
}
