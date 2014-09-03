#define _XOPEN_SOURCE 500

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>

#include "gpio.h"
#include "messages.h"

struct gpio_handle
{
    bool is_in_use;
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

    while((temp = close(fd)) == -1 && errno == EINTR)
        ;

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
                         unsigned int sleep_ms_between_tries,
                         bool show_error)
{
    while(1)
    {
        if(access(path, W_OK) == 0)
            return 0;

        if(--tries <= 0)
        {
            if(show_error)
                msg_error(errno, LOG_EMERG, "Path \"%s\" does not exist", path);

            return -1;
        }

        usleep(1000U * sleep_ms_between_tries);
    }
}

static int hook_to_gpio(unsigned int num)
{
    char export_dir[32];
    snprintf(export_dir, sizeof(export_dir), "/sys/class/gpio/gpio%u", num);

    char buffer[10];
    size_t buffer_length;

    if(wait_for_path(export_dir, 1, 0, false) < 0)
    {
        buffer_length = snprintf(buffer, sizeof(buffer), "%u", num);

        if(write_to_gpio_file("/sys/class/gpio/export", buffer, buffer_length) < 0)
            return -1;
    }

    const char *filename;

    filename = mk_gpio_name(export_dir, "direction");
    if(wait_for_path(filename, 50, 10, true) < 0)
        return -1;

    buffer_length = snprintf(buffer, sizeof(buffer), "%s", "in");
    if(write_to_gpio_file(filename, buffer, buffer_length) < 0)
        return -1;

    filename = mk_gpio_name(export_dir, "edge");
    if(wait_for_path(filename, 50, 10, true) < 0)
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
    ssize_t bytes;
    while((bytes = read(fd, buffer, sizeof(buffer))) == -1 && errno == EINTR)
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

struct gpio_handle *gpio_open(unsigned int gpio_num)
{
    if(the_gpio.is_in_use)
        return NULL;

    the_gpio.value_fd = hook_to_gpio(gpio_num);
    if(the_gpio.value_fd < 0)
        return NULL;

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

    return buffer[0] == '0';
}

bool gpio_is_active(const struct gpio_handle *gpio)
{
    static const int debounce_count = 10;

    int debounce = debounce_count;
    bool value = sample_gpio_value(gpio);

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

    return value;
}
