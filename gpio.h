#ifndef GPIO_H
#define GPIO_H

struct gpio_handle;

struct gpio_handle *gpio_open(unsigned int gpio_num);
void gpio_close(struct gpio_handle *gpio);
int gpio_get_poll_fd(const struct gpio_handle *gpio);
bool gpio_is_active(const struct gpio_handle *gpio);

#endif /* !GPIO_H */
