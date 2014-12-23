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
