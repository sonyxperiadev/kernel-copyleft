#ifndef __LINUX_GPIO_H
#define __LINUX_GPIO_H

void gpio_set_value(unsigned gpio, int value);
int gpio_get_value(unsigned gpio);

#endif /* __LINUX_GPIO_H */
