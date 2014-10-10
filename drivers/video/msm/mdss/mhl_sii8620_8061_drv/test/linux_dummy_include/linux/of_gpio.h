#ifndef _LINUX_OF_GPIO_H
#define _LINUX_OF_GPIO_H

int of_get_named_gpio(struct device_node *np, const char *propname, int index);

#endif /* _LINUX_OF_GPIO_H */
