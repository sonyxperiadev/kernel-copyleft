#ifndef __DUMMY_UTIL_H
#define __DUMMY_UTIL_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/clk.h>

/* #define pr_debug printf */
#define pr_debug
#define pr_info printf
#define pr_warn printf
#define pr_err printf
#define printk printf

#define EXPORT_SYMBOL(x) struct dummy_##x {int dummy;}

void kfree(const void *objp);



#define __init
#define __exit



typedef unsigned int gfp_t;

#define GFP_KERNEL (gfp_t)1

void *kzalloc(size_t size, gfp_t flags);

#define devm_kzalloc(_dev,_size,_gfp) malloc(_size)
#define devm_kfree(_dev, _p) pr_info("test")

#define module_init(x) struct dummy_##x {int dummy;}
#define module_exit(x) struct dummy_##x {int dummy;}
#define MODULE_LICENSE(x) static char dummy[] = x

/*
bool gpio_is_valid(int number);
int gpio_request(unsigned int gpio, const char *tag);
int gpio_direction_output(unsigned gpio, int level);
int gpio_direction_input(unsigned gpio);
void gpio_free(unsigned gpio);
int gpio_to_irq(int gpio);
*/
void msleep(int ms);
/*struct platform_device *of_find_device_by_node(struct device_node *np);*/
/* void mhl_device_release(struct device *dev); */
void mhl_tx_release(void);
/* int gpio_get_value(unsigned int gpio); */
/* void mhl_pf_i2c_init(struct i2c_adapter *adapter); */
/* void i2c_set_clientdata(struct i2c_client *dev, void *data); */
/* void mhl_device_initialize(struct device *dev); */
int mhl_tx_initialize(void);

#define kmalloc(_x, _y) malloc((size_t)_x)

#endif
