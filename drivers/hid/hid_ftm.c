#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kobject.h>

#define DEBUG 0


int hid_state = 0;

struct platform_device hid_attr = {
	.name	       = "hid_attr",
	.id            = -1,
};
static ssize_t hid_show_state(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, 10, "%d\n", hid_state);
}

static DRIVER_ATTR(state, 0644, hid_show_state, NULL);

static struct driver_attribute *hid_attr_list[] = {
	&driver_attr_state,
};

static struct platform_driver hid_attr_driver;

static int hid_attr_probe(struct platform_device *pdev)
{
	int rt;

	rt = driver_create_file(&hid_attr_driver.driver, hid_attr_list[0]);
	if (rt) {
		pr_err("hid_attr_probe: driver_create_file[0] fail!!!\n");
		return rt;
	}

	return 0;
}

static struct platform_driver hid_attr_driver = {
	.probe      = hid_attr_probe,
	.driver = {
		.name   = "hid_attr",
		.owner  = THIS_MODULE,
	},
};

static int __init hid_init(void)
{
	int rc;

	rc = platform_device_register(&hid_attr);

	if (rc != 0)
		return rc;

	rc = platform_driver_register(&hid_attr_driver);
	if (rc < 0)
		pr_err("****register hid_attr_driver fail!\n");

	return rc;
}

static void __exit hid_exit(void)
{
	platform_driver_unregister(&hid_attr_driver);
}

module_init(hid_init);
module_exit(hid_exit);

MODULE_DESCRIPTION("hid ftm driver");
MODULE_LICENSE("GPL");
