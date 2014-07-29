/*
 * platform_device.h - generic, centralized driver model
 *
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 *
 * This file is released under the GPLv2
 *
 * See Documentation/driver-model/ for more information.
 */

#ifndef _PLATFORM_DEVICE_H_
#define _PLATFORM_DEVICE_H_

#include <stdbool.h>
#include "device.h"

#undef DEVICE_ATTR
#define DEVICE_ATTR(_name,_mode,_show,_store)

struct of_device_id {
	char name[32];
	char type[32];
};

struct device_node {
	struct	device_node *parent;
	bool dummy;
};

struct module {
	bool dummy;
};

struct device_driver {
	const char * name;
	struct module * owner;
	struct of_device_id * table;
};

struct device {
	bool dummy;
	struct device_node *of_node;
	struct device_driver *driver;
};

struct platform_device {
	bool dummy;
	struct device dev;
};



struct device_attribute {
	ssize_t (*show)(struct device *dev, struct device_attribute *attr, char *buf);
	ssize_t (*store)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
};
#endif /* _PLATFORM_DEVICE_H_ */
