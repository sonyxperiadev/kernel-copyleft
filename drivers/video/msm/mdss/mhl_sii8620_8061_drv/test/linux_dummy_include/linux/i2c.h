#ifndef __LINUX_I2C_H
#define __LINUX_I2C_H

#include "linux/platform_device.h"
#include "linux/types.h"

#undef I2C_DEVICE_ID_DEF

struct i2c_msg {
	unsigned short addr;
	unsigned short flags;
	unsigned short len;
	unsigned char *buf;
#define I2C_M_RD 0x01
};

#define module_i2c_driver(_x) char _x##2


struct i2c_device_id {
	char name[20];
	unsigned short id;
};

struct i2c_client {
	bool dummy;
	struct device dev;
	struct i2c_adapter *adapter;
};


struct i2c_driver {
	struct device_driver driver;
	int (*probe)(struct i2c_client *, const struct i2c_device_id *);
	int (*remove)(struct i2c_client *);
	struct i2c_device_id *id_table;
};

struct i2c_adapter {
	bool dummy;
};

#endif /* __LINUX_I2C_H  */
