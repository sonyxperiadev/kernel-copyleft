#ifndef __RMI4_I2C__
#define __RMI4_I2C__

struct rmi4_core_device_data;

#define RMI4_I2C_ADAPTER_NAME	"rmi4_i2c_adaptor"

struct rmi4_i2c_adapter_platform_data {
	char *driver_name;
	int irq;
	int attn_gpio;
	int (*vreg_config)(struct device *dev, int enable);
	struct rmi4_core_device_data *cdev_data;
};

#endif /* __RMI4_I2C__ */
