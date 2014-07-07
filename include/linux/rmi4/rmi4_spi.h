#ifndef __RMI4_SPI__
#define __RMI4_SPI__

struct rmi4_core_device_data;

#define RMI4_SPI_ADAPTER_NAME	"rmi4-spi-adapter"

struct rmi4_spi_v2_platform_data {
	int block_delay_us;
	int split_read_block_delay_us;
	int read_delay_us;
	int write_delay_us;
	int split_read_byte_delay_us;
	int pre_delay_us;
	int post_delay_us;

	void *cs_assert_data;
	int (*cs_assert) (struct device *dev, bool assert);
};

struct rmi4_spi_adapter_platform_data {
	char *driver_name;

	int attn_gpio;
	int irq_polarity;
	bool irq_is_shared;
	bool assert_level_low;

	int (*gpio_config)(struct device *dev, bool enable);

	struct rmi4_spi_v2_platform_data spi_v2;

	struct rmi4_core_device_data *cdev_data;
};

#endif /* __RMI4_SPI__ */
