#
# Makefile for RMI4 bus.
#

# Each configuration option enables a list of files.


obj-$(CONFIG_RMI4_BUS)			+= rmi4_bus.o
obj-$(CONFIG_RMI4_DRIVER)		+= rmi4_driver.o
obj-$(CONFIG_RMI4_SPI_ADAPTOR)		+= rmi4_spi_adaptor.o
obj-$(CONFIG_RMI4_I2C_ADAPTOR)		+= rmi4_i2c_adaptor.o
obj-$(CONFIG_RMI4_F01)			+= rmi4_function_01.o
obj-$(CONFIG_RMI4_F11)			+= rmi4_function_11.o
obj-$(CONFIG_RMI4_F34)			+= rmi4_function_34.o
obj-$(CONFIG_RMI4_F54)			+= rmi4_function_54.o
obj-$(CONFIG_RMI4_CHAR_DEV)		+= rmi4_char_dev.o
obj-$(CONFIG_RMI4_EARLY_SUSPEND)	+= rmi4_early_suspend.o
obj-$(CONFIG_RMI4_WATCHDOG)		+= rmi4_watchdog.o

ifeq ($(CONFIG_RMI4_DEBUG_BUS),y)
EXTRA_CFLAGS += -DRMI4_CORE_DEBUG
endif
