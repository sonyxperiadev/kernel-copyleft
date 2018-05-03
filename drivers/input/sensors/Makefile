#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

#EXTRA_CFLAGS += -DBMM_SMBUS

obj-$(CONFIG_SENSORS_BMA2X2)    += bma2x2_driver.o
obj-$(CONFIG_SENSORS_BMM050)    += bmm050_driver.o bmm050.o
obj-$(CONFIG_SENSORS_BMG160)    += bmg160_driver.o bmg160.o
obj-$(CONFIG_SENSORS_ALS_PRX_STK3X1X)	+= stk3x1x.o
