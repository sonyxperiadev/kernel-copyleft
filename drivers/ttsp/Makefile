obj-$(CONFIG_CYPRESS_CYTTSP4_BUS)	+= cyttsp4_bus.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4)	+= cyttsp4_core.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_I2C)	+= cyttsp4_i2c.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_SPI)	+= cyttsp4_spi.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_MT_A)	+= cyttsp4_mt_a.o
cyttsp4_mt_a-y := cyttsp4_mta.o cyttsp4_mt_common.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_MT_B)	+= cyttsp4_mt_b.o
cyttsp4_mt_b-y := cyttsp4_mtb.o cyttsp4_mt_common.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_BUTTON)	+= cyttsp4_btn.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICE_ACCESS)	+= cyttsp4_device_access.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_LOADER)	+= cyttsp4_loader.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEBUG_MODULE)	+= cyttsp4_debug.o

ifeq ($(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEBUG),y)
CFLAGS_cyttsp4_bus.o += -DDEBUG
CFLAGS_cyttsp4_core.o += -DDEBUG
CFLAGS_cyttsp4_i2c.o += -DDEBUG
CFLAGS_cyttsp4_spi.o += -DDEBUG
CFLAGS_cyttsp4_mta.o += -DDEBUG
CFLAGS_cyttsp4_mtb.o += -DDEBUG
CFLAGS_cyttsp4_mt_a.o += -DDEBUG
CFLAGS_cyttsp4_mt_b.o += -DDEBUG
CFLAGS_cyttsp4_mt_common.o += -DDEBUG
CFLAGS_cyttsp4_btn.o += -DDEBUG
CFLAGS_cyttsp4_device_access.o += -DDEBUG
CFLAGS_cyttsp4_loader.o += -DDEBUG
CFLAGS_cyttsp4_debug.o += -DDEBUG
endif

ifeq ($(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_VDEBUG),y)
CFLAGS_cyttsp4_bus.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_core.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_i2c.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_spi.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_mta.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_mtb.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_mt_a.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_mt_b.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_mt_common.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_btn.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_device_access.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_loader.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp4_debug.o += -DVERBOSE_DEBUG
endif
