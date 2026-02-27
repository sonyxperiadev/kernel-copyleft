#
# Kernel module make instructions go here.
#
obj-$(CONFIG_NFC_SN1X0_DEVICES) += sn1x0.o
sn1x0-objs := common.o common_ese.o i2c_drv.o
ccflags-$(CONFIG_NFC_ENABLE_G7_COLD_RESET) := -DHW_COLD_RESET
#ccflags-y := -DDEBUG
