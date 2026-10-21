ccflags-y += -I$(BT_ROOT)/include
ccflags-y += -I$(BT_ROOT)/../wlan/platform/inc
ccflags-y += -I$(BT_ROOT)/../wlan/platform/cnss_utils

ifeq ($(CONFIG_BT_HW_SECURE_DISABLE), y)
ccflags-y += -I$(BT_ROOT)/../securemsm-kernel/smcinvoke/
ccflags-y += -I$(BT_ROOT)/../securemsm-kernel/linux/
endif

obj-$(CONFIG_MSM_BT_POWER)  += btpower.o
