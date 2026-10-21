LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/*) $(wildcard $(LOCAL_PATH)/**/*)
LOCAL_MODULE := cirrus_wm_adsp.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(TOP)/device/qcom/common/dlkm/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/*) $(wildcard $(LOCAL_PATH)/**/*)
ifeq ($(CIRRUS_AMP_CODEC),cs35l45_spi)
KERNEL_CFLAGS += CONFIG_SND_SOC_CS35L45_SPI=y
else
KERNEL_CFLAGS += CONFIG_SND_SOC_CS35L45_I2C=y
endif
LOCAL_MODULE := cirrus_cs35l45.ko

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(TOP)/device/qcom/common/dlkm/Build_external_kernelmodule.mk

