#Build rules to create ST54SPI_GPIO_PLATFORM_DRIVER

ifeq ($(strip $(TARGET_USES_ST_ESE)),true)
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS := -Wno-macro-redefined -Wno-unused-function -Wall -Werror
LOCAL_CLANG :=true

LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_MODULE := stm_st54se_gpio.ko
LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)

DLKM_DIR := $(TOP)/device/qcom/common/dlkm
LOCAL_MODULE_DDK_BUILD := true

include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #end of check for TARGET_USES_ST_ESE flag
