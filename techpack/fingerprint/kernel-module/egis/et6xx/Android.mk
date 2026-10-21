LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

DLKM_DIR   := $(TOP)/device/qcom/common/dlkm
LOCAL_MODULE              := et603-int.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

#include $(CLEAR_VARS)

#DLKM_DIR   := $(TOP)/device/qcom/common/dlkm
#LOCAL_MODULE              := navi_input.ko
#LOCAL_MODULE_TAGS         := optional
#LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
#include $(DLKM_DIR)/Build_external_kernelmodule.mk

