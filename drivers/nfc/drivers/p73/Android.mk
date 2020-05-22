LOCAL_PATH := $(call my-dir)
DLKM_DIR := $(TOP)/device/qcom/common/dlkm

include $(CLEAR_VARS)
LOCAL_MODULE              := p73.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)

include $(DLKM_DIR)/AndroidKernelModule.mk
