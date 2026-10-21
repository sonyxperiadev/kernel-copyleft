LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE              := sec_touchscreen.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
