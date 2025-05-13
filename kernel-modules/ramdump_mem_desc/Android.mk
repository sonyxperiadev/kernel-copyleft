LOCAL_MODULE_DDK_BUILD := true
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE              := ramdump_mem_desc.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

