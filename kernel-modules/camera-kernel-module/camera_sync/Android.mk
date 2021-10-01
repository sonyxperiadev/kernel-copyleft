LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_SRC_FILES         := camera_sync.c
LOCAL_C_INCLUDES        := camera_sync.h
LOCAL_MODULE            := camera_sync.ko
LOCAL_MODULE_TAGS       := optional
LOCAL_MODULE_PATH       := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
