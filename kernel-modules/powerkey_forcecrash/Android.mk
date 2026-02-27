LOCAL_PATH := $(call my-dir)
KBUILD_OPTIONS :=
include $(CLEAR_VARS)
LOCAL_MODULE              := powerkey_forcecrash.ko
LOCAL_MODULE_KBUILD_NAME  := powerkey_forcecrash.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
