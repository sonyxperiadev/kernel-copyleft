LOCAL_PATH := $(call my-dir)
DLKM_DIR := $(TOP)/device/qcom/common/dlkm

include $(CLEAR_VARS)
LOCAL_SRC_FILES           += main_module.c bd7602/bd7602.c cxd224x/cxd224x-i2c.c
LOCAL_MODULE              := sony_carillon_nfc.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)

include $(DLKM_DIR)/AndroidKernelModule.mk
