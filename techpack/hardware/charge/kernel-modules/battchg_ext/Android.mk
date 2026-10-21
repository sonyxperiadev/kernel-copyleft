LOCAL_PATH := $(call my-dir)

$(shell rm $(LOCAL_PATH)/pmic-voter.c)
$(shell ln -s ../../../../../../kernel/msm-5.4/drivers/power/supply/qcom/pmic-voter.c $(LOCAL_PATH)/pmic-voter.c)

include $(CLEAR_VARS)

DLKM_DIR   := $(TOP)/device/qcom/common/dlkm
LOCAL_MODULE              := somc_battchg_ext.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

