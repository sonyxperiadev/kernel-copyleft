ifeq ($(TARGET_USES_QMAA),true)
	KGSL_ENABLED := false
	ifeq ($(TARGET_USES_QMAA_OVERRIDE_GFX),true)
		KGSL_ENABLED := true
	endif # TARGET_USES_QMAA_OVERRIDE_GFX
else
	KGSL_ENABLED := true
endif # TARGET_USES_QMAA

ifeq ($(KGSL_ENABLED),true)
KGSL_SELECT := CONFIG_QCOM_KGSL=m

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# This makefile is only for DLKM
ifneq ($(findstring vendor,$(LOCAL_PATH)),)

DLKM_DIR   := device/qcom/common/dlkm

KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += $(KGSL_SELECT)
KBUILD_OPTIONS += MODNAME=msm_kgsl

include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE      := msm_kgsl.ko
LOCAL_MODULE_KBUILD_NAME  := msm_kgsl.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

# Include msm_kgsl.ko in the /vendor/lib/modules (vendor.img)
BOARD_VENDOR_KERNEL_MODULES += $(LOCAL_MODULE_PATH)/$(LOCAL_MODULE)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

endif # DLKM check
endif # KGSL_ENABLED
