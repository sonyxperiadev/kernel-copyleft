DISPLAY_SELECT := CONFIG_DRM_MSM=m

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# This makefile is only for DLKM
ifneq ($(findstring vendor,$(LOCAL_PATH)),)

ifneq ($(findstring opensource,$(LOCAL_PATH)),)
	DISPLAY_BLD_DIR := $(TOP)/vendor/qcom/opensource/display-drivers
endif # opensource

DLKM_DIR := $(TOP)/device/qcom/common/dlkm

LOCAL_ADDITIONAL_DEPENDENCIES := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)

# Build display.ko as msm_drm.ko
###########################################################
# This is set once per LOCAL_PATH, not per (kernel) module
KBUILD_OPTIONS := DISPLAY_ROOT=$(DISPLAY_BLD_DIR)
KBUILD_OPTIONS += MODNAME=msm_drm
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += $(DISPLAY_SELECT)

ifneq ($(TARGET_BOARD_AUTO),true)
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS+=$(PWD)/$(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
ifneq ($(TARGET_BOARD_PLATFORM), taro)
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS+=$(PWD)/$(call intermediates-dir-for,DLKM,msm-ext-disp-module-symvers)/Module.symvers
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS+=$(PWD)/$(call intermediates-dir-for,DLKM,sec-module-symvers)/Module.symvers
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS+=$(PWD)/$(call intermediates-dir-for,DLKM,hw-fence-module-symvers)/Module.symvers
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS+=$(PWD)/$(call intermediates-dir-for,DLKM,sync-fence-module-symvers)/Module.symvers
endif
endif

###########################################################

include $(CLEAR_VARS)
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := drm-module-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := msm_drm.ko
LOCAL_MODULE_KBUILD_NAME  := msm_drm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)

ifneq ($(TARGET_BOARD_AUTO),true)
LOCAL_REQUIRED_MODULES    += mmrm-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
ifneq ($(TARGET_BOARD_PLATFORM), taro)
	LOCAL_REQUIRED_MODULES    += msm-ext-disp-module-symvers
	LOCAL_REQUIRED_MODULES    += sec-module-symvers
	LOCAL_REQUIRED_MODULES    += hw-fence-module-symvers
	LOCAL_REQUIRED_MODULES    += sync-fence-module-symvers
	LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,msm-ext-disp-module-symvers)/Module.symvers
	LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,sec-module-symvers)/Module.symvers
	LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,hw-fence-module-symvers)/Module.symvers
	LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,sync-fence-module-symvers)/Module.symvers
endif
endif

include $(DLKM_DIR)/Build_external_kernelmodule.mk
###########################################################
endif # DLKM check
