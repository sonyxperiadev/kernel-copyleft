TARGET_SYNX_ENABLE := false
ifeq ($(TARGET_KERNEL_DLKM_DISABLE),true)
	ifeq ($(TARGET_KERNEL_DLKM_SYNX_OVERRIDE),true)
		TARGET_SYNX_ENABLE := true
	endif
else
TARGET_SYNX_ENABLE := true
endif

ifeq ($(TARGET_SYNX_ENABLE),true)
SYNX_BLD_DIR := $(TOP)/vendor/qcom/opensource/synx-kernel


# Build synx-driver.ko
###########################################################
# This is set once per LOCAL_PATH, not per (kernel) module
KBUILD_OPTIONS := SYNX_ROOT=$(SYNX_BLD_DIR)
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS+=$(PWD)/$(call intermediates-dir-for,DLKM,hw-fence-module-symvers)/Module.symvers
###########################################################

DLKM_DIR   := $(TOP)/device/qcom/common/dlkm

LOCAL_PATH := $(call my-dir)
LOCAL_MODULE_DDK_BUILD := true
LOCAL_MODULE_KO_DIRS := msm/synx/synx-driver.ko msm/synx/ipclite.ko msm/synx/test/ipclite_test.ko

include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES           := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := synx-driver-symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
#LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
# Include kp_module.ko in the /vendor/lib/modules (vendor.img)
# BOARD_VENDOR_KERNEL_MODULES += $(LOCAL_MODULE_PATH)/$(LOCAL_MODULE)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
$(info LOCAL_SRC_FILES = $(LOCAL_SRC_FILES))
LOCAL_MODULE      := synx-driver.ko
LOCAL_MODULE_KBUILD_NAME := msm/synx/synx-driver.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_REQUIRED_MODULES    := hw-fence-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,hw-fence-module-symvers)/Module.symvers
include $(DLKM_DIR)/Build_external_kernelmodule.mk


include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
$(info LOCAL_SRC_FILES = $(LOCAL_SRC_FILES))
LOCAL_MODULE      := ipclite.ko
LOCAL_MODULE_KBUILD_NAME := msm/synx/ipclite.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
#BOARD_VENDOR_KERNEL_MODULES += $(LOCAL_MODULE_PATH)/$(LOCAL_MODULE)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
$(info LOCAL_SRC_FILES = $(LOCAL_SRC_FILES))
LOCAL_MODULE      := ipclite_test.ko
LOCAL_MODULE_KBUILD_NAME := msm/synx/test/ipclite_test.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
#BOARD_VENDOR_KERNEL_MODULES += $(LOCAL_MODULE_PATH)/$(LOCAL_MODULE)

# print out variables
$(info KBUILD_OPTIONS = $(KBUILD_OPTIONS))
$(info LOCAL_ADDITIONAL_DEPENDENCY = $(LOCAL_ADDITIONAL_DEPENDENCY))
$(info LOCAL_ADDITIONAL_DEPENDENCIES = $(LOCAL_ADDITIONAL_DEPENDENCIES))
$(info LOCAL_REQUIRED_MODULES = $(LOCAL_REQUIRED_MODULES))
$(info DLKM_DIR = $(DLKM_DIR))
include $(DLKM_DIR)/Build_external_kernelmodule.mk


endif # End of check for TARGET_SYNX_ENABLE
