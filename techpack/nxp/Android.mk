# Android makefile for nfc kernel modules

# Path to DLKM make scripts
DLKM_DIR          :=  $(TOP)/device/qcom/common/dlkm

LOCAL_PATH        := $(call my-dir)

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(PWD)/$(call intermediates-dir-for,DLKM,sec-module-symvers)/Module.symvers

LOCAL_REQUIRED_MODULES := sec-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,sec-module-symvers)/Module.symvers

include $(CLEAR_VARS)

LOCAL_MODULE      := nxp-nci.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)

NFC_DLKM_ENABLED := false

########## Check and set local DLKM flag based on system-wide global flags ##########
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_NFC_OVERRIDE), true)
    NFC_DLKM_ENABLED := true
  endif
else
  NFC_DLKM_ENABLED := true
endif

########## Build kernel module based on local DLKM flag status ##########
ifeq ($(NFC_DLKM_ENABLED), true)
  include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif
