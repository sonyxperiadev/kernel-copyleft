# Android makefile for SMEM kernel modules
ifeq ($(call is-board-platform-in-list,sun), true)
ifneq (,$(filter arm aarch64 arm64, $(TARGET_ARCH)))

LOCAL_PATH := $(call my-dir)
LOCAL_MODULE_DDK_BUILD := true

DLKM_DIR   := device/qcom/common/dlkm

include $(CLEAR_VARS)
LOCAL_SRC_FILES := $(LOCAL_PATH)/smem-mailbox.c
LOCAL_EXPORT_KO_INCLUDE_DIRS := $(LOCAL_PATH)/include
LOCAL_MODULE := smem-mailbox.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

endif
endif