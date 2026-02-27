FASTRPC_DLKM_ENABLED := true

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_FASTRPC_OVERRIDE), false)
		FASTRPC_DLKM_ENABLED := false
	endif
endif

ifeq ($(FASTRPC_DLKM_ENABLED), true)
DLKM_DIR   := device/qcom/common/dlkm

LOCAL_PATH := $(call my-dir)

DSP_BLD_DIR := $(abspath .)/vendor/qcom/opensource/dsp-kernel

LOCAL_MODULE_DDK_BUILD := true

include $(CLEAR_VARS)
$(info DLKM_DIR = $(DLKM_DIR))
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE      := dsp-module-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
$(info DLKM_DIR = $(DLKM_DIR))
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE      := frpc-adsprpc.ko
LOCAL_EXPORT_KO_INCLUDE_DIRS    := $(LOCAL_PATH)/include/linux
LOCAL_MODULE_KBUILD_NAME := frpc-adsprpc.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
KBUILD_OPTIONS += DSP_ROOT=$(DSP_BLD_DIR)
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
$(info DLKM_DIR = $(DLKM_DIR))
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE      := cdsp-loader.ko
LOCAL_MODULE_KBUILD_NAME := cdsp-loader.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
KBUILD_OPTIONS += DSP_ROOT=$(DSP_BLD_DIR)
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

# print out variables
$(info KBUILD_OPTIONS = $(KBUILD_OPTIONS))
$(info intermediates dsp symvers path = $(call intermediates-dir-for,DLKM,dsp-module-symvers))
$(info DLKM_DIR = $(DLKM_DIR))

endif