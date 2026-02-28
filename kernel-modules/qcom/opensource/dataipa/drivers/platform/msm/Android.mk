ifeq ($(TARGET_DATAIPA_DLKM_ENABLE), true)
ifneq ($(TARGET_BOARD_PLATFORM),qssi)

GSI_DLKM_PLATFORMS_LIST := taro kalama bengal monaco pineapple blair holi sun cliffs parrot

#Enabling BAZEL
LOCAL_MODULE_DDK_BUILD := true
LOCAL_MODULE_KO_DIRS := gsi/gsim.ko
LOCAL_MODULE_KO_DIRS += ipa/ipam.ko
LOCAL_MODULE_KO_DIRS += ipa/ipanetm.ko
ifeq ($(CONFIG_LOCALVERSION), "-gki-consolidate")
LOCAL_MODULE_KO_DIRS += ipa/ipatestm.ko
endif

ifeq ($(call is-board-platform-in-list, $(GSI_DLKM_PLATFORMS_LIST)),true)
#Make file to create GSI DLKM

BOARD_COMMON_DIR ?= device/qcom/common
DLKM_DIR := $(TOP)/$(BOARD_COMMON_DIR)/dlkm
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS := -Wno-macro-redefined -Wno-unused-function -Wall -Werror
LOCAL_CLANG :=true


KBUILD_OPTIONS += MODNAME=gsim
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := gsim.ko
LOCAL_MODULE_KBUILD_NAME  := gsi/gsim.ko
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
$(warning $(DLKM_DIR))
include $(DLKM_DIR)/Build_external_kernelmodule.mk


include $(CLEAR_VARS)
KBUILD_OPTIONS += MODNAME=ipam
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := ipam.ko
LOCAL_MODULE_KBUILD_NAME  := ipa/ipam.ko
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_EXPORT_KO_INCLUDE_DIRS    := $(LOCAL_PATH)/include
LOCAL_EXPORT_KO_INCLUDE_DIRS    += $(LOCAL_PATH)/include/uapi
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
$(warning $(DLKM_DIR))
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
KBUILD_OPTIONS += MODNAME=ipanetm
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := ipanetm.ko
LOCAL_MODULE_KBUILD_NAME  := ipa/ipanetm.ko
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
$(warning $(DLKM_DIR))
include $(DLKM_DIR)/Build_external_kernelmodule.mk

ifeq ($(CONFIG_LOCALVERSION), "-gki-consolidate")
include $(CLEAR_VARS)
KBUILD_OPTIONS += MODNAME=ipatestm
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := ipatestm.ko
LOCAL_MODULE_KBUILD_NAME  := ipa/ipatestm.ko
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_HEADER_LIBRARIES    := ipa_test_kernel_headers
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
$(warning $(DLKM_DIR))
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

endif #End of Check for target
endif #End of Check for qssi target
endif #DLKM
