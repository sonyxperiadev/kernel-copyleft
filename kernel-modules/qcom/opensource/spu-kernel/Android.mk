# Android makefile for spu kernel modules (spcom.ko and spss_utils.ko)
ifeq ($(call is-vendor-board-platform,QCOM),true)
ifeq ($(call is-board-platform-in-list,kalama pineapple sun),true)

# This makefile is only for DLKM
ifneq ($(findstring vendor,$(LOCAL_PATH)),)
	LOCAL_PATH := $(call my-dir)
	SPU_BLD_DIR := $(TOP)/vendor/qcom/opensource/spu-kernel
	LOCAL_MODULE_DDK_BUILD := true
	LOCAL_MODULE_KO_DIRS := drivers/spcom.ko drivers/spss_utils.ko
	DLKM_DIR := $(TOP)/device/qcom/common/dlkm
	INSTALL_MODULE_HEADERS := 1

	# Kbuild options
	KBUILD_OPTIONS := SPU_ROOT=$(SPU_BLD_DIR)
	KBUILD_OPTIONS += MODNAME=spcom
	KBUILD_OPTIONS += MODNAME=spss_utils

	########################### spcom ################################
	include $(CLEAR_VARS)
	LOCAL_SRC_FILES   :=  $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
	LOCAL_MODULE              := spcom.ko
	LOCAL_MODULE_KBUILD_NAME  := drivers/spcom.ko
	LOCAL_MODULE_TAGS         := optional
	LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
	include $(DLKM_DIR)/Build_external_kernelmodule.mk

	########################### spss_utils ################################
	include $(CLEAR_VARS)
	LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
	LOCAL_MODULE              := spss_utils.ko
	LOCAL_MODULE_KBUILD_NAME  := drivers/spss_utils.ko
	LOCAL_MODULE_TAGS         := optional
	LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
	include $(DLKM_DIR)/Build_external_kernelmodule.mk
	###########################################################

	LOCAL_SHARED_LIBRARIES := libdmabufheap
endif
endif
endif # End of check for board platform
