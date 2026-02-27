# Android Makefile for WLAN platform modules

ENABLE_WLAN_PLATFORM_DLKM := false
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_WLAN_OVERRIDE), true)
    ENABLE_WLAN_PLATFORM_DLKM := true
  endif
else
  ENABLE_WLAN_PLATFORM_DLKM := true
endif

ifeq ($(ENABLE_WLAN_PLATFORM_DLKM), true)

# LOCAL_PATH is a relative path to root build directory.
LOCAL_PATH := $(call my-dir)
LOCAL_MODULE_DDK_BUILD := true
LOCAL_MODULE_DDK_ALLOW_UNSAFE_HEADERS := true
LOCAL_MODULE_KO_DIRS := cnss2/cnss2.ko
LOCAL_MODULE_KO_DIRS += cnss_utils/cnss_plat_ipc_qmi_svc.ko
LOCAL_MODULE_KO_DIRS += cnss_utils/wlan_firmware_service.ko
LOCAL_MODULE_KO_DIRS += cnss_genl/cnss_nl.ko
LOCAL_MODULE_KO_DIRS += cnss_prealloc/cnss_prealloc.ko
LOCAL_MODULE_KO_DIRS += cnss_utils/cnss_utils.ko
LOCAL_MODULE_KO_DIRS += icnss2/icnss2.ko

BOARD_COMMON_DIR ?= device/qcom/common
DLKM_DIR := $(TOP)/$(BOARD_COMMON_DIR)/dlkm

# WLAN_PLATFORM_ROOT needs to be a absolute since it will be used
# for header files. $(TOP) cannot be used here since it will be
# resolved as "." which won't work for Kbuild.
KBUILD_OPTIONS := WLAN_PLATFORM_ROOT=$(abspath $(LOCAL_PATH))

# WLAN_PLATFORM_KBUILD_OPTIONS should be defined from upper level
# Product or Board related Makefiles like wlan.mk.
KBUILD_OPTIONS += $(foreach wlan_platform_kbuild_option, \
		   $(WLAN_PLATFORM_KBUILD_OPTIONS), \
		   $(wlan_platform_kbuild_option))

# Sourcing all files is for better incremental compilation.
CNSS_SRC_FILES := \
	$(wildcard $(LOCAL_PATH)/*) \
	$(wildcard $(LOCAL_PATH)/*/*) \

ifeq ($(TARGET_KERNEL_DLKM_SECURE_MSM_OVERRIDE), true)
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(PWD)/$(call intermediates-dir-for,DLKM,sec-module-symvers)/Module.symvers
endif

# Module.symvers needs to be generated as a intermediate module so that
# other modules which depend on WLAN platform modules can set local
# dependencies to it.

########################### Module.symvers ############################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := wlan-platform-module-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

# Below are for Android build system to recognize each module name, so
# they can be installed properly. Since Kbuild is used to compile these
# modules, invoking any of them will cause other modules to be compiled
# as well if corresponding flags are added in KBUILD_OPTIONS from upper
# level Makefiles like wlan.mk.

################################ cnss2 ################################
include $(CLEAR_VARS)
ifeq ($(TARGET_KERNEL_DLKM_SECURE_MSM_OVERRIDE), true)
LOCAL_REQUIRED_MODULES := sec-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,sec-module-symvers)/Module.symvers
endif #TARGET_KERNEL_DLKM_SECURE_MSM_OVERRIDE
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := cnss2.ko
LOCAL_MODULE_KBUILD_NAME  := cnss2/cnss2.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
######################## cnss_plat_ipc_qmi_svc ########################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := cnss_plat_ipc_qmi_svc.ko
LOCAL_MODULE_KBUILD_NAME  := cnss_utils/cnss_plat_ipc_qmi_svc.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
######################## wlan_firmware_service ########################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := wlan_firmware_service.ko
LOCAL_MODULE_KBUILD_NAME  := cnss_utils/wlan_firmware_service.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
############################### cnss_nl ###############################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := cnss_nl.ko
LOCAL_MODULE_KBUILD_NAME  := cnss_genl/cnss_nl.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
############################ cnss_prealloc ############################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := cnss_prealloc.ko
LOCAL_MODULE_KBUILD_NAME  := cnss_prealloc/cnss_prealloc.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
############################ cnss_utils ###############################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := cnss_utils.ko
LOCAL_MODULE_KBUILD_NAME  := cnss_utils/cnss_utils.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
################################ icnss2 ################################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(CNSS_SRC_FILES)
LOCAL_MODULE              := icnss2.ko
LOCAL_MODULE_KBUILD_NAME  := icnss2/icnss2.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_WLAN_PLATFORM_DLKM
