# Android makefile for securemsm kernel modules

ENABLE_SECUREMSM_DLKM := true
ENABLE_SECUREMSM_QTEE_DLKM := true

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_SECURE_MSM_OVERRIDE),false)
    ENABLE_SECUREMSM_DLKM := false
  endif
  ifeq ($(TARGET_KERNEL_DLKM_SECUREMSM_QTEE_OVERRIDE),false)
    ENABLE_SECUREMSM_QTEE_DLKM := false
  endif
endif

ifeq ($(ENABLE_SECUREMSM_DLKM), true)
  ENABLE_QCRYPTO_DLKM := true
  ENABLE_HDCP_QSEECOM_DLKM := true
  ENABLE_QRNG_DLKM := true
  ifeq ($(TARGET_USES_SMMU_PROXY), true)
    ENABLE_SMMU_PROXY := true
  endif #TARGET_USES_SMMU_PROXY
endif #ENABLE_SECUREMSM_DLKM

ifeq ($(ENABLE_SECUREMSM_QTEE_DLKM), true)
  ENABLE_SMCINVOKE_DLKM := true
  ENABLE_TZLOG_DLKM := true
  #Enable Qseecom if TARGET_ENABLE_QSEECOM or TARGET_BOARD_AUTO is set to true
  ifneq (, $(filter true, $(TARGET_ENABLE_QSEECOM) $(TARGET_BOARD_AUTO)))
    ENABLE_QSEECOM_DLKM := true
  endif #TARGET_ENABLE_QSEECOM OR TARGET_BOARD_AUTO
endif #ENABLE_SECUREMSM_QTEE_DLKM

ifeq ($(TARGET_USES_GY), true)
  ENABLE_QCRYPTO_DLKM := false
  ENABLE_HDCP_QSEECOM_DLKM := false
  ENABLE_QRNG_DLKM := false
  ENABLE_SMMU_PROXY := false
  ENABLE_SMCINVOKE_DLKM := true
  ENABLE_TZLOG_DLKM := false
  ENABLE_QSEECOM_DLKM := false
endif #TARGET_USES_GY

LOCAL_PATH := $(call my-dir)

VENDOR_OPENSOURCE_DIR ?= vendor/qcom/opensource
VENDOR_COMMON_DIR ?= device/qcom/common

DLKM_DIR := $(TOP)/$(VENDOR_COMMON_DIR)/dlkm

SEC_KERNEL_DIR := $(TOP)/$(VENDOR_OPENSOURCE_DIR)/securemsm-kernel

LOCAL_EXPORT_KO_INCLUDE_DIRS := $(LOCAL_PATH)/include/ \
                                $(LOCAL_PATH)/include/uapi

SSG_SRC_FILES := \
	$(wildcard $(LOCAL_PATH)/*) \
 	$(wildcard $(LOCAL_PATH)/*/*) \
 	$(wildcard $(LOCAL_PATH)/*/*/*) \
 	$(wildcard $(LOCAL_PATH)/*/*/*/*)
LOCAL_MODULE_DDK_BUILD := true
# This is set once per LOCAL_PATH, not per (kernel) module
KBUILD_OPTIONS := SSG_ROOT=$(SEC_KERNEL_DIR)
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)

CONDITIONAL_FLAGS := $(ENABLE_SECUREMSM_QTEE_DLKM) $(ENABLE_SECUREMSM_DLKM)

ifneq (0, $(words $(filter true, $(CONDITIONAL_FLAGS))))
include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := sec-module-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

ifeq ($(ENABLE_SMCINVOKE_DLKM), true)
include $(CLEAR_VARS)
#LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := smcinvoke_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := smcinvoke_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_HEADER_LIBRARIES    := smcinvoke_kernel_headers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_SMCINVOKE_DLKM
###################################################
###################################################
ifeq ($(ENABLE_TZLOG_DLKM), true)
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := tz_log_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := tz_log_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_TZLOG_DLKM

ifeq ($(ENABLE_QSEECOM_DLKM), true)
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := qseecom_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := qseecom_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_QSEECOM_DLKM
###################################################
###################################################

ifeq ($(ENABLE_QCRYPTO_DLKM), true)
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := qce50_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := qce50_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
###################################################
###################################################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := qcedev-mod_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := qcedev-mod_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
###################################################
###################################################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := qcrypto-msm_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := qcrypto-msm_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_QCRYPTO_DLKM
###################################################
###################################################
ifeq ($(ENABLE_HDCP_QSEECOM_DLKM), true)
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := hdcp_qseecom_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := hdcp_qseecom_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_HDCP_QSEECOM_DLKM
###################################################
###################################################
ifeq ($(ENABLE_QRNG_DLKM), true)
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_MODULE              := qrng_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := qrng_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_QRNG_DLKM
###################################################
###################################################
ifeq ($(ENABLE_SMMU_PROXY), true)
include $(CLEAR_VARS)
#LOCAL_SRC_FILES           := $(SSG_SRC_FILES)
LOCAL_EXPORT_KO_INCLUDE_DIRS := $(LOCAL_PATH)/smmu-proxy/ $(LOCAL_PATH)/
LOCAL_MODULE              := smmu_proxy_dlkm.ko
LOCAL_MODULE_KBUILD_NAME  := smmu_proxy_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif #ENABLE_SMMU_PROXY
