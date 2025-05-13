ENABLE_EVA_KERNEL := true
ifeq ($(TARGET_USES_QMAA), true)
ifneq ($(TARGET_USES_QMAA_OVERRIDE_CVP), true)
ENABLE_EVA_KERNEL := false
endif
endif

ifeq ($(ENABLE_EVA_KERNEL), true)
ifneq ($(TARGET_BOARD_PLATFORM), qssi)
ifeq ($(call is-board-platform-in-list, $(TARGET_BOARD_PLATFORM)),true)

DLKM_DIR   := device/qcom/common/dlkm

LOCAL_PATH := $(call my-dir)
# For DDK
LOCAL_MODULE_DDK_BUILD := true
LOCAL_MODULE_KO_DIRS := msm/msm-eva.ko

include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE      := msm-eva.ko
LOCAL_MODULE_KBUILD_NAME := msm/msm-eva.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)

LOCAL_ADDITIONAL_DEPENDENCY      := synx-driver.ko

# export to kbuild
# Setup mmrm dependency
LOCAL_REQUIRED_MODULES    := mmrm-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
KBUILD_REQUIRED_KOS += msm-mmrm.ko

# Setup SynX dependency
CONFIG_SYNX := y
#ifdef CONFIG_SYNX
ifeq ($(CONFIG_SYNX), y)
$(warning Compiling SynX)
LOCAL_REQUIRED_MODULES    += synx-driver-symvers
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,synx-driver-symvers)/synx-driver-symvers
KBUILD_REQUIRED_KOS += synx-driver.ko
endif

# Setup fastRPC dependency
CONFIG_FASTRPC := y
ifeq ($(CONFIG_FASTRPC), y)
$(warning Compiling FastRPC)
LOCAL_REQUIRED_MODULES    += dsp-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,dsp-module-symvers)/Module.symvers
KBUILD_REQUIRED_KOS += frpc-adsprpc.ko
endif

# print out variables
$(info KBUILD_OPTIONS = $(KBUILD_OPTIONS))
$(info intermediates mmrm symvers path = $(call intermediates-dir-for,DLKM,mmrm-module-symvers))
$(info LOCAL_ADDITIONAL_DEPENDENCY = $(LOCAL_ADDITIONAL_DEPENDENCY))
$(info LOCAL_ADDITIONAL_DEPENDENCIES = $(LOCAL_ADDITIONAL_DEPENDENCIES))
$(info LOCAL_REQUIRED_MODULES = $(LOCAL_REQUIRED_MODULES))
$(info DLKM_DIR = $(DLKM_DIR))

include $(DLKM_DIR)/Build_external_kernelmodule.mk

endif # End of check for board platform
endif # End of check for target product
endif # End of enable eva kernel check