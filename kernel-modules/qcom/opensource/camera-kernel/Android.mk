CAMERA_DLKM_ENABLED := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_CAMERA_OVERRIDE), false)
		CAMERA_DLKM_ENABLED := false;
	endif
endif

ifeq ($(CAMERA_DLKM_ENABLED),true)
ifeq ($(call is-board-platform-in-list, $(TARGET_BOARD_PLATFORM)),true)

# Make target to specify building the camera.ko from within Android build system.
LOCAL_PATH := $(call my-dir)
# Path to DLKM make scripts
DLKM_DIR := $(TOP)/device/qcom/common/dlkm

# List of board platforms for which MMRM driver API should be enabled
MMRM_BOARDS := taro parrot kalama pineapple

CAMERA_SRC_FILES := \
                    $(addprefix $(LOCAL_PATH)/, $(call all-named-files-under,*.h,drivers dt-bindings include))\
                    $(addprefix $(LOCAL_PATH)/, $(call all-named-files-under,*.mk,config))\
                    $(addprefix $(LOCAL_PATH)/, $(call all-named-files-under,*.c,drivers))\
                    $(LOCAL_PATH)/dependency.mk \
                    $(LOCAL_PATH)/board.mk      \
                    $(LOCAL_PATH)/product.mk    \
                    $(LOCAL_PATH)/Kbuild

# Target for pre-sil symbols
ifeq ($(CONFIG_CAM_PRESIL), y)
$(warning camera-kernel: Enabling Pre-Sil Kbuild Options!)
KBUILD_OPTIONS := CONFIG_CAM_PRESIL=y
include $(CLEAR_VARS)
$(warning camera-kernel: Enabling Pre-Sil build, exporting symbols!)
LOCAL_SRC_FILES           := $(CAMERA_SRC_FILES)
LOCAL_MODULE              := camera-kernel-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
# Check build for optional dependencies
include $(LOCAL_PATH)/dependency.mk

include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

# Kbuild options
KBUILD_OPTIONS := CAMERA_KERNEL_ROOT=$(TOP)/$(LOCAL_PATH)
KBUILD_OPTIONS += KERNEL_ROOT=$(TOP)/kernel_platform/common
KBUILD_OPTIONS += MODNAME=camera
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)

# Clear shell environment variables from previous android module during build
include $(CLEAR_VARS)
# For incremental compilation support.
LOCAL_SRC_FILES             := $(CAMERA_SRC_FILES)
LOCAL_MODULE_PATH           := $(KERNEL_MODULES_OUT)
LOCAL_MODULE                := camera.ko
LOCAL_MODULE_TAGS           := optional
#LOCAL_MODULE_KBUILD_NAME   := camera.ko
#LOCAL_MODULE_DEBUG_ENABLE  := true

# Check build for optional dependencies
include $(LOCAL_PATH)/dependency.mk

# $(info LOCAL_SRC_FILES = $(LOCAL_SRC_FILES))
# $(info intermediates mmrm symvers path = $(call intermediates-dir-for,DLKM,mmrm-module-symvers))
# $(info CAMERA_EXTRA_SYMBOLS = $(CAMERA_EXTRA_SYMBOLS))
# $(info CAMERA_EXTRA_CONFIGS = $(CAMERA_EXTRA_CONFIGS))
# $(info LOCAL_ADDITIONAL_DEPENDENCIES = $(LOCAL_ADDITIONAL_DEPENDENCIES))
# $(info LOCAL_REQUIRED_MODULES = $(LOCAL_REQUIRED_MODULES))
# $(info DLKM_DIR = $(DLKM_DIR))
$(info KBUILD_OPTIONS = $(KBUILD_OPTIONS))

ifeq ($(TARGET_BOARD_PLATFORM), lahaina)
# Include Kernel DLKM Android.mk target to place generated .ko file in image
include $(DLKM_DIR)/AndroidKernelModule.mk
# Include Camera UAPI Android.mk target to copy headers
include $(LOCAL_PATH)/include/uapi/Android.mk
else
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

endif # End of check for board platform
endif # ifeq ($(CAMERA_DLKM_ENABLED),true)
