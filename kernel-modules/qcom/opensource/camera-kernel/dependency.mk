# SPDX-License-Identifier: GPL-2.0-only

# Check if this board's product.mk finds msm-mmrm.ko driver
ifeq ($(findstring msm-mmrm.ko,$(BOARD_VENDOR_KERNEL_MODULES)), msm-mmrm.ko)
# Assume if msm-mmrm.ko driver is found, then symbols will be required!

ifeq ($(call is-board-platform-in-list, $(MMRM_BOARDS)),true)
# Add MMRM driver symbols, requires absolute path
CAM_MMRM_EXTRA_SYMBOLS ?= $(realpath $(TOP))/$(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
$(info camera-kernel: Found msm-mmrm driver, adding symbol dependency! $(CAM_MMRM_EXTRA_SYMBOLS))
ifneq ($(TARGET_BOARD_PLATFORM), pineapple)
LOCAL_REQUIRED_MODULES    := mmrm-module-symvers
endif # End of check lanai
CAM_MMRM_EXTRA_CONFIGS ?= $(TOP)/vendor/qcom/opensource/mmrm-driver/config/waipiommrm.conf
LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers

endif # End of check for board platform MMRM_BOARDS

endif # End of find msm-mmrm driver

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(CAM_MMRM_EXTRA_SYMBOLS)
KBUILD_OPTIONS += KBUILD_EXTRA_CONFIGS=$(CAM_MMRM_EXTRA_CONFIGS)
