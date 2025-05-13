# SPDX-License-Identifier: GPL-2.0-only

# Check if this board's product.mk finds msm-mmrm.ko driver
ifeq ($(findstring msm-mmrm.ko,$(BOARD_VENDOR_KERNEL_MODULES)), msm-mmrm.ko)
# Assume if msm-mmrm.ko driver is found, then symbols will be required!

ifeq ($(call is-board-platform-in-list, $(MMRM_BOARDS)),true)
# Add MMRM driver symbols, requires absolute path
CAM_MMRM_EXTRA_SYMBOLS ?= $(realpath $(TOP))/$(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
$(info camera-kernel: Found msm-mmrm driver, adding symbol dependency! $(CAM_MMRM_EXTRA_SYMBOLS))
ifneq ($(TARGET_BOARD_PLATFORM), pineapple)
LOCAL_REQUIRED_MODULES  := mmrm-module-symvers
endif # End of check lanai
CAM_MMRM_EXTRA_CONFIGS ?= $(realpath $(TOP))/vendor/qcom/opensource/mmrm-driver/config/waipiommrm.conf
LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers

endif # End of check for board platform MMRM_BOARDS

endif # End of find msm-mmrm driver

# Check if this board's product.mk finds synx-driver.ko driver
ifeq ($(findstring synx-driver.ko,$(BOARD_VENDOR_KERNEL_MODULES)), synx-driver.ko)
# Assume if synx-driver.ko driver is found, then symbols will be required!

ifeq ($(call is-board-platform-in-list, $(SYNX_VENDOR_BOARDS)),true)
# Add SYNX driver symbols, requires absolute path
CAM_SYNX_EXTRA_SYMBOLS ?= $(realpath $(TOP))/$(call intermediates-dir-for,DLKM,synx-driver-symvers)/synx-driver-symvers
$(info camera-kernel: Found synx driver, adding symbol dependency! $(CAM_SYNX_EXTRA_SYMBOLS))
LOCAL_REQUIRED_MODULES    := synx-driver-symvers
CAM_SYNX_EXTRA_CONFIGS ?= $(realpath $(TOP))/vendor/qcom/opensource/synx-kernel/config/pineapplesynx.conf
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,synx-driver-symvers)/synx-driver-symvers

endif # End of check for board platform SYNX_VENDOR_BOARDS

endif # End of find synx driver

# Check if this board's product.mk finds smcinvoke_dlkm.ko driver
ifeq ($(findstring smcinvoke_dlkm.ko, $(BOARD_VENDOR_KERNEL_MODULES)), smcinvoke_dlkm.ko)

ifeq ($(call is-board-platform-in-list, $(SMCINVOKE_DLKM_BOARDS)),true)
SMCINVOKE_EXTRA_SYMBOLS ?= $(realpath $(TOP))/$(call intermediates-dir-for,DLKM,smcinvoke_dlkm.ko)/Module.symvers
$(info camera-kernel: Found smcinvoke driver, adding symbol dependency! $(SMCINVOKE_EXTRA_SYMBOLS))
LOCAL_REQUIRED_MODULES  += smcinvoke_dlkm.ko
CAM_SMCINOKE_EXTRA_CONFIGS ?= $(realpath $(TOP))/vendor/qcom/opensource/securemsm-kernel/config/sec-kernel_defconfig_smcinvoke.conf
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,smcinvoke_dlkm.ko)/Module.symvers

endif # End of check for board platform SMCINVOKE_DLKM_BOARDS

endif # End of find smcinvoke_dlkm driver

# Check if this board's product.mk finds smmu_proxy_dlkm.ko driver
ifeq ($(findstring smmu_proxy_dlkm.ko, $(BOARD_VENDOR_KERNEL_MODULES)), smmu_proxy_dlkm.ko)

ifeq ($(call is-board-platform-in-list, $(SMMU_PROXY_DLKM_BOARDS)),true)
SMMU_PROXY_EXTRA_SYMBOLS ?= $(realpath $(TOP))/$(call intermediates-dir-for,DLKM,smmu_proxy_dlkm.ko)/Module.symvers
$(info camera-kernel: Found smmu proxy driver, adding symbol dependency! $(SMMU_PROXY_EXTRA_SYMBOLS))
LOCAL_REQUIRED_MODULES  += smmu_proxy_dlkm.ko
CAM_SMMU_PROXY_EXTRA_CONFIGS ?= $(realpath $(TOP))/vendor/qcom/opensource/securemsm-kernel/config/sec-kernel_defconfig_smmu_proxy.conf
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,smmu_proxy_dlkm.ko)/Module.symvers

endif # End of check for board platform SMMU_PROXY_DLKM_BOARDS

endif # End of find smmu_proxy_dlkm driver

KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(CAM_MMRM_EXTRA_SYMBOLS) KBUILD_EXTRA_SYMBOLS+=$(CAM_SYNX_EXTRA_SYMBOLS) KBUILD_EXTRA_SYMBOLS+=$(SMCINVOKE_EXTRA_SYMBOLS) KBUILD_EXTRA_SYMBOLS+=$(SMMU_PROXY_EXTRA_SYMBOLS)
KBUILD_OPTIONS += KBUILD_EXTRA_CONFIGS=$(CAM_MMRM_EXTRA_CONFIGS) KBUILD_EXTRA_CONFIGS+=$(CAM_SYNX_EXTRA_CONFIGS) KBUILD_EXTRA_CONFIGS+=$(CAM_SMCINOKE_EXTRA_CONFIGS) KBUILD_EXTRA_CONFIGS+=$(CAM_SMMU_PROXY_EXTRA_CONFIGS)
