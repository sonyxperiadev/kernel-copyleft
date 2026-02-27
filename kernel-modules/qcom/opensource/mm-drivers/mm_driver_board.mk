#SPDX-License-Identifier: GPL-2.0-only

MM_DRV_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_MM_DRV_OVERRIDE), false)
		MM_DRV_DLKM_ENABLE := false
	endif
endif

ifeq ($(MM_DRV_DLKM_ENABLE), true)
	ifneq ($(TARGET_BOARD_AUTO),true)
		ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/msm_ext_display.ko
			BOARD_VENDOR_RAMDISK_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/msm_ext_display.ko
			BOARD_VENDOR_RAMDISK_RECOVERY_KERNEL_MODULES_LOAD += $(KERNEL_MODULES_OUT)/msm_ext_display.ko
			ifneq ($(TARGET_BOARD_PLATFORM), taro)
				BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/sync_fence.ko \
					       $(KERNEL_MODULES_OUT)/msm_hw_fence.ko
				BOARD_VENDOR_RAMDISK_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/sync_fence.ko \
					               $(KERNEL_MODULES_OUT)/msm_hw_fence.ko
				BOARD_VENDOR_RAMDISK_RECOVERY_KERNEL_MODULES_LOAD += $(KERNEL_MODULES_OUT)/sync_fence.ko \
					                             $(KERNEL_MODULES_OUT)/msm_hw_fence.ko
			endif
		endif
	endif
endif
