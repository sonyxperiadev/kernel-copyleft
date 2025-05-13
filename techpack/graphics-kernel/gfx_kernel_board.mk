#SPDX-License-Identifier: GPL-2.0-only

ifneq ($(TARGET_USES_QMAA),true)
	ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
		BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/msm_kgsl.ko
		BOARD_VENDOR_RAMDISK_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/msm_kgsl.ko
		BOARD_VENDOR_RAMDISK_RECOVERY_KERNEL_MODULES_LOAD += $(KERNEL_MODULES_OUT)/msm_kgsl.ko
	endif
endif
