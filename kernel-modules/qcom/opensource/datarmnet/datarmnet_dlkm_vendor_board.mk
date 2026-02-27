TARGET_DATARMNET_ENABLE := false

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_DATARMNET_OVERRIDE), true)
		TARGET_DATARMNET_ENABLE := true
	endif
else
	TARGET_DATARMNET_ENABLE := true
endif

ifeq ($(TARGET_DATARMNET_ENABLE), true)
	#Build rmnet core
	DATA_DLKM_BOARD_PLATFORMS_LIST := pineapple
	DATA_DLKM_BOARD_PLATFORMS_LIST += blair
	DATA_DLKM_BOARD_PLATFORMS_LIST += monaco

	ifneq ($(TARGET_BOARD_AUTO),true)
		ifeq ($(call is-board-platform-in-list,$(DATA_DLKM_BOARD_PLATFORMS_LIST)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_core.ko
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_ctl.ko
		endif
	endif
endif
