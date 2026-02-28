SPU_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_SPU_OVERRIDE), false)
		SPU_DLKM_ENABLE := false
	endif
endif

ifeq ($(SPU_DLKM_ENABLE),  true)
	ifneq ($(TARGET_BOARD_AUTO),true)
		ifeq ($(call is-board-platform-in-list, kalama pineapple sun),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/spcom.ko \
				$(KERNEL_MODULES_OUT)/spss_utils.ko
		endif
	endif
endif
