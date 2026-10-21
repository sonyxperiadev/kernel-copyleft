
PRODUCT_PACKAGES += msm_ext_display.ko

MM_DRV_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_MM_DRV_OVERRIDE), false)
		MM_DRV_DLKM_ENABLE := false
	endif
endif

ifeq ($(MM_DRV_DLKM_ENABLE), true)
	ifneq ($(TARGET_BOARD_PLATFORM), taro)
		PRODUCT_PACKAGES += sync_fence.ko msm_hw_fence.ko
	endif
endif

DISPLAY_MM_DRIVER := msm_ext_display.ko sync_fence.ko msm_hw_fence.ko