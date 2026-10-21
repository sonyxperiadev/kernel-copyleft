MM_DRIVER_PATH := $(call my-dir)

MM_DRV_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_MM_DRV_OVERRIDE), false)
		MM_DRV_DLKM_ENABLE := false
	endif
endif

ifeq ($(MM_DRV_DLKM_ENABLE), true)
	include $(MM_DRIVER_PATH)/msm_ext_display/Android.mk
	ifneq ($(TARGET_BOARD_PLATFORM), taro)
		include $(MM_DRIVER_PATH)/hw_fence/Android.mk
		include $(MM_DRIVER_PATH)/sync_fence/Android.mk
	endif
endif
