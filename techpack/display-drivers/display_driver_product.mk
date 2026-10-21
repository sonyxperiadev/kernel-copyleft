# SPDX-License-Identifier: GPL-2.0-only

DISPLAY_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_DISPLAY_OVERRIDE), false)
		DISPLAY_DLKM_ENABLE := false
	endif
endif

ifeq ($(DISPLAY_DLKM_ENABLE),  true)
	PRODUCT_PACKAGES += msm_drm.ko
endif

DISPLAY_MODULES_DRIVER := msm_drm.ko