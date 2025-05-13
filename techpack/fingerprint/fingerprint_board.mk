FINGERPRINT_DLKM_ENABLED := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
       ifeq ($(TARGET_KERNEL_DLKM_FINGERPRINT_OVERRIDE), false)
             FINGERPRINT_DLKM_ENABLED := false;
       endif
endif

ifeq ($(FINGERPRINT_DLKM_ENABLED), true)
ifeq ($(call is-board-platform-in-list,pineapple), true)
BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/qbt_handler.ko
endif

endif
