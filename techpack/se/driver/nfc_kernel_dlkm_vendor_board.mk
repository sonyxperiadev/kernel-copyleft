NFC_DLKM_ENABLED := false

########## Check and set local DLKM flag based on system-wide global flags ##########
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_NFC_OVERRIDE), true)
    NFC_DLKM_ENABLED := true
  endif
else
  NFC_DLKM_ENABLED := true
endif

########## Build kernel module based on local DLKM flag status ##########
# Build NFC kernel driver
ifeq ($(NFC_DLKM_ENABLED), true)
ifeq ($(call is-board-platform-in-list, pineapple),true)
  BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/stm_nfc_i2c.ko
endif
endif
