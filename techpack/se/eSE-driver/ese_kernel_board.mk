# Check and set local DLKM flag based on system-wide global flags
ESE_DLKM_ENABLED := false

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_ESE_OVERRIDE), true)
    ESE_DLKM_ENABLED := true
  endif
else
  ESE_DLKM_ENABLED := true
endif

# Target specific build rules
ifeq ($(call is-board-platform-in-list, pineapple sun),true)
  ifeq ($(ESE_DLKM_ENABLED), true)
    TARGET_USES_ST_ESE :=true
    BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/stm_st54se_gpio.ko
  endif
endif
