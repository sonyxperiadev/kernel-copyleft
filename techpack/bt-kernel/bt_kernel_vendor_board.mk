# Build audio kernel driver
ifneq ($(TARGET_BOARD_AUTO),true)
ifeq ($(TARGET_USES_QMAA),true)
  ifeq ($(TARGET_USES_QMAA_OVERRIDE_BLUETOOTH), true)
     ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
           BT_KERNEL_DRIVER := $(KERNEL_MODULES_OUT)/btpower.ko\
             $(KERNEL_MODULES_OUT)/bt_fm_slim.ko \
             $(KERNEL_MODULES_OUT)/radio-i2c-rtc6226-qca.ko
           BOARD_VENDOR_KERNEL_MODULES += $(BT_KERNEL_DRIVER)
     endif
  endif
else
  ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
     BT_KERNEL_DRIVER := $(KERNEL_MODULES_OUT)/btpower.ko\
            $(KERNEL_MODULES_OUT)/bt_fm_slim.ko \
            $(KERNEL_MODULES_OUT)/radio-i2c-rtc6226-qca.ko
     BOARD_VENDOR_KERNEL_MODULES += $(BT_KERNEL_DRIVER)
  endif
endif
endif
