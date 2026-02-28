FASTRPC_DLKM_ENABLED := true

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_FASTRPC_OVERRIDE), false)
		FASTRPC_DLKM_ENABLED := false
	endif
endif

ifeq ($(FASTRPC_DLKM_ENABLED), true)
ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/frpc-adsprpc.ko
ifeq ($(TARGET_BOARD_PLATFORM), niobe)
BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/cdsp-loader.ko
endif
endif
endif