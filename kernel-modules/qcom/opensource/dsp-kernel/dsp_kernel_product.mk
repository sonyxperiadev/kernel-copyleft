FASTRPC_DLKM_ENABLED := true

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_FASTRPC_OVERRIDE), false)
		FASTRPC_DLKM_ENABLED := false
	endif
endif

ifeq ($(FASTRPC_DLKM_ENABLED), true)
PRODUCT_PACKAGES += frpc-adsprpc.ko
ifeq ($(TARGET_BOARD_PLATFORM), niobe)
PRODUCT_PACKAGES += cdsp-loader.ko
endif
endif