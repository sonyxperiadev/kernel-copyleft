ifneq ($(TARGET_KERNEL_DLKM_DISABLE), true)
PRODUCT_PACKAGES += ubwcp.ko
endif
