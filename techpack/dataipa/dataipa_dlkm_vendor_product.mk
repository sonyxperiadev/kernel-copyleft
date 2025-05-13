PRODUCT_PACKAGES += gsim.ko
PRODUCT_PACKAGES += ipam.ko
PRODUCT_PACKAGES += ipanetm.ko
ifeq ($(CONFIG_LOCALVERSION), "-gki-consolidate")
PRODUCT_PACKAGES += ipatestm.ko
endif
