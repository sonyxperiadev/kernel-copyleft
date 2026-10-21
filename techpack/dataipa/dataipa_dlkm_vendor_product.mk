PRODUCT_PACKAGES += gsim.ko
PRODUCT_PACKAGES += ipam.ko
PRODUCT_PACKAGES += ipanetm.ko
PRODUCT_PACKAGES += rndisipam.ko
PRODUCT_PACKAGES += ipa_clientsm.ko
ifeq ($(CONFIG_LOCALVERSION), "-gki-consolidate")
PRODUCT_PACKAGES += ipatestm.ko
endif
