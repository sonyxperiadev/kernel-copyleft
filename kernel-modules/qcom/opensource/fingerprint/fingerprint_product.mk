ifeq ($(call is-board-platform-in-list,pineapple), true)
PRODUCT_PACKAGES += qbt_handler.ko
endif
