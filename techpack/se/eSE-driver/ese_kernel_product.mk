ifeq ($(strip $(TARGET_USES_ST_ESE)),true)
PRODUCT_PACKAGES += stm_st54se_gpio.ko
endif
