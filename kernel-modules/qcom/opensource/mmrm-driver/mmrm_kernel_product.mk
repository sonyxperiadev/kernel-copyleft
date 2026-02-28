ifneq ($(filter $(TARGET_BOARD_PLATFORM), monaco vienna),$(TARGET_BOARD_PLATFORM))
PRODUCT_PACKAGES += msm-mmrm.ko
endif
