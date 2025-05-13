MY_LOCAL_PATH := $(call my-dir)
DLKM_DIR   := $(TOP)/device/qcom/common/dlkm

ifeq ($(SOMC_CFG_TOUCHSCREEN_LXS),yes)

include $(MY_LOCAL_PATH)/msm/lxs_ts/Android.mk

endif