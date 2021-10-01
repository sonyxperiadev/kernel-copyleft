MY_LOCAL_PATH := $(call my-dir)
DLKM_DIR   := $(TOP)/device/qcom/common/dlkm

ifeq ($(SOMC_CFG_TOUCHSCREEN_CLEARPAD),yes)

include $(MY_LOCAL_PATH)/msm/clearpad/clearpad_core/Android.mk
include $(MY_LOCAL_PATH)/msm/clearpad/clearpad_i2c/Android.mk
include $(MY_LOCAL_PATH)/msm/clearpad/clearpad_rmi_dev/Android.mk

endif

ifeq ($(SOMC_CFG_TOUCHSCREEN_ATMEL),yes)

include $(MY_LOCAL_PATH)/msm/atmel_mxt640u/Android.mk

endif

ifeq ($(SOMC_CFG_SIM_DETECTION),yes)

include $(MY_LOCAL_PATH)/misc/sim_detect/Android.mk

endif

ifeq ($(SOMC_CFG_HALL_IC),yes)

include $(MY_LOCAL_PATH)/misc/bu520x1nvx/Android.mk

endif

ifeq ($(SOMC_CFG_LDO_VIBRATOR),yes)

include $(MY_LOCAL_PATH)/misc/ldo_vibrator/Android.mk

endif

ifeq ($(SOMC_CFG_TOUCHSCREEN_SYNAPTICS),yes)

include $(MY_LOCAL_PATH)/msm/synaptics_tcm/Android.mk

endif

ifeq ($(SOMC_CFG_TOUCHSCREEN_SIW),yes)

include $(MY_LOCAL_PATH)/msm/siw/Android.mk

endif

ifeq ($(SOMC_CFG_TOUCHSCREEN_SIWMON),yes)

include $(MY_LOCAL_PATH)/msm/siw/mon/Android.mk

endif

ifeq ($(SOMC_CFG_TOUCHSCREEN_SAMSUNG),yes)

include $(MY_LOCAL_PATH)/msm/sec_ts/Android.mk

endif

ifeq ($(SOMC_CFG_TOUCHSCREEN_NVT_NT36672A),yes)

include $(MY_LOCAL_PATH)/msm/nvt_nt36672a/Android.mk

endif

ifeq ($(SOMC_CFG_CPSENSOR_ADUX1050),yes)

include $(MY_LOCAL_PATH)/misc/adux1050/Android.mk

endif
