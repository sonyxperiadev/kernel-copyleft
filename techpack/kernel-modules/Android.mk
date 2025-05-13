MY_LOCAL_PATH := $(call my-dir)
DLKM_DIR   := $(TOP)/device/qcom/common/dlkm

ifeq ($(SOMC_CFG_POWERKEY_FORCECRASH),yes)

include $(MY_LOCAL_PATH)/powerkey_forcecrash/Android.mk

endif

ifeq ($(SOMC_CFG_LAST_LOGS),yes)

include $(MY_LOCAL_PATH)/last_logs/Android.mk

endif

ifeq ($(SOMC_CFG_RAMDUMP_MEM_DESC),yes)

include $(MY_LOCAL_PATH)/ramdump_mem_desc/Android.mk

endif

ifeq ($(SOMC_CFG_RDTAGS),yes)

include $(MY_LOCAL_PATH)/rdtags/Android.mk

endif

