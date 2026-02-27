# Build/Package options for different targets
ifneq (,$(filter msm8974 msm8226 msm8610 msm8916 msm8909 msm8952 msm8937 msm8953,$(TARGET_BOARD_PLATFORM)))
BUILD_PTT_SOCKET_APP:=1
endif


ifeq ($(BUILD_PTT_SOCKET_APP),1)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES += \
	pttSocketApp.c

LOCAL_CFLAGS += \
	-fno-short-enums  \
	-Wall -Werror

#LOCAL_CFLAGS += \
	-DANI_DEBUG
LOCAL_CFLAGS += \
	-DWLAN_KD_READY_NOTIFIER

ifeq ($(TARGET_SUPPORTS_WEARABLES),true)
LOCAL_CFLAGS += \
       -DTARGET_SUPPORTS_WEARABLES
endif

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../asf/inc
LOCAL_HEADER_LIBRARIES := vendor_common_inc
LOCAL_HEADER_LIBRARIES += libdiag_headers

LOCAL_SHARED_LIBRARIES := \
	libutils \
	libcutils \
	libdiag \
        liblog

LOCAL_STATIC_LIBRARIES := \
	libAniAsf

LOCAL_PRELINK_MODULE := false

LOCAL_MODULE := ptt_socket_app
ifeq ($(PRODUCT_VENDOR_MOVE_ENABLED),true)
LOCAL_CFLAGS += -DVENDOR_MOVE_ENABLED
LOCAL_PROPRIETARY_MODULE := true
endif
LOCAL_MODULE_TAGS := optional

LOCAL_MODULE_OWNER := qcom
LOCAL_SANITIZE := integer_overflow
include $(BUILD_EXECUTABLE)

endif
