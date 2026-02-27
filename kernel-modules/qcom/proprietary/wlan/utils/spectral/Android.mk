ifeq ($(BOARD_HAS_QCOM_WLAN), true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_REQUIRED_MODULES :=
LOCAL_CFLAGS += -Wno-unused-parameter -Wno-int-to-pointer-cast
LOCAL_CFLAGS += -Wno-maybe-uninitialized -Wno-parentheses
LOCAL_CFLAGS += -DCONFIG_CLD80211_LIB

LOCAL_MODULE := spectraltool
ifeq ($(PRODUCT_VENDOR_MOVE_ENABLED), true)
LOCAL_PROPRIETARY_MODULE := true
endif
LOCAL_CLANG := true
LOCAL_MODULE_TAGS := optional

LOCAL_C_INCLUDES :=
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH) \
    external/libnl/include \
    external/icu/icu4c/source/common \
    $(LOCAL_PATH)/../uapi/linux
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/cld80211-lib

LOCAL_SHARED_LIBRARIES :=

ifneq ($(wildcard external/libnl),)
LOCAL_SHARED_LIBRARIES += libnl
endif

LOCAL_SHARED_LIBRARIES += libdl libcld80211

LOCAL_SRC_FILES :=
LOCAL_SRC_FILES += spectraltool.c
LOCAL_HEADER_LIBRARIES := libcld80211_headers
LOCAL_LDLIBS += -Lpthread
LOCAL_SANITIZE := integer_overflow
include $(BUILD_EXECUTABLE)

endif # Global WLAN enable check
