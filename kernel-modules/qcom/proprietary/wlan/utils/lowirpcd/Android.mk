ifeq ($(call is-vendor-board-platform,QCOM),true)
LOCAL_PATH:= $(call my-dir)

ifneq ($(call is-board-platform-in-list,$(TRINKET) sdm710 $(MSMSTEPPE) qcs605), true)
include $(CLEAR_VARS)
LOCAL_C_INCLUDES := \
        $(LOCAL_PATH)/inc \

LOCAL_HEADER_LIBRARIES  := libfastrpc_vendor_headers

LOCAL_HEADER_LIBRARIES += vendor_common_inc

LOCAL_SRC_FILES:= \
        src/lowirpcd.cpp \

ifneq ($(call is-board-platform-in-list, lito bengal atoll),true)
LOCAL_CFLAGS := -DADSP_DEFAULT_LISTENER_NAME=\"libssc_default_listener.so\"
endif
LOCAL_SHARED_LIBRARIES := liblog libdl
LOCAL_CFLAGS += -Werror -Wall -fexceptions
LOCAL_MODULE := lowirpcd
LOCAL_INIT_RC := vendor.wlan.lowirpcd.rc

LOCAL_PROPRIETARY_MODULE := true
LOCAL_PRELINK_MODULE := false
LOCAL_UNINSTALLABLE_MODULE :=
LOCAL_MODULE_OWNER := qti

include $(BUILD_EXECUTABLE)

endif
endif
