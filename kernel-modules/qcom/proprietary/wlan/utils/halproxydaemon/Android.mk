ifeq ($(BOARD_HAS_QCOM_WLAN), true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := hal_proxy_daemon
LOCAL_PROPRIETARY_MODULE := true
LOCAL_CLANG := true
LOCAL_MODULE_TAGS := optional

LOCAL_C_INCLUDES :=
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/inc
LOCAL_C_INCLUDES += \
	$(call include-path-for, libhardware_legacy)/hardware_legacy

ifneq ($(QCOM_WLAN_ROOT), )
    LOCAL_C_INCLUDES += $(QCOM_WLAN_ROOT)/qcwcn/wifi_hal
else
    LOCAL_C_INCLUDES += hardware/qcom/wlan/qcwcn/wifi_hal
endif # QCOM_WLAN_ROOT

LOCAL__CPP_EXTENSION := .cpp

LOCAL_SHARED_LIBRARIES :=

ifneq ($(wildcard external/libnl),)
LOCAL_SHARED_LIBRARIES += libnl
endif

LOCAL_SHARED_LIBRARIES += libcutils
LOCAL_SHARED_LIBRARIES += liblog
LOCAL_SHARED_LIBRARIES += libdl
LOCAL_SHARED_LIBRARIES += libwifi-hal-qcom
LOCAL_SHARED_LIBRARIES += libhardware_legacy

LOCAL_SRC_FILES :=
LOCAL_SRC_FILES += src/halProxyDaemon.cpp
ifneq ($(TARGET_SUPPORTS_ANDROID_WEAR),true)
LOCAL_SRC_FILES += src/nan_test.cpp
endif
LOCAL_SRC_FILES += src/llstats_test.cpp
LOCAL_SRC_FILES += src/wifihal_vendor_test.cpp
LOCAL_SRC_FILES += src/gscan_test.cpp
LOCAL_SRC_FILES += src/rtt_test.cpp
LOCAL_SRC_FILES += src/tdls_test.cpp
LOCAL_SRC_FILES += src/wifihal_test.cpp
LOCAL_SRC_FILES += src/wificonfig_test.cpp
LOCAL_SRC_FILES += src/wifilogger_test.cpp
LOCAL_SRC_FILES += src/wifi_calling_test.cpp
LOCAL_SRC_FILES += src/packet_filtering.cpp
LOCAL_SRC_FILES += src/nd_offload.cpp
LOCAL_SRC_FILES += src/roam_test.cpp

ifeq ($(TARGET_SUPPORTS_ANDROID_WEAR),true)
LOCAL_CFLAGS += "-DANDROID_WEAR"
endif
ifeq ($(shell expr $(PLATFORM_VERSION) \>= 14), 1)
LOCAL_CFLAGS += "-DCONFIG_ANDROID_14"
endif

HAL_PROXY_DAEMON_GIT_VER := $(shell cd $(LOCAL_PATH)/ && git describe --always)
LOCAL_CFLAGS += -DHAL_PROXY_DAEMON_VER=\"$(HAL_PROXY_DAEMON_GIT_VER)\"
LOCAL_CFLAGS += -Wall -Werror
ifneq ($(TARGET_USES_AOSP),true)
LOCAL_CFLAGS += -DQTI_BSP=1
endif
ifneq ($(TARGET_USES_AOSP_FOR_WLAN), true)
LOCAL_CFLAGS += -DWCNSS_QTI_AOSP
endif
LOCAL_LDLIBS += -Lpthread
LOCAL_SANITIZE := integer_overflow

include $(BUILD_EXECUTABLE)

endif # Global WLAN enable check
