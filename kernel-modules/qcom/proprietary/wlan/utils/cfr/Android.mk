ifneq ($(TARGET_BUILD_VARIANT), user)
LOCAL_PATH := $(call my-dir)
MY_LOCAL_PATH := $(LOCAL_PATH)

include $(CLEAR_VARS)
LOCAL_CLANG := true
LOCAL_PROPRIETARY_MODULE := true

LOCAL_MODULE := wificfrtool

LOCAL_SRC_FILES:= \
    cfrtool.c

LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := \
      libbase \
      liblog \
      libutils \
      libnl \
      libc

ifeq ($(filter $(PLATFORM_VERSION),14 UpsideDownCake),$(PLATFORM_VERSION))
LOCAL_SHARED_LIBRARIES += \
    vendor.qti.hardware.wifi.wificfr-V1-ndk \
    libbinder_ndk
LOCAL_SRC_FILES += \
    aidl/aidl.cpp \
    aidl/wificfr.cpp
LOCAL_CFLAGS := -DCONFIG_WIFICFR_USE_AIDL
else
LOCAL_SHARED_LIBRARIES += \
    libhidlbase \
    vendor.qti.hardware.wifi.wificfr@1.0
LOCAL_SRC_FILES += \
    hidl/1.0/wificfr.cpp \
    hidl/1.0/hidl.cpp
endif

LOCAL_MODULE_OWNER := qti
LOCAL_SANITIZE := integer_overflow
ifeq ($(filter $(PLATFORM_VERSION),14 UpsideDownCake),$(PLATFORM_VERSION))
LOCAL_INIT_RC := vendor.qti.hardware.wifi.wificfr-service.rc
LOCAL_VINTF_FRAGMENTS := vendor.qti.hardware.wifi.wificfr-service.xml
else
LOCAL_INIT_RC := vendor.qti.hardware.wifi.wificfr@1.0-service.rc
LOCAL_VINTF_FRAGMENTS := vendor.qti.hardware.wifi.wificfr@1.0-service.xml
endif

include $(BUILD_EXECUTABLE)

endif
