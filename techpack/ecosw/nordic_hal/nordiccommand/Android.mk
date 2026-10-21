LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := NordicCommand.cpp

LOCAL_MODULE := libnordic_command
LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true
LOCAL_CFLAGS += -Wall
LOCAL_CFLAGS += -Wno-unused-parameter
LOCAL_INIT_RC := nordic.rc

LOCAL_C_INCLUDES := $(TOP)/system/memory/libion/include \
					$(TOP)/system/memory/libion/kernel-headers \
					$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES := libion liblog

include $(BUILD_SHARED_LIBRARY)

