/*
 * controller's Nordic Hal Service client helper
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <vendor/shadowcreator/hardware/nordic/1.0/INordic.h>
#include <hidl/Status.h>
#include <hidl/LegacySupport.h>
#include <utils/misc.h>
#include <hidl/HidlSupport.h>
#include <stdio.h>

using ::android::hardware::hidl_string;
using ::android::sp;
using vendor::shadowcreator::hardware::nordic::V1_0::INordic;
using android::hardware::hidl_string;
using android::hardware::hidl_handle;
using ::android::hidl::memory::V1_0::IMemory;

extern "C" {
int hal_nordic_client_start() {
    android::sp <INordic> service = INordic::getService();

    if (service == nullptr) {
        printf("Failed to get service\n");
        return -1;
    }
    int res = service->Nordic_Start();
    return res;
}
int hal_nordic_client_close() {
    android::sp <INordic> service = INordic::getService();

    if (service == nullptr) {
        printf("Failed to get service\n");
        return -1;
    }
    int res = service->Nordic_Stop();
    return res;
}

int hal_nordic_client_set_vibration(int32_t value) {
    android::sp <INordic> service = INordic::getService();

    if (service == nullptr) {
        printf("Failed to get service\n");
        return -1;
    }
    int res = service->Nordic_Set_Vibration(value);
    return res;
}

void hal_nordic_client_get_memory(int32_t &fd, int32_t &m_size) {
    android::sp <INordic> service = INordic::getService();

    if (service == nullptr) {
        printf("Failed to get service\n");
        fd = -1;
        m_size = 0;
        return;
    }
    service->Nordic_Get_Memory([&](hidl_handle _handle, int32_t _size) {
        fd = dup(_handle->data[0]);
        m_size = _size;
    });
}
}