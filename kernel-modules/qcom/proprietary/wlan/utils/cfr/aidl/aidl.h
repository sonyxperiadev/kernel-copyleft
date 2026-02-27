/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef AIDL_H
#define AIDL_H

#ifdef __cplusplus
extern "C"
{
#endif  // _cplusplus

int wificfr_hidl_process();
void wificfr_hidl_notify_cfrdata(uint8_t* read_buf, int length);
#ifdef __cplusplus
}
#endif  // _cplusplus

#endif  // AIDL_H
