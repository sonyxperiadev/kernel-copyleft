/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef HIDL_H
#define HIDL_H

#ifdef __cplusplus
extern "C"
{
#endif  // _cplusplus
//#include "wificsi.h"

int wificfr_hidl_process();
void wificfr_hidl_notify_cfrdata(uint8_t* read_buf, int length);

#ifdef __cplusplus
}
#endif  // _cplusplus

#endif  // HIDL_H
