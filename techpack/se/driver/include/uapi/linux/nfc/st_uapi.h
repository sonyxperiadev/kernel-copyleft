/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 ****************************************************************************/

#ifndef _UAPI_ST_UAPI_H_
#define _UAPI_ST_UAPI_H_

/*
 * ST21NFC power control via ioctl
 * ST21NFC_GET_WAKEUP :  poll gpio-level for Wakeup pin
 */
#define ST21NFC_GET_WAKEUP _IO(ST21NFC_MAGIC, 0x01)
#define ST21NFC_PULSE_RESET _IO(ST21NFC_MAGIC, 0x02)
#define ST21NFC_SET_POLARITY_RISING _IO(ST21NFC_MAGIC, 0x03)
#define ST21NFC_SET_POLARITY_HIGH _IO(ST21NFC_MAGIC, 0x05)
#define ST21NFC_GET_POLARITY _IO(ST21NFC_MAGIC, 0x07)
#define ST21NFC_RECOVERY _IO(ST21NFC_MAGIC, 0x08)
#define ST21NFC_USE_ESE _IOW(ST21NFC_MAGIC, 0x09, unsigned int)

#define NFC_SECURE_ZONE _IO(ST21NFC_MAGIC, 0x0B)

// Keep compatibility with older user applications.
#define ST21NFC_LEGACY_GET_WAKEUP _IOR(ST21NFC_MAGIC, 0x01, unsigned int)
#define ST21NFC_LEGACY_PULSE_RESET _IOR(ST21NFC_MAGIC, 0x02, unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_RISING                                     \
	_IOR(ST21NFC_MAGIC, 0x03, unsigned int)
#define ST21NFC_LEGACY_SET_POLARITY_HIGH _IOR(ST21NFC_MAGIC, 0x05, unsigned int)
#define ST21NFC_LEGACY_GET_POLARITY _IOR(ST21NFC_MAGIC, 0x07, unsigned int)
#define ST21NFC_LEGACY_RECOVERY _IOR(ST21NFC_MAGIC, 0x08, unsigned int)

#endif
