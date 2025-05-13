/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NFC Controller Driver
 * Copyright (C) 2020 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define ST21NFC_MAGIC 0xEA

#define ST21NFC_NAME "st21nfc"

#define ST54SPI_CB_RESET_END 0
#define ST54SPI_CB_RESET_START 1
#define ST54SPI_CB_ESE_NOT_USED 2
#define ST54SPI_CB_ESE_USED 3
void st21nfc_register_st54spi_cb(void (*cb)(int, void *), void *data);
void st21nfc_unregister_st54spi_cb(void);

#define ACCESS_OK(x, y, z) access_ok(y, z)

