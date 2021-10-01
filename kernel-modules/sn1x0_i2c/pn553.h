/*
 * Copyright (C) 2010 Trusted Logic S.A.
 * Copyright (C) 2020 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are licensed under the License.
 */
/******************************************************************************
 *
 *  The original Work has been changed by NXP Semiconductors.
 *
 *  Copyright (C) 2013-2020 NXP Semiconductors
 *   *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
#ifndef _PN553_H_
#define _PN553_H_

#include <linux/miscdevice.h>

#define PN544_MAGIC 0xE9

/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PN544_SET_PWR    _IOW(PN544_MAGIC, 0x01, long)

/*
 * SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 * This also be used to perform eSE cold reset when
 * argument value is 0x03
 */
#define P61_SET_SPI_PWR    _IOW(PN544_MAGIC, 0x02, long)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define P61_GET_PWR_STATUS    _IOR(PN544_MAGIC, 0x03, long)

/* DWP side this ioctl will be called
 * level 1 = Wired access is enabled/ongoing
 * level 0 = Wired access is disalbed/stopped
*/
#define P61_SET_WIRED_ACCESS _IOW(PN544_MAGIC, 0x04, long)

/*
  NFC Init will call the ioctl to register the PID with the i2c driver
*/
#define P544_SET_NFC_SERVICE_PID _IOW(PN544_MAGIC, 0x05, long)

/*
  NFC and SPI will call the ioctl to get the i2c/spi bus access
*/
#define P544_GET_ESE_ACCESS _IOW(PN544_MAGIC, 0x06, long)
/*
  NFC and SPI will call the ioctl to update the power scheme
*/
#define P544_SET_POWER_SCHEME _IOW(PN544_MAGIC, 0x07, long)

/*
  NFC will call the ioctl to release the svdd protection
*/
#define P544_REL_SVDD_WAIT _IOW(PN544_MAGIC, 0x08, long)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define PN544_SET_DWNLD_STATUS    _IOW(PN544_MAGIC, 0x09, long)
/*
  NFC will call the ioctl to release the dwp on/off protection
*/
#define P544_REL_DWPONOFF_WAIT _IOW(PN544_MAGIC, 0x0A, long)

/*
  NFC will call the ioctl to start Secure Timer
*/

#define P544_SECURE_TIMER_SESSION _IOW(PN544_MAGIC, 0x0B, long)

#define MAX_ESE_ACCESS_TIME_OUT_MS 200 /*100 milliseconds*/

/*
  NFC_ON: Driver is being used by the NFC service (bit b0)
*/
#define P544_FLAG_NFC_ON         0x01
/*
  FW_DNLD: NFC_ON and FW download is going on  (bit b1)
*/
#define P544_FLAG_FW_DNLD        0x02
/*
 * VEN_RESET: NFC_ON and FW download with VEN reset (bit b2)
*/
#define P544_FLAG_NFC_VEN_RESET  0x04
/*
 * ESE_RESET: Starting of flag positions for eSE cold reset origin
*/
#define ESE_COLD_RESET_ORIGIN_FLAGS_POS     (4) //(bit b4)
#define ESE_COLD_RESET_ORIGIN_NFC_FLAG_POS  (4) //(bit b4)
/*
 * ESE_RESET: Mask for the flags used for Driver to driver cold reset
 * b6, b5, b4 :
 * 0   0   0 -> no request for ese_cold_reset
 * 0   0   1 -> ese_cold_reset requested from NFC driver
 * 0   1   0 -> ese_cold_reset requested from eSE driver
 * 1   0   0 -> ese_cold_reset requested from UWB driver
*/
#define MASK_ESE_COLD_RESET             (0x70)
/*
 * ESE_RESET: Bit mask to check if ese_reset_guard timer is started (bit b7)
*/
#define MASK_ESE_COLD_RESET_GUARD_TIMER  (0x80)
/*
 * ESE_RESET: Guard time to allow eSE cold reset from the driver
*/
#define ESE_COLD_RESET_GUARD_TIME        (3000) //3s
/*
 * ESE_RESET: NCI command response timeout
*/
#define NCI_CMD_RSP_TIMEOUT              (2000) //2s
/*
 * ESE_RESET: Guard time to reboot the JCOP
*/
#define ESE_COLD_RESET_REBOOT_GUARD_TIME   (50) //50ms

typedef enum p61_access_state{
    P61_STATE_INVALID = 0x0000,
    P61_STATE_IDLE = 0x0100, /* p61 is free to use */
    P61_STATE_WIRED = 0x0200,  /* p61 is being accessed by DWP (NFCC)*/
    P61_STATE_SPI = 0x0400, /* P61 is being accessed by SPI */
    P61_STATE_DWNLD = 0x0800, /* NFCC fw download is in progress */
    P61_STATE_SPI_PRIO = 0x1000, /*Start of p61 access by SPI on priority*/
    P61_STATE_SPI_PRIO_END = 0x2000, /*End of p61 access by SPI on priority*/
    P61_STATE_SPI_END = 0x4000,
    P61_STATE_JCP_DWNLD = 0x8000,/* JCOP downlad in progress */
    P61_STATE_SECURE_MODE = 0x100000, /* secure mode state*/
    P61_STATE_SPI_SVDD_SYNC_START = 0x0001, /*ESE_VDD Low req by SPI*/
    P61_STATE_SPI_SVDD_SYNC_END = 0x0002, /*ESE_VDD is Low by SPI*/
    P61_STATE_DWP_SVDD_SYNC_START = 0x0004, /*ESE_VDD  Low req by Nfc*/
    P61_STATE_DWP_SVDD_SYNC_END = 0x0008 /*ESE_VDD is Low by Nfc*/
}p61_access_state_t;

typedef enum chip_type_pwr_scheme{
    PN67T_PWR_SCHEME = 0x01,
    PN80T_LEGACY_PWR_SCHEME,
    PN80T_EXT_PMU_SCHEME,
}chip_pwr_scheme_t;

typedef enum jcop_dwnld_state{
    JCP_DWNLD_IDLE = P61_STATE_JCP_DWNLD,   /* jcop dwnld is ongoing*/
    JCP_DWNLD_INIT=0x8010,                         /* jcop dwonload init state*/
    JCP_DWNLD_START=0x8020,                        /* download started */
    JCP_SPI_DWNLD_COMPLETE=0x8040,                 /* jcop download complete in spi interface*/
    JCP_DWP_DWNLD_COMPLETE=0x8080,                 /* jcop download complete */
} jcop_dwnld_state_t;

struct pn544_i2c_platform_data {
    unsigned int irq_gpio;
    unsigned int ven_gpio;
    unsigned int firm_gpio;
    unsigned int ese_pwr_gpio; /* gpio to give power to p61, only TEE should use this */
    unsigned int iso_rst_gpio; /* gpio used for ISO hard reset P73*/
};

struct hw_type_info {
    /*
     * Response of get_version_cmd will be stored in data
     * byte structure :
     * byte 0-1     : Header
     * byte 2       : Status
     * byte 3       : Hardware Version
     * byte 4       : ROM code
     * byte 5       : 0x00 constant
     * byte 6-7     : Protected data version
     * byte 8-9     : Trim data version
     * byte 10-11   : FW version
     * byte 12-13   : CRC
     * */
    char data[20];
    int len;
};

#define NEXUS5x    1
#define HWINFO     0
#if NEXUS5x
#undef ISO_RST
#undef ESE_PWR
#else
#define ISO_RST
#endif

struct pn544_dev    {
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct i2c_client   *client;
    struct miscdevice   pn544_device;
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
    unsigned int        ese_pwr_gpio; /* gpio used by SPI to provide power to p61 via NFCC */
#ifdef ISO_RST
    unsigned int        iso_rst_gpio; /* ISO-RST pin gpio*/
#endif
    struct mutex        p61_state_mutex; /* used to make p61_current_state flag secure */
    p61_access_state_t  p61_current_state; /* stores the current P61 state */
    bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */
    bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
    bool                irq_enabled;
    spinlock_t          irq_enabled_lock;
    long                nfc_service_pid; /*used to signal the nfc the nfc service */
    chip_pwr_scheme_t   chip_pwr_scheme;
    unsigned int        secure_timer_cnt;
    struct workqueue_struct *pSecureTimerCbWq;
    struct work_struct wq_task;
    /* This byte represents different flags used during eSE cold reset request from
     * Driver to driver
     * Bit value  Status           Remark
     * b0 : 1  -> NFC_ON           Driver Open should set the flag
     *      0     NFC_OFF          Driver release should reset this flag
     * b1 : 1  -> FWDNLD           If FWDNLD is going on.
     *      0     Normal operation
     * b2 : 1 --> Ven reset has been requested
     * b3 : reserved bit
     * b6, b5, b4 :
     * 0   0   0 -> no request for ese_cold_reset
     * 0   0   1 -> ese_cold_reset requested from NFC driver
     * 0   1   0 -> ese_cold_reset requested from eSE driver
     * 0   1   1 -> ese_cold_reset requested from UWB driver
     *              Remaining combinations: Reserved for future use.
     *              These bits will be cleared once cold reset rsp is received.
     * b7 : 1 -->   The ese_cold reset guard time has is running
     *              It will be cleared by the Guard Timer Callback
     * */
    volatile uint8_t    state_flags;
};

#endif
