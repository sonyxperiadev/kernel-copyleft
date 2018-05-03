/*
*Copyright(C) 2011-2013 Foxconn International Holdings, Ltd. All rights reserved
*/

#include <linux/ioctl.h>

/* 
 * ioctl operation definition for ftm device driver
 * please sync../../../ftm/hal/fih_ftm_dev.h
 */

#define FIH_IOCTL_FTM_TYPE    'f'
#define FIH_IOCTL_GPIO_CFG      _IOWR(FIH_IOCTL_FTM_TYPE, 0x1, int)
#define FIH_IOCTL_GPIO_OUTV     _IOWR(FIH_IOCTL_FTM_TYPE, 0x2, int) //set output value
#define FIH_IOCTL_GPIO_INV      _IOWR(FIH_IOCTL_FTM_TYPE, 0x3, int) //get input value
//======================================================================================
#define FIH_IOCTL_HWID_ADC      _IOWR(FIH_IOCTL_FTM_TYPE, 0x8, int) //get HWID ADC value

