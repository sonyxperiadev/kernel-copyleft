/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* Copyright (C) 2012 Sony Mobile Communications AB.
*
* File Name		: l3gd20_gyr.h
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Neil Gao (neil.gao@sonymobile.com)
* Version		: V 1.01 sysfs
* Date			: 19/11/2010
* Description		: L3GD20 digital output gyroscope sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
********************************************************************************
* REVISON HISTORY
*
* VERSION | DATE        | AUTHORS            | DESCRIPTION
*
* 1.0     | 19/11/2010  | Carmine Iascone    | First Release
* 1.01    | 07/08/2012  | Neil Gao           | Modify ODR etc.
*
*******************************************************************************/

#ifndef __L3GD20_GYR_H__
#define __L3GD20_GYR_H__
#include <linux/device.h>
#include <linux/kernel.h>

#define L3GD20_DEV_NAME	"l3gd20_gyr"

#define L3GD20_FS_250DPS	0x00
#define L3GD20_FS_500DPS	0x10
#define L3GD20_FS_2000DPS	0x20

#define L3GD20_ENABLED	1
#define L3GD20_DISABLED	0

#ifdef __KERNEL__
struct l3gd20_gyr_platform_data {
	int poll_interval;
	int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(struct device *dev);
	int (*power_off)(struct device *dev);
	int (*power_config)(struct device *dev, bool value);
};
#endif /* __KERNEL__ */

#endif  /* __L3GD20_GYR_H__ */
