
/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by Sony Mobile Communications AB.
 * Modifications are licensed under the License.
 */
#ifndef __MHL_API_H__
#define __MHL_API_H__

#if defined(CONFIG_FB_MSM_HDMI_MHL_8334) || defined(CONFIG_FB_MSM_MHL_SII8334)
bool mhl_is_connected(void);
#else
static bool mhl_is_connected(void)
{
	return false;
}
#endif

#endif /* __MHL_API_H__ */
