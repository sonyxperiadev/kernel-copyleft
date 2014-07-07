/*
 * RMI4 bus driver.
 * include/linux/rmi4/rmi4_function_54.h
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2112 Sony Mobile Communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __RMI4_FUNCTION_54_H__
#define __RMI4_FUNCTION_54_H__

struct rmi4_function_54_platform_data {
	u8 num_rx_electrodes;
	u8 num_tx_electrodes;
};

#endif /* __RMI4_FUNCTION_54_H__ */
