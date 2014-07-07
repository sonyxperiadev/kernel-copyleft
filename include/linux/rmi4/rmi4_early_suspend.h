/*
 * RMI4 early suspend handler.
 * include/linux/rmi4/rmi4_early_suspend.h
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2012 Sony Mobile communications AB
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

#ifndef __RMI4_EARLY_SUSPEND__
#define __RMI4_EARLY_SUSPEND__

/* Sicne all chips can and will have different FW, the early
 * suspend/resume functionality implemented by Android may
 * require different behavior. To account for this, we will
 * be able to create different early suspend handlers for
 * different chips. Here, we give them all an individual
 * module name.
 */

/* Synaptics 3250 Incell touch */
#define RMI4_E_SUSP_NAME	"rmi4_early_suspend"

#endif /* __RMI4_3250_EARLY_SUSPEND__ */
