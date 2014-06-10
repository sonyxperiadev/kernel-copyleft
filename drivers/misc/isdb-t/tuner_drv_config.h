/******************************************************************************
 *
 *  file name       : tuner_drv_config.h
 *  brief note      : Driver Config Header
 *
 *  creation data   : 2011.08.28
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 322                        $ Revision of Last commit
 *  $Date:: 2011-10-26 13:33:02 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *              Copyright (C) 2012 Sony Mobile Communications AB.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/08/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/
#ifndef _TUNER_DRV_CONFIG_H
#define _TUNER_DRV_CONFIG_H

#define TUNER_SET_ON                     1       /* Enable setting           */
#define TUNER_SET_OFF                    0       /* Disable setting          */

/* Driver name */
#define TUNER_CONFIG_DRIVER_NAME     "mmtuner_drv"

/* Device file setting */
#define TUNER_CONFIG_DRV_MAJOR         100       /* MAJOR number setting     */
#define TUNER_CONFIG_DRV_MINOR         200       /* MINOR number setting     */

/* Interrupt number */
#define TUNER_CONFIG_INT              0x07       /* IRQ number               */

/* I2C bus number */
#define TUNER_CONFIG_I2C_BUSNUM       0          /* I2C bus number           */

/* Kernel thread priority  */
#define TUNER_CONFIG_KTH_PRI          95         /* Priority (0-99)         */

/* Interrupt triger setting */
#define TUNER_CONFIG_IRQ_LEVEL  TUNER_SET_ON     /* Interrupt triger = level */

#endif/* _TUNER_DRV_CONFIG_H */
