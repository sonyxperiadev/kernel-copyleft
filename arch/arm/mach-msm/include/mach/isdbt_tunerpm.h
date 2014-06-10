/* arch/arm/mach-msm/include/mach/isdbt_tunerpm.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Tatsuya Ooka <Tatsuya.Ooka@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef _ARCH_ARM_MACH_MSM_ISDBT_TUNERPM_H_
#define _ARCH_ARM_MACH_MSM_ISDBT_TUNERPM_H_

#define D_TUNER_CONFIG_DRIVER_NAME "mmtuner"

extern int isdbt_tunerpm_init(void);
extern int isdbt_tunerpm_free(void);
extern int isdbt_tunerpm_power_control(int on);
extern int isdbt_tunerpm_reset_control(int on);
extern int isdbt_tunerpm_mpp1_control(int on);
extern int isdbt_tunerpm_mpp2_control(int on);
extern int isdbt_tunerpm_mpp3_control(int on);

#endif /* _ARCH_ARM_MACH_MSM_ISDBT_TUNERPM_H_ */
