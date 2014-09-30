/* arch/arm/mach-msm/include/mach/isdbt_tunerpm.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef _ARCH_ARM_MACH_MSM_ISDBT_TUNERPM_H_
#define _ARCH_ARM_MACH_MSM_ISDBT_TUNERPM_H_

#define D_TUNER_CONFIG_DRIVER_NAME "mmtuner_pdev"

enum ANTMODE {
	ANTMODE_WHIP,
	ANTMODE_EARPHONE,
	ANTMODE_USB,
	ANTMODE_CHARGER,
	ANTMODE_AUTO,
	ANTMODE_NOTUSE
};

struct isdbt_tunerpm_platform_data {
	int i2c_adapter_id;
	int (*init) (struct device *dev);
	int (*free) (struct device *dev);
	int (*reset_control) (struct device *dev, int on);
	int (*power_control) (struct device *dev, int on);
	int (*ant_switch) (struct device *dev, int ant_mode);
};

#endif /* _ARCH_ARM_MACH_MSM_ISDBT_TUNERPM_H_ */
