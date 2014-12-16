/* arch/arm/mach-msm/include/mach/oneseg_tunerpm.h
 *
 * Copyright (C) 2010-2012 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _ARCH_ARM_MACH_MSM_ONESEG_TUNERPM_H_
#define _ARCH_ARM_MACH_MSM_ONESEG_TUNERPM_H_

#define D_ONESEG_TUNERPM_DRIVER_NAME	"oneseg_tunerpm"

struct oneseg_tunerpm_platform_data {
	int i2c_adapter_id;
	int (*init) (struct device *dev);
	int (*free) (struct device *dev);
	int (*reset_control) (struct device *dev, int on);
	int (*power_control) (struct device *dev, int on);
};

#endif /* _ARCH_ARM_MACH_MSM_ONESEG_TUNERPM_H_ */
