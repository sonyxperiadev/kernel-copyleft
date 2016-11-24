/*
 * Author: Yoshio Yamamoto yoshio.xa.yamamoto@sonymobile.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef _MSM_SONY_HWEFFECT_H
#define _MSM_SONY_HWEFFECT_H

#include <sound/sony-hweffect.h>

#define CP_MODE_1             0
#define CP_MODE_OFF           4
#define VPT_MODE_OFF          4

int msm_sony_hweffect_sonybundle_handler(struct audio_client *ac,
				     struct sonybundle_params *sb,
				     long *values);

void init_sonybundle_params(struct sonybundle_params *sb);

#endif /*_MSM_SONYEFFECT_HW_H*/
