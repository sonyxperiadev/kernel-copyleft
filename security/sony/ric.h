/*
 * security/sony/ric.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2013 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __RIC_H_
#define __RIC_H_

extern int sony_ric_mount(const char *dev_name, struct path *path,
			  const char *type, unsigned long flags, void *data);
extern int sony_ric_enabled(void);

#endif /*__RIC_H_*/
