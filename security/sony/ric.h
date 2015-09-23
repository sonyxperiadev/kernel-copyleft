/*
 * security/sony/ric.h
 *
 * Copyright (c) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __RIC_H_
#define __RIC_H_

extern int sony_ric_mount(char *dev_name, struct path *path,
			  char *type, unsigned long flags, void *data);
extern int sony_ric_enabled(void);

#endif /*__RIC_H_*/
