/**
 * @file security_status.h
 *
 * @brief Gets security status of device by
 *		checking oemandroidboot.securityflags
 *
 * @author Nandhakumar Rangasamy (nandhakumar.x.rangasamy@sonymobile.com)
 */
/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __SECURITY_STATUS_H
#define __SECURITY_STATUS_H

#define SECURITY_ON 1
#define SECURITY_OFF 0

#ifdef CONFIG_SECURITY_STATUS
int get_security_status(int *status);
#else
static inline int get_security_status(int *status)
{
	return -ENOTSUP;
}
#endif /* CONFIG_SECURITY_STATUS */
#endif /* __SECURITY_STATUS_H */
