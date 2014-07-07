 /* include/linux/if_pppocpw.h
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __LINUX_IF_PPPOCPW_H
#define __LINUX_IF_PPPOCPW_H

#include <linux/socket.h>
#include <linux/types.h>

struct sockaddr_pppocpw {
	sa_family_t	sa_family;	/* AF_PPPOX */
	unsigned int	sa_protocol;	/* PX_PROTO_OLAC */
	int			udp_socket;
	__u32		stream_id;
} __packed;

#endif /* __LINUX_IF_PPPOLAC_H */
