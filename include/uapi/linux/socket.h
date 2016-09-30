#ifndef _UAPI_LINUX_SOCKET_H
#define _UAPI_LINUX_SOCKET_H

#include <linux/types.h>

/*
 * Desired design of maximum size and alignment (see RFC2553)
 */
#define _K_SS_MAXSIZE	128	/* Implementation specific max size */
#define _K_SS_ALIGNSIZE	(__alignof__ (struct sockaddr *))
				/* Implementation specific desired alignment */

typedef unsigned short __kernel_sa_family_t;

struct __kernel_sockaddr_storage {
	__kernel_sa_family_t	ss_family;		/* address family */
	/* Following field(s) are implementation specific */
	char		__data[_K_SS_MAXSIZE - sizeof(unsigned short)];
				/* space to achieve desired size, */
				/* _SS_MAXSIZE value minus size of ss_family */
} __attribute__ ((aligned(_K_SS_ALIGNSIZE)));	/* force desired alignment */

struct sock_sizehint {
	__u32	order_zero_size;
		/* max size that can fit into one page in kernel */
	__u32	cache_size;
		/* max size that can fit in socket cache */
};


#endif /* _UAPI_LINUX_SOCKET_H */
