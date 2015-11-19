/*
 *      Copyright (c) INSIDE Secure Oy 2011, 2012, 2013, 2014
 *      All Rights Reserved
 *
 *      This software is open source; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This General Public License does NOT permit incorporating this software
 *      into proprietary programs.  If you are unable to comply with the GPL, a
 *      commercial license for this software may be purchased from INSIDE
 *      Secure at
 *      www.insidesecure.com/Produits/Security-Solutions-for-Android/MatrixDAR
 *
 *      This program is distributed in WITHOUT ANY WARRANTY; without even the
 *      implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *      See the GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *      http://www.gnu.org/copyleft/gpl.html
 */
/******************************************************************************/

#ifndef _CRYPTO_KFIPS_H
#define _CRYPTO_KFIPS_H

#ifdef __KERNEL__
#include "linux/threads.h"
#if defined NR_CPUS
#ifndef KFIPS_MAX_WORKERS
#define KFIPS_MAX_WORKERS NR_CPUS /* At most max CPUs. */
#endif /* KFIPS_MAX_WORKERS */
#else
#ifndef KFIPS_MAX_WORKERS
#define KFIPS_MAX_WORKERS 1 /* The default. */
#endif /* KFIPS_MAX_WORKERS */
#endif /* Kernel space KFIPS_MAX_WORKERS. */
#endif /* Number of workers. */

#define KFIPS_PROC_NAME "driver/kfips"
#define KFIPS_PROC_PATH "/proc/" KFIPS_PROC_NAME

#ifndef KFIPS_BUFFER_SIZE
#define KFIPS_BUFFER_SIZE 512 /* Optimize for buffer sizes >= 512 bytes. */
#endif /* KFIPS_BUFFER_SIZE */

#define KFIPS_AES_MAX_KEY_SIZE 32
#define KFIPS_AES_BLOCK_SIZE 16

#define KFIPS_PENDING_LEN 128

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

#ifdef __ANDROID__
/* Use "BAD" version of _IOWR for Android to avoid
   undefined __invalid_size_argument_for_IOC. */
#define KFIPS_QUEUE_IOCTL(szt) _IOWR_BAD('q', 0x1, szt)
#else /* __ANDROID__ */
#define KFIPS_QUEUE_IOCTL(szt) _IOWR('q', 0x1, szt)
#endif /* __ANDROID__ */

#ifdef __KERNEL__
#include <crypto/aes.h>
#include <linux/device-mapper.h>
#if KFIPS_AES_MAX_KEY_SIZE != AES_MAX_KEY_SIZE
#error bad KFIPS_AES_MAX_KEY_SIZE
#endif
#if KFIPS_AES_BLOCK_SIZE != AES_BLOCK_SIZE
#error bad KFIPS_AES_BLOCK_SIZE
#endif
#endif

/* Values for combined_valid. */
#define REQUEST_VALID 1
#define RESPONSE_VALID 2

/* Control part of request (key, iv and lengths) */
struct kfips_ctrl {
	/* two keys for XTS, one for ECB and CBC */
	uint8_t key[2 * KFIPS_AES_MAX_KEY_SIZE];
	/* initialization vector */
	uint8_t iv[KFIPS_AES_BLOCK_SIZE];
	/* length of data */
	uint32_t len;
	/* key length in bytes */
	uint32_t keylen;
};

/* Individual KFIPS Operation (control + bookkeeping) */
struct kfips_req {
	/* Operation and direction. */
	uint32_t flags;         /* Command/Status */
	uint32_t context;       /* For book keeping.
				   User-space: do not touch. */
	struct kfips_ctrl ctrl; /* Input only. */
};

#define KFIPS_FLAGS_BLANK       0x80000000U /* Block is not filled. */
#define KFIPS_FLAGS_SEND        0x40000000U /* Block is finished. */
#define KFIPS_FLAGS_ERR         0x20000000U /* Block cannot be finished. */
#define KFIPS_FLAGS_BUSY        0x10000000U /* Long queue. */
#define KFIPS_FLAGS_QUEUE_MASK  0xF0000000U /* Reserved for queue's
					       operation. */

/* individual operation structure (used in size estimation only) */
struct kfips_msg_estimate {
	/* kernel-userland synchronization fields */
	uint32_t combined_valid;
	/* two keys for XTS, one for ECB and CBC */
	uint8_t key[2 * KFIPS_AES_MAX_KEY_SIZE];
	/* initialization vector */
	uint8_t iv[KFIPS_AES_BLOCK_SIZE];
	/* data to encrypt/decrypt */
	uint8_t buf[KFIPS_BUFFER_SIZE];
	/* length of data */
	int len;
	/* operation flags */
	uint32_t flags;
	/* key length in bytes */
	uint32_t keylen;
	/* context pointer */
	void *pointer;
};

/* Flags for kfips (encryption operation and mode.) */
#define KFIPS_FLAGS_DECRYPT 0x01
#define KFIPS_FLAGS_ENCRYPT 0x02
#define KFIPS_FLAGS_ECB 0x04
#define KFIPS_FLAGS_CBC 0x08
#define KFIPS_FLAGS_XTS 0x10

/* operation ring or entry buffer */
#ifndef KFIPS_RING_INDEX_BITS
#if KFIPS_BUFFER_SIZE <= 512
#define KFIPS_RING_INDEX_BITS 5 /* 512 bytes blocks */
#else
#define KFIPS_RING_INDEX_BITS 2 /* 4096 bytes blocks. */
#endif /* KFIPS_BUFFER_SIZE <= 512 */
#endif /* KFIPS_RING_INDEX_BITS */
#define KFIPS_RING_ENTRIES (1 << KFIPS_RING_INDEX_BITS)
#define KFIPS_RING_INDEX_MASK (KFIPS_RING_ENTRIES - 1)

#define KFIPS_RING_ENTRIES_SIZE_MATH \
	(16383 / sizeof(struct kfips_msg_estimate))

#define KFIPS_RING_ENTRIES_SAFE \
	((KFIPS_RING_ENTRIES < KFIPS_RING_ENTRIES_SIZE_MATH) ?	\
	 KFIPS_RING_ENTRIES :					\
	 KFIPS_RING_ENTRIES_SIZE_MATH)

#define KFIPS_DATA_SIZE (KFIPS_BUFFER_SIZE*KFIPS_RING_ENTRIES_SAFE)

/* Select combination of requests and data area size, which fits
   inside IOCTL. */
struct kfips_reqs_and_data {
	struct kfips_req reqs[KFIPS_RING_ENTRIES_SAFE];
	uint8_t          data[KFIPS_DATA_SIZE];
};

/* Ensure structure size is below maximum. */
extern int kfips_reqs_and_data_sizecheck_t[
	sizeof(struct kfips_reqs_and_data) > 16383 ? -1 : 1];

/* KFIPS_Key ID size used by UFIPS and FIPS Lib */
#define KFIPS_KEY_ID_SIZE ((unsigned int)sizeof(uint32_t))

#endif /* _CRYPTO_KFIPS_H */
