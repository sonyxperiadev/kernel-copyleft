/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __QSEECOM_PRIV_H_
#define __QSEECOM_PRIV_H_

#include <linux/types.h>

#if IS_ENABLED(CONFIG_QSEECOM) || IS_ENABLED(CONFIG_ARCH_SA8155)

int qseecom_process_listener_from_smcinvoke(uint32_t *result,
                                        u64 *response_type, unsigned int *data);
#else
static inline int qseecom_process_listener_from_smcinvoke(uint32_t *result,
                                        u64 *response_type, unsigned int *data)
{
        return -EOPNOTSUPP;
}
#endif

int get_qseecom_kernel_fun_ops(void);

#endif
