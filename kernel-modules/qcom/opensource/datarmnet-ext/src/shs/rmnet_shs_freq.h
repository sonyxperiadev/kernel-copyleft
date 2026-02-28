/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _RMNET_SHS_FREQ_H_
#define _RMNET_SHS_FREQ_H_

int rmnet_shs_freq_init(void);
int rmnet_shs_freq_exit(void);
void rmnet_shs_boost_cpus(void);
void rmnet_shs_reset_cpus(void);
void rmnet_shs_pb_boost_cpus(void);
void rmnet_shs_pb_reset_cpus(void);

void rmnet_shs_boost_gold_cpu(int cpu);

#endif
