/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPA_TSP_H_
#define _IPA_TSP_H_

#include <linux/msm_ipa.h>
/* The following line should be removed once TSP feature is POR */
#include "ipa_test_module_tsp.h"

int ipa_tsp_init(void);
int ipa_tsp_commit(void);
int ipa_tsp_reset(void);
int ipa_tsp_get_ingr_tc(u8 index, struct ipa_ioc_tsp_ingress_class_params *output);
int ipa_tsp_get_egr_ep(u8 index, struct ipa_ioc_tsp_egress_prod_params *output);
int ipa_tsp_get_egr_tc(u8 index, struct ipa_ioc_tsp_egress_class_params *output);
int ipa_tsp_set_ingr_tc(u8 index, const struct ipa_ioc_tsp_ingress_class_params *input);
int ipa_tsp_set_egr_ep(u8 index, const struct ipa_ioc_tsp_egress_prod_params *input);
int ipa_tsp_set_egr_tc(u8 index, const struct ipa_ioc_tsp_egress_class_params *input);

#endif /* _IPA_TSP_H_ */
