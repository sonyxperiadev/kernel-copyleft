/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPAHAL_TSP_H_
#define _IPAHAL_TSP_H_

#include <linux/msm_ipa.h>
#include "ipa_tsp.h"

#define IPA_TSP_IN_TC_NUM 16 // TBD: Get rid of it

#define IPA_TSP_INGR_TC_SIZE 8
#define IPA_TSP_EGR_EP_SIZE 12
#define IPA_TSP_EGR_TC_SIZE 20


void ipahal_tsp_fill_hw_ingr_tc(const struct ipa_ioc_tsp_ingress_class_params *input,
	void *table, u8 index);
void ipahal_tsp_fill_hw_egr_ep(const struct ipa_ioc_tsp_egress_prod_params *input,
	void *table, u8 index);
void ipahal_tsp_fill_hw_egr_tc(const struct ipa_ioc_tsp_egress_class_params *input,
	void *table, u8 index);
void ipahal_tsp_parse_hw_ingr_tc(const void *table, u8 index,
	struct ipa_ioc_tsp_ingress_class_params *output);
void ipahal_tsp_parse_hw_egr_ep(const void *table, u8 index,
	struct ipa_ioc_tsp_egress_prod_params *output);
void ipahal_tsp_parse_hw_egr_tc(const void *table, u8 index,
	struct ipa_ioc_tsp_egress_class_params *output);

#endif /* _IPAHAL_TSP_H_ */
