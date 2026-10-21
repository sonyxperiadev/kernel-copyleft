// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#include "ipa_tsp.h"
#include "ipahal_i.h"
#include "ipahal_tsp_i.h"
#include "ipahal_reg.h"
#include "ipahal_tsp.h"


void ipahal_tsp_fill_hw_ingr_tc(const struct ipa_ioc_tsp_ingress_class_params *input,
	void *table, u8 index)
{
	/* The first index is 1 */
	struct ipahal_tsp_ingress_class *hal_ingr_tc =
		(struct ipahal_tsp_ingress_class *)table + index - 1;

	hal_ingr_tc->max_rate = input->max_rate;
	hal_ingr_tc->max_burst = input->max_burst;
	hal_ingr_tc->max_bucket = input->max_burst;
	hal_ingr_tc->last_rtc = 0x0;
}

void ipahal_tsp_fill_hw_egr_ep(const struct ipa_ioc_tsp_egress_prod_params *input,
	void *table, u8 index)
{
	union ipahal_tsp_egress_prod *hal_egr_ep =
		(union ipahal_tsp_egress_prod *)table + index;

	if ((index & 0x1)) {
		/* even index */
		hal_egr_ep->even.max_rate = input->max_rate;
		hal_egr_ep->even.max_freq = 65536 / input->max_rate;
		hal_egr_ep->even.max_burst = input->max_burst;
		hal_egr_ep->even.max_bucket = input->max_burst;
		hal_egr_ep->even.last_rtc = 0x0;
	} else {
		/* odd index */
		hal_egr_ep->odd.max_rate = input->max_rate;
		hal_egr_ep->odd.max_freq = 65536 / input->max_rate;
		hal_egr_ep->odd.max_burst = input->max_burst;
		hal_egr_ep->odd.max_bucket = input->max_burst;
		hal_egr_ep->odd.last_rtc = 0x0;
	}
}

void ipahal_tsp_fill_hw_egr_tc(const struct ipa_ioc_tsp_egress_class_params *input,
	void *table, u8 index)
{
	/* The first index is 1 */
	union ipahal_tsp_egress_class *hal_egr_tc =
		(union ipahal_tsp_egress_class *)table + index - 1;

	if (!(index & 0x1)) {
		/* even index */
		hal_egr_tc->even.guaranteed_rate = input->guaranteed_rate;
		hal_egr_tc->even.max_rate = input->max_rate;
		hal_egr_tc->even.guaranteed_freq = 65536 / input->guaranteed_rate;
		hal_egr_tc->even.max_freq = 65536 / input->max_rate;
		hal_egr_tc->even.guaranteed_burst = input->guaranteed_burst;
		hal_egr_tc->even.max_burst = input->max_burst;
		hal_egr_tc->even.max_bucket = input->max_burst;
		hal_egr_tc->even.last_rtc = 0x0;
	} else {
		/* odd index */
		hal_egr_tc->odd.guaranteed_rate = input->guaranteed_rate;
		hal_egr_tc->odd.max_rate = input->max_rate;
		hal_egr_tc->odd.guaranteed_freq = 65536 / input->guaranteed_rate;
		hal_egr_tc->odd.max_freq = 65536 / input->max_rate;
		hal_egr_tc->odd.guaranteed_burst = input->guaranteed_burst;
		hal_egr_tc->odd.max_burst = input->max_burst;
		hal_egr_tc->odd.max_bucket = input->max_burst;
		hal_egr_tc->odd.last_rtc = 0x0;
	}
}

void ipahal_tsp_parse_hw_ingr_tc(const void *table, u8 index,
	struct ipa_ioc_tsp_ingress_class_params *output)
{
	/* The first index is 1 */
	struct ipahal_tsp_ingress_class *hal_ingr_tc =
		(struct ipahal_tsp_ingress_class *)table + index - 1;

	output->max_rate = hal_ingr_tc->max_rate;
	output->max_burst = hal_ingr_tc->max_burst;
}

void ipahal_tsp_parse_hw_egr_tc(const void *table, u8 index,
	struct ipa_ioc_tsp_egress_class_params *output)
{
	/* The first index is 1 */
	union ipahal_tsp_egress_class *hal_egr_tc =
		(union ipahal_tsp_egress_class *)table + index - 1;

	if (!(index & 0x1)) {
		/* even index */
		output->guaranteed_rate = hal_egr_tc->even.guaranteed_rate;
		output->max_rate = hal_egr_tc->even.max_rate;
		output->guaranteed_burst = hal_egr_tc->even.guaranteed_burst;
		output->max_burst = hal_egr_tc->even.max_burst;
	} else {
		/* odd index */
		output->guaranteed_rate = hal_egr_tc->odd.guaranteed_rate;
		output->max_rate = hal_egr_tc->odd.max_rate;
		output->guaranteed_burst = hal_egr_tc->odd.guaranteed_burst;
		output->max_burst = hal_egr_tc->odd.max_burst;
	}
}

void ipahal_tsp_parse_hw_egr_ep(const void *table, u8 index,
	struct ipa_ioc_tsp_egress_prod_params *output)
{
	union ipahal_tsp_egress_prod *hal_egr_ep =
		(union ipahal_tsp_egress_prod *)table + index;

	if ((index & 0x1)) {
		/* even index */
		output->max_rate = hal_egr_ep->even.max_rate;
		output->max_burst = hal_egr_ep->even.max_burst;
	} else {
		/* odd index */
		output->max_rate = hal_egr_ep->odd.max_rate;
		output->max_burst = hal_egr_ep->odd.max_burst;
	}
}
