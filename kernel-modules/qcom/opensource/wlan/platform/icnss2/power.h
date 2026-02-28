/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __ICNSS_POWER_H__
#define __ICNSS_POWER_H__

int icnss_hw_power_on(struct icnss_priv *priv);
int icnss_hw_power_off(struct icnss_priv *priv);
int icnss_power_on_chain1_reg(struct icnss_priv *priv);
int icnss_get_clk(struct icnss_priv *priv);
int icnss_get_vreg(struct icnss_priv *priv);
void icnss_put_resources(struct icnss_priv *priv);
void icnss_put_vreg(struct icnss_priv *priv);
void icnss_put_clk(struct icnss_priv *priv);
int icnss_vreg_unvote(struct icnss_priv *priv);
int icnss_get_psf_info(struct icnss_priv *priv);
#endif
