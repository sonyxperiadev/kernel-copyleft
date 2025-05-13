/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __WCD939X_MBHC_H__
#define __WCD939X_MBHC_H__
#include <asoc/wcd-mbhc-v2.h>

struct wcd939x_mbhc {
	struct wcd_mbhc wcd_mbhc;
	struct blocking_notifier_head notifier;
	struct fw_info *fw_data;
	struct timer_list rdown_timer;
	int rdown_prev_iter;
	bool rdown_timer_complete;
};

static inline u32 get_r_gnd_res_tot_mohms(u32 r_gnd_int_fet_mohms, u32 r_gnd_ext_fet_mohms,
					  u32 r_gnd_par_tot_mohms)
{
	return r_gnd_int_fet_mohms + r_gnd_ext_fet_mohms + r_gnd_par_tot_mohms;
}

static inline u32 get_r_aud_res_tot_mohms(u32 r_aud_int_fet_mohms, u32 r_aud_ext_fet_mohms,
					  u32 r_load_eff_mohms)
{
	return r_aud_int_fet_mohms + r_aud_ext_fet_mohms + r_load_eff_mohms;
}

#if IS_ENABLED(CONFIG_SND_SOC_WCD939X)
extern int wcd939x_mbhc_init(struct wcd939x_mbhc **mbhc,
			   struct snd_soc_component *component,
			   struct fw_info *fw_data);
extern void wcd939x_mbhc_hs_detect_exit(struct snd_soc_component *component);
extern int wcd939x_mbhc_hs_detect(struct snd_soc_component *component,
				struct wcd_mbhc_config *mbhc_cfg);
extern void wcd939x_mbhc_deinit(struct snd_soc_component *component);
extern void wcd939x_mbhc_ssr_down(struct wcd939x_mbhc *mbhc,
				struct snd_soc_component *component);
extern int wcd939x_mbhc_post_ssr_init(struct wcd939x_mbhc *mbhc,
				    struct snd_soc_component *component);
extern int wcd939x_mbhc_get_impedance(struct wcd939x_mbhc *wcd939x_mbhc,
				    uint32_t *zl, uint32_t *zr);
#else
static inline int wcd939x_mbhc_init(struct wcd939x_mbhc **mbhc,
				  struct snd_soc_component *component,
				  struct fw_info *fw_data)
{
	return 0;
}
static inline void wcd939x_mbhc_hs_detect_exit(
					struct snd_soc_component *component)
{
}
static inline int wcd939x_mbhc_hs_detect(struct snd_soc_component *component,
				       struct wcd_mbhc_config *mbhc_cfg)
{
		return 0;
}
static inline void wcd939x_mbhc_deinit(struct snd_soc_component *component)
{
}
static inline void wcd939x_mbhc_ssr_down(struct wcd939x_mbhc *mbhc,
					struct snd_soc_component *component)
{
}
static inline int wcd939x_mbhc_post_ssr_init(struct wcd939x_mbhc *mbhc,
					   struct snd_soc_component *component)
{
	return 0;
}

static inline int wcd939x_mbhc_get_impedance(struct wcd939x_mbhc *wcd939x_mbhc,
					   uint32_t *zl, uint32_t *zr)
{
	if (zl)
		*zl = 0;
	if (zr)
		*zr = 0;
	return -EINVAL;
}
#endif

#endif /* __WCD939X_MBHC_H__ */
