#ifndef _MSM_AHC_CONFIG_H_
#define _MSM_AHC_CONFIG_H_

#define AHC_PORT_ID	SLIMBUS_0_RX

#ifdef CONFIG_AHC
void msm_routing_ahc_set_copp_idx(int copp_idx);
void msm_routing_ahc_add_controls(struct snd_soc_platform *platform);
#else
void msm_routing_ahc_set_copp_idx(int copp_idx) {}
void msm_routing_ahc_add_controls(struct snd_soc_platform *platform) {}
#endif

#endif
