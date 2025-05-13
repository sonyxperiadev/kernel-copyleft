ifeq ($(call is-board-platform-in-list,pineapple cliffs),true)
LOCAL_MODULE_DDK_BUILD := true

LOCAL_MODULE_KO_DIRS := dsp/q6_notifier_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/spf_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audpkt_ion_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/gpr_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/audio_pkt_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/adsp_loader_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audio_prm_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_pdr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/pinctrl_lpi_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_ctrl_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/snd_event_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/mbhc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/swr_dmic_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd9xxx_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/swr_haptics_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/stub_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/machine_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_wsa2_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_wsa_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_va_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_tx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_rx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wsa884x/wsa884x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wsa883x/wsa883x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd937x/wcd937x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd937x/wcd937x_slave_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd938x/wcd938x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd938x/wcd938x_slave_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd939x/wcd939x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd939x/wcd939x_slave_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/hdmi_dlkm.ko
endif

ifeq ($(call is-board-platform-in-list,blair),true)
LOCAL_MODULE_DDK_BUILD := true

LOCAL_MODULE_KO_DIRS := dsp/q6_notifier_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/spf_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audpkt_ion_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/gpr_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/audio_pkt_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/adsp_loader_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audio_prm_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_pdr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/pinctrl_lpi_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_ctrl_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/snd_event_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/mbhc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd9xxx_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/stub_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/machine_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/bolero_cdc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/va_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/tx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/bolero/rx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wsa881x_analog_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd937x/wcd937x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd937x/wcd937x_slave_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd938x/wcd938x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd938x/wcd938x_slave_dlkm.ko
endif

ifeq ($(call is-board-platform-in-list, niobe),true)
LOCAL_MODULE_DDK_BUILD := true

LOCAL_MODULE_KO_DIRS := dsp/q6_notifier_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/spf_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audpkt_ion_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/gpr_dlkm.ko
LOCAL_MODULE_KO_DIRS += ipc/audio_pkt_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/adsp_loader_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/audio_prm_dlkm.ko
LOCAL_MODULE_KO_DIRS += dsp/q6_pdr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/pinctrl_lpi_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/swr_ctrl_dlkm.ko
LOCAL_MODULE_KO_DIRS += soc/snd_event_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd_core_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/mbhc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/swr_dmic_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd9xxx_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/swr_haptics_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/stub_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/machine_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_wsa2_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_wsa_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_va_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_tx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/lpass-cdc/lpass_cdc_rx_macro_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wsa884x/wsa884x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wsa883x/wsa883x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd937x/wcd937x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd937x/wcd937x_slave_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd938x/wcd938x_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/wcd938x/wcd938x_slave_dlkm.ko
LOCAL_MODULE_KO_DIRS += asoc/codecs/hdmi_dlkm.ko
endif

