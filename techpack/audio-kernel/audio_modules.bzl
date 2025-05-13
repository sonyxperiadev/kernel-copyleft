load(":module_mgr.bzl", "create_module_registry")

DSP_PATH = "dsp"
IPC_PATH = "ipc"
SOC_PATH = "soc"
ASOC_PATH = "asoc"
ASOC_CODECS_PATH = ASOC_PATH + "/codecs"
ASOC_CODECS_LPASS_CDC_PATH = ASOC_CODECS_PATH + "/lpass-cdc"
ASOC_CODECS_BOLERO_PATH = ASOC_CODECS_PATH + "/bolero"

audio_modules = create_module_registry([":audio_headers"])
# ------------------------------------ AUDIO MODULE DEFINITIONS ---------------------------------
# >>>> DSP MODULES <<<<
audio_modules.register(
    name = "q6_dlkm",
    path = DSP_PATH,
    conditional_srcs = {
        "CONFIG_SND_SOC_MSM_QDSP6V2_INTF": [
            "msm-audio-event-notify.c",
            "q6_init.c",
        ],
        "CONFIG_SND_SOC_MSM_QDSP6V2_VM": [
            "msm-audio-event-notify.c",
            "msm_audio_ion_vm.c",
            "q6_init.c",
        ],
        "CONFIG_MSM_AVTIMER": [
            "avtimer.c"
        ],
        "CONFIG_XT_LOGGING": [
            "sp_params.c"
        ]
    }
)
audio_modules.register(
    name = "spf_core_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_SPF_CORE",
    srcs = ["spf-core.c"],
    conditional_srcs = {
        "CONFIG_DIGITAL_CDC_RSC_MGR": [
            "digital-cdc-rsc-mgr.c"
        ]
    }
)
audio_modules.register(
    name = "audpkt_ion_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_AUDIO_PKT_ION",
    srcs = ["msm_audio_ion.c"]
)
audio_modules.register(
    name = "q6_notifier_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_MSM_QDSP6_NOTIFIER",
    srcs = [
        "audio_notifier.c",
        "audio_ssr.c"
    ],
)
audio_modules.register(
    name = "adsp_loader_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_MSM_ADSP_LOADER",
    srcs = ["adsp-loader.c"],
)
audio_modules.register(
    name = "audio_prm_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_AUDIO_PRM",
    srcs = ["audio_prm.c"],
)
audio_modules.register(
    name = "q6_pdr_dlkm",
    path = DSP_PATH,
    config_option = "CONFIG_MSM_QDSP6_PDR",
    srcs = ["audio_pdr.c"]
)
# >>>> IPC MODULES <<<<
audio_modules.register(
    name = "gpr_dlkm",
    path = IPC_PATH,
    config_option = "CONFIG_MSM_QDSP6_GPR_RPMSG",
    srcs = ["gpr-lite.c"],
)
audio_modules.register(
    name = "audio_pkt_dlkm",
    path = IPC_PATH,
    config_option = "CONFIG_AUDIO_PKT",
    srcs = ["audio-pkt.c"],
)
# >>>> SOC MODULES <<<<
audio_modules.register(
    name = "pinctrl_lpi_dlkm",
    path = SOC_PATH,
    config_option = "CONFIG_PINCTRL_LPI",
    srcs = ["pinctrl-lpi.c"],
)
audio_modules.register(
    name = "swr_dlkm",
    path = SOC_PATH,
    config_option = "CONFIG_SOUNDWIRE",
    srcs = [
        "regmap-swr.c",
        "soundwire.c"
    ]
)
audio_modules.register(
    name = "swr_ctrl_dlkm",
    path = SOC_PATH,
    conditional_srcs = {
        "CONFIG_SOUNDWIRE_WCD_CTRL": [
            "swr-wcd-ctrl.c"
        ],
        "CONFIG_SOUNDWIRE_MSTR_CTRL": [
            "swr-mstr-ctrl.c"
        ]
    },
)
audio_modules.register(
    name = "snd_event_dlkm",
    path = SOC_PATH,
    config_option = "CONFIG_SND_EVENT",
    srcs = ["snd_event.c"]
)
# >>>> ASOC MODULES <<<<
audio_modules.register(
    name = "machine_dlkm",
    path = ASOC_PATH,
    srcs = [
        "msm_common.c",
    ],
    conditional_srcs = {
        "CONFIG_SND_SOC_SM8150": [
            "sm8150.c",
            "machine_815x_init.c"
        ],
        "CONFIG_SND_SOC_SM6150": [
            "sm6150.c",
            "machine_615x_init.c"
        ],
        "CONFIG_SND_SOC_SA6155": [
            "sa6155.c"
        ],
        "CONFIG_SND_SOC_QCS405": [
            "qcs405.c"
        ],
        "CONFIG_SND_SOC_KONA": [
            "kona.c"
        ],
        "CONFIG_SND_SOC_LAHAINA": [
            "lahaina.c"
        ],
        "CONFIG_SND_SOC_WAIPIO": [
            "waipio.c"
        ],
        "CONFIG_SND_SOC_KALAMA": [
            "kalama.c"
        ],
        "CONFIG_SND_SOC_PINEAPPLE": [
            "pineapple.c"
        ],
        "CONFIG_SND_SOC_NIOBE": [
            "pineapple.c"
        ],
        "CONFIG_SND_SOC_HOLI": [
            "holi.c"
        ],
        "CONFIG_SND_SOC_LITO": [
            "kona.c"
        ],
        "CONFIG_SND_SOC_BENGAL": [
            "bengal.c"
        ],
        "CONFIG_SND_SOC_SA8155": [
            "sa8155.c"
        ],
        "CONFIG_SND_SOC_SDX": [
            "sdx-target.c"
        ]
    },
)
# >>>> ASOC/CODEC MODULES <<<<
audio_modules.register(
    name = "wcd_core_dlkm",
    path = ASOC_CODECS_PATH,
    conditional_srcs = {
        "CONFIG_WCD9XXX_CODEC_CORE": [
            "wcd9xxx-rst.c",
            "wcd9xxx-core-init.c",
            "wcd9xxx-core.c",
            "wcd9xxx-irq.c",
            "wcd9xxx-slimslave.c",
            "wcd9xxx-utils.c",
            "wcd9335-regmap.c",
            "wcd9335-tables.c",
            "msm-cdc-pinctrl.c",
            "msm-cdc-supply.c",
            "wcd934x/wcd934x-regmap.c",
            "wcd934x/wcd934x-tables.c",
        ],
        "CONFIG_WCD9XXX_CODEC_CORE_V2": [
            "wcd9xxx-core-init.c",
            "msm-cdc-pinctrl.c",
            "msm-cdc-supply.c",
        ],
        "CONFIG_SND_SOC_WCD_IRQ": [
            "wcd-irq.c"
        ]
    }
)
audio_modules.register(
    name = "mbhc_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_WCD_MBHC",
    srcs = ["wcd-mbhc-v2.c"],
    conditional_srcs = {
        "CONFIG_SND_SOC_WCD_MBHC_ADC": [
            "wcd-mbhc-adc.c"
        ],
        "CONFIG_SND_SOC_WCD_MBHC_LEGACY": [
            "wcd-mbhc-legacy.c"
        ]
    },
)
audio_modules.register(
    name = "swr_dmic_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_SWR_DMIC",
    srcs = ["swr-dmic.c"]
)
audio_modules.register(
    name = "wcd9xxx_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_WCD9XXX_V2",
    srcs = [
        "wcdcal-hwdep.c",
        "wcd9xxx-soc-init.c",
        "audio-ext-clk-up.c"
    ],
    conditional_srcs = {
        "CONFIG_WCD9XXX_CODEC_CORE": {
            True: [
                "wcd9xxx-common-v2.c",
                "wcd9xxx-resmgr-v2.c",
                "wcd-dsp-utils.c",
                "wcd-dsp-mgr.c",
            ],
            False: [
                "wcd-clsh.c"
            ]
        }
    }
)
audio_modules.register(
    name = "swr_haptics_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SWR_HAPTICS",
    srcs = ["swr-haptics.c"]
)
audio_modules.register(
    name = "stub_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_MSM_STUB",
    srcs = ["msm_stub.c"]
)
audio_modules.register(
    name = "hdmi_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_MSM_HDMI_CODEC_RX",
    srcs = ["msm_hdmi_codec_rx.c"],
    deps = ["//vendor/qcom/opensource/mm-drivers/msm_ext_display:%b_msm_ext_display"]
)
# >>>> ASOC/CODECS/LPASS-CDC MODULES <<<<
audio_modules.register(
    name = "lpass_cdc_dlkm",
    path = ASOC_CODECS_LPASS_CDC_PATH,
    config_option = "CONFIG_SND_SOC_LPASS_CDC",
    srcs = [
        "lpass-cdc.c",
        "lpass-cdc-comp.c",
        "lpass-cdc-utils.c",
        "lpass-cdc-regmap.c",
        "lpass-cdc-tables.c",
        "lpass-cdc-clk-rsc.c",
    ],
)
audio_modules.register(
    name = "lpass_cdc_wsa_macro_dlkm",
    path = ASOC_CODECS_LPASS_CDC_PATH,
    config_option = "CONFIG_LPASS_CDC_WSA_MACRO",
    srcs = ["lpass-cdc-wsa-macro.c"]
)
audio_modules.register(
    name = "lpass_cdc_wsa2_macro_dlkm",
    path = ASOC_CODECS_LPASS_CDC_PATH,
    config_option = "CONFIG_LPASS_CDC_WSA2_MACRO",
    srcs = ["lpass-cdc-wsa2-macro.c"]
)
audio_modules.register(
    name = "lpass_cdc_va_macro_dlkm",
    path = ASOC_CODECS_LPASS_CDC_PATH,
    config_option = "CONFIG_LPASS_CDC_VA_MACRO",
    srcs = ["lpass-cdc-va-macro.c"]
)
audio_modules.register(
    name = "lpass_cdc_rx_macro_dlkm",
    path = ASOC_CODECS_LPASS_CDC_PATH,
    config_option = "CONFIG_LPASS_CDC_RX_MACRO",
    srcs = ["lpass-cdc-rx-macro.c"],
)
audio_modules.register(
    name = "lpass_cdc_tx_macro_dlkm",
    path = ASOC_CODECS_LPASS_CDC_PATH,
    config_option = "CONFIG_LPASS_CDC_TX_MACRO",
    srcs = ["lpass-cdc-tx-macro.c"]
)
# >>>> ASOC/CODECS/BOLERO MODULES <<<<
audio_modules.register(
    name = "bolero_cdc_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_SND_SOC_BOLERO",
    srcs = [
        "bolero-cdc.c",
        "bolero-cdc-utils.c",
        "bolero-cdc-regmap.c",
        "bolero-cdc-tables.c",
        "bolero-clk-rsc.c",
    ],
)
audio_modules.register(
    name = "va_macro_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_VA_MACRO",
    srcs = ["va-macro.c"]
)
audio_modules.register(
    name = "rx_macro_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_RX_MACRO",
    srcs = ["rx-macro.c"]
)
audio_modules.register(
    name = "tx_macro_dlkm",
    path = ASOC_CODECS_BOLERO_PATH,
    config_option = "CONFIG_TX_MACRO",
    srcs = ["tx-macro.c"]
)
# >>>> WSA881X-ANALOG MODULE <<<<
audio_modules.register(
    name = "wsa881x_analog_dlkm",
    path = ASOC_CODECS_PATH,
    config_option = "CONFIG_SND_SOC_WSA881X_ANALOG",
    srcs = [
        "wsa881x-analog.c",
        "wsa881x-tables-analog.c",
        "wsa881x-regmap-analog.c",
	],
    conditional_srcs = {
        "CONFIG_WSA881X_TEMP_SENSOR_DISABLE": {
            False: [
                "wsa881x-temp-sensor.c"
            ]
        }
    }
)
# >>>> WSA883X MODULE <<<<
audio_modules.register(
    name = "wsa883x_dlkm",
    path = ASOC_CODECS_PATH + "/wsa883x",
    config_option = "CONFIG_SND_SOC_WSA883X",
    srcs = [
        "wsa883x.c",
        "wsa883x-regmap.c",
        "wsa883x-tables.c",
    ],
)
# >>>> WSA884X MODULE <<<<
audio_modules.register(
    name = "wsa884x_dlkm",
    path = ASOC_CODECS_PATH + "/wsa884x",
    config_option = "CONFIG_SND_SOC_WSA884X",
    srcs = [
        "wsa884x.c",
        "wsa884x-regmap.c",
        "wsa884x-tables.c",
    ]
)
# >>>> WCD937X MODULES <<<<
audio_modules.register(
    name = "wcd937x_dlkm",
    path = ASOC_CODECS_PATH + "/wcd937x",
    config_option = "CONFIG_SND_SOC_WCD937X",
    srcs = [
        "wcd937x.c",
        "wcd937x-regmap.c",
        "wcd937x-tables.c",
        "wcd937x-mbhc.c",
    ]
)
audio_modules.register(
    name = "wcd937x_slave_dlkm",
    path = ASOC_CODECS_PATH + "/wcd937x",
    config_option = "CONFIG_SND_SOC_WCD937X_SLAVE",
    srcs = ["wcd937x_slave.c"]
)
# >>>> WCD938X MODULES <<<<
audio_modules.register(
    name = "wcd938x_dlkm",
    path = ASOC_CODECS_PATH + "/wcd938x",
    config_option = "CONFIG_SND_SOC_WCD938X",
    srcs = [
        "wcd938x.c",
        "wcd938x-regmap.c",
        "wcd938x-tables.c",
        "wcd938x-mbhc.c",
    ]
)
audio_modules.register(
    name = "wcd938x_slave_dlkm",
    path = ASOC_CODECS_PATH + "/wcd938x",
    config_option = "CONFIG_SND_SOC_WCD938X_SLAVE",
    srcs = ["wcd938x-slave.c"]
)
# >>>> WCD939X MODULES <<<<
audio_modules.register(
    name = "wcd939x_dlkm",
    path = ASOC_CODECS_PATH + "/wcd939x",
    config_option = "CONFIG_SND_SOC_WCD939X",
    srcs = [
        "wcd939x.c",
        "wcd939x-regmap.c",
        "wcd939x-tables.c",
        "wcd939x-mbhc.c",
    ]
)
audio_modules.register(
    name = "wcd939x_slave_dlkm",
    path = ASOC_CODECS_PATH + "/wcd939x",
    config_option = "CONFIG_SND_SOC_WCD939X_SLAVE",
    srcs = ["wcd939x-slave.c"]
)
