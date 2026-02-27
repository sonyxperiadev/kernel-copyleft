PWR_PATH = "pwr"
SLIMBUS_PATH = "slimbus"
FMRTC_PATH = "rtc6226"
BTFMCODEC_PATH = "btfmcodec"

# This dictionary holds all the BT modules included in the bt-kernel
bt_modules = {}

def register_bt_modules(name, path = None, config_opt = None, srcs = [], config_srcs = {}, deps = [], config_deps = {}):
    """
    Register modules
    Args:
        name: Name of the module (which will be used to generate the name of the .ko file)
        path: Path in which the source files can be found
        config_opt: Config name used in Kconfig (not needed currently)
        srcs: source files and local headers
        config_srcs: source files and local headers that depend on a config define being enabled.
        deps: a list of dependent targets
        config_deps: a list of dependent targets that depend on a config define being enabled.
    """
    processed_config_srcs = {}
    processed_config_deps = {}

    for config_src_name in config_srcs:
        config_src = config_srcs[config_src_name]

        if type(config_src) == "list":
            processed_config_srcs[config_src_name] = {True: config_src}
        else:
            processed_config_srcs[config_src_name] = config_src

    for config_deps_name in config_deps:
        config_dep = config_deps[config_deps_name]

        if type(config_dep) == "list":
            processed_config_deps[config_deps_name] = {True: config_dep}
        else:
            processed_config_deps[config_deps_name] = config_dep

    module = struct(
        name = name,
        path = path,
        srcs = srcs,
        config_srcs = processed_config_srcs,
        config_opt = config_opt,
        deps = deps,
        config_deps = processed_config_deps,
    )
    bt_modules[name] = module

# --- BT Modules ---

register_bt_modules(
    name = "btpower",
    path = PWR_PATH,
    config_opt = "CONFIG_MSM_BT_POWER",
    srcs = ["btpower.c"],
    config_deps = {
        "CONFIG_BT_HW_SECURE_DISABLE": [
            "//vendor/qcom/opensource/securemsm-kernel:%b_smcinvoke_dlkm",
        ]
    },
)

register_bt_modules(
    name = "bt_fm_slim",
    path = SLIMBUS_PATH,
    # config_opt = "CONFIG_BTFM_SLIM",
    srcs = [
        "btfm_slim.c",
        "btfm_slim.h",
        "btfm_slim_slave.c",
        "btfm_slim_slave.h",
        "btfm_slim_codec.c",
    ],
    deps = [":%b_btpower"],
)

register_bt_modules(
    name = "btfm_slim_codec",
    path = SLIMBUS_PATH,
    config_opt = "CONFIG_SLIM_BTFM_CODEC",
    srcs = [
        "btfm_slim.c",
        "btfm_slim.h",
        "btfm_slim_slave.c",
        "btfm_slim_slave.h",
        "btfm_slim_hw_interface.c",
        "btfm_slim_hw_interface.h",
    ],
    deps = [":%b_btpower", ":%b_btfmcodec", ":btfmcodec_headers"],
)

register_bt_modules(
   name = "btfmcodec",
   path = BTFMCODEC_PATH,
   config_opt = "CONFIG_BTFM_CODEC",
   srcs = [
        "btfm_codec.c",
        "btfm_codec_btadv_interface.c",
	"btfm_codec_hw_interface.c",
	"btfm_codec_interface.c",
	],
   deps = [":btfmcodec_headers"],
)

register_bt_modules(
    name = "radio-i2c-rtc6226-qca",
    path = FMRTC_PATH,
    config_opt = "CONFIG_I2C_RTC6226_QCA",
    srcs = [
        "radio-rtc6226-common.c",
        "radio-rtc6226-i2c.c",
        "radio-rtc6226.h",
    ],
)
