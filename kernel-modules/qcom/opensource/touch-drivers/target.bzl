load(":touch_modules.bzl", "touch_driver_modules")
load(":touch_modules_build.bzl", "define_target_variant_modules")
load(":target_variants.bzl", "get_all_variants")

def define_vienna(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "raydium_ts",
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_VIENNA",
            "CONFIG_MSM_TOUCH",
            "CONFIG_TOUCHSCREEN_RM_TS",
        ],
)

def define_sun(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "atmel_mxt_ts",
            "dummy_ts",
            "st_fts",
           "focaltech_tp",
            "qts"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_SUN",
            "CONFIG_MSM_TOUCH",
            "CONFIG_TOUCHSCREEN_ATMEL_MXT",
            "CONFIG_TOUCHSCREEN_ST",
            "CONFIG_TOUCH_FOCALTECH",
            "CONFIG_QTS_ENABLE",
            "CONFIG_TOUCHSCREEN_DUMMY",
           "CONFIG_TOUCH_FOCALTECH_FT3610"
        ],
)

def define_sunvm(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "goodix_ts",
            "st_fts",
            "qts"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_SUN",
        ],
        vm_target = True,
)

def define_canoevm(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "goodix_ts",
            "st_fts",
            "qts"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_CANOE",
        ],
        vm_target = True,
)

def define_canoe(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "atmel_mxt_ts",
            "dummy_ts",
            "goodix_ts",
            "st_fts",
            "qts"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_CANOE",
            "CONFIG_MSM_TOUCH",
            "CONFIG_TOUCHSCREEN_GOODIX_BRL",
            "CONFIG_TOUCHSCREEN_ATMEL_MXT",
            "CONFIG_TOUCHSCREEN_ST",
            "CONFIG_QTS_ENABLE",
            "CONFIG_TOUCHSCREEN_DUMMY"
        ],
)

def define_pineapple(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "atmel_mxt_ts",
            "dummy_ts",
            "focaltech_tp",
            "qts"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_PINEAPPLE",
            "CONFIG_MSM_TOUCH",
            "CONFIG_TOUCHSCREEN_ATMEL_MXT",
            "CONFIG_TOUCHSCREEN_DUMMY",
            "CONFIG_QTS_ENABLE",
            "CONFIG_TOUCH_FOCALTECH_FT3610"
        ],
)

def define_blair(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "synaptics_tcm_ts",
            "focaltech_tp"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_BLAIR",
            "CONFIG_MSM_TOUCH",
            "CONFIG_TOUCH_FOCALTECH",
            "CONFIG_TOUCHSCREEN_SYNAPTICS_TCM",
            "CONFIG_TOUCH_FOCALTECH_FT3610"
        ],
)

def define_parrot(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
	    "qts",
	    "focaltech_tp"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_PARROT",
	    "CONFIG_ARCH_RAVELIN",
            "CONFIG_MSM_TOUCH",
	    "CONFIG_QTS_ENABLE",
	    "CONFIG_TOUCH_FOCALTECH_FT3610"
        ],
)

def define_lahaina(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "nt36xxx-i2c",
	    "qts",
	    "focaltech_fts"
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_LAHAINA",
            "CONFIG_MSM_TOUCH",
	    "CONFIG_QTS_ENABLE",
            "CONFIG_TOUCHSCREEN_NT36XXX_I2C",
	    "CONFIG_TOUCH_FOCALTECH",
        ],
)

def define_monaco(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = touch_driver_modules,
        modules = [
            "pt_ts",
            "pt_i2c",
            "pt_device_access",
            "pt_debug",
            "raydium_ts",
        ],
        config_options = [
            "TOUCH_DLKM_ENABLE",
            "CONFIG_ARCH_MONACO",
            "CONFIG_MSM_TOUCH",
            "CONFIG_TOUCHSCREEN_PARADE",
            "CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT",
            "CONFIG_TOUCHSCREEN_PARADE_I2C",
            "CONFIG_TOUCHSCREEN_PARADE_DEVICE_ACCESS",
            "CONFIG_TOUCHSCREEN_PARADE_BUTTON",
            "CONFIG_TOUCHSCREEN_PARADE_PROXIMITY",
            "CONFIG_TOUCHSCREEN_PARADE_DEBUG_MDL",
            "CONFIG_TOUCHSCREEN_RM_TS",
        ],
)

def define_touch_target():
    for (t, v) in get_all_variants():
        if t == "blair":
            define_blair(t, v)
        elif t == "pineapple":
            define_pineapple(t, v)
        elif t == "parrot":
            define_parrot(t, v)
        elif t == "lahaina":
            define_lahaina(t, v)
        elif t == "monaco":
            define_monaco(t, v)
        elif t == "sun-tuivm":
            define_sunvm(t, v)
        elif t == "sun-oemvm":
            define_sunvm(t, v)
        elif t == "canoe":
            define_canoe(t, v)
        elif t == "canoe-tuivm":
            define_canoevm(t, v)
        elif t == "canoe-oemvm":
            define_canoevm(t, v)
        elif t == "sun":
            define_sun(t, v)
        elif t == "vienna":
            define_vienna(t, v)
        else:
            pass
