load(":video_modules.bzl", "video_driver_modules")
load(":video_driver_build.bzl", "define_target_variant_modules")
load(":target_variants.bzl", "get_all_la_variants", "get_all_le_variants")

def define_blair(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = video_driver_modules,
        modules = [
            "msm_video",
        ],
        config_options = [
            "CONFIG_MSM_VIDC_BLAIR",
            "CONFIG_MSM_VIDC_AR50LT",
        ],
    )

def define_monaco(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = video_driver_modules,
        modules = [
            "msm_video",
        ],
        config_options = [
            "CONFIG_MSM_VIDC_MONACO",
            "CONFIG_MSM_VIDC_AR50LT",
        ],
    )

def define_pitti(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = video_driver_modules,
        modules = [
            "msm_video",
        ],
        config_options = [
            "CONFIG_MSM_VIDC_BLAIR",
            "CONFIG_MSM_VIDC_AR50LT",
        ],
    )

def define_vienna(t,v):
    define_target_variant_modules(
        target = t,
        variant = v,
        registry = video_driver_modules,
        modules = [
            "msm_video",
        ],
        config_options = [
            "CONFIG_MSM_VIDC_VIENNA",
            "CONFIG_MSM_VIDC_AR50LT",
        ],
    )

def define_target_modules():
    for (t, v) in get_all_la_variants() + get_all_le_variants():
        if t == "blair":
            define_blair(t, v)
        elif t == "monaco":
            define_monaco(t, v)
        elif t == "pitti":
            define_pitti(t, v)
        elif t == "vienna":
            define_vienna(t, v)
