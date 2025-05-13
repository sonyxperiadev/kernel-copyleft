load(":eva_modules.bzl", "eva_modules")
load(":eva_module_build.bzl", "define_consolidate_gki_modules")

def define_pineapple():
    define_consolidate_gki_modules(
        target = "pineapple",
        registry = eva_modules,
        modules = [
            "msm-eva",
        ],
        config_options = [
            #"CONFIG_TARGET_SYNX_ENABLE",
            "TARGET_SYNX_ENABLE",
            "TARGET_DSP_ENABLE",
            "CONFIG_EVA_PINEAPPLE"
        ],
    )
