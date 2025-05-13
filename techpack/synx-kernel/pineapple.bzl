load(":synx_modules.bzl", "synx_modules")
load(":synx_module_build.bzl", "define_consolidate_gki_modules")

def define_pineapple():
    define_consolidate_gki_modules(
        target = "pineapple",
        registry = synx_modules,
        modules = [
            "synx-driver",
            "ipclite",
            "ipclite_test",
        ],
        config_options = [
            "TARGET_SYNX_ENABLE",
        ],
    )
