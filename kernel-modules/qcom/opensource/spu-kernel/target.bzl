load(":spu_modules.bzl", "spu_driver_modules")
load(":spu_module_build.bzl", "define_consolidate_gki_modules")

def define_pineapple():
    define_consolidate_gki_modules(
        target = "pineapple",
        registry = spu_driver_modules,
        modules = [
            "spcom",
            "spss_utils",
        ],
)
