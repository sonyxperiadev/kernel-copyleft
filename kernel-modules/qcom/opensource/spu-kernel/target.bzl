load(":spu_modules.bzl", "spu_driver_modules")
load(":spu_module_build.bzl", "define_consolidate_gki_modules")

def define_modules(targets):
    # go over all targets
    for t in targets:
        define_consolidate_gki_modules(
            target = t,
            registry = spu_driver_modules,
            modules = [
               "spcom",
               "spss_utils",
            ],
)
