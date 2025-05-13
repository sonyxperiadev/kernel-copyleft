load(":spu_module_build.bzl", "spu_driver_module_entry")


spu_driver_modules = spu_driver_module_entry([":spu_headers"])
module_entry = spu_driver_modules.register

#--------------- SPU-DRIVERS MODULES ------------------

module_entry(
    name = "spcom",
    config_option = "CONFIG_MSM_SPCOM",
    srcs = ["spcom.c"],
    path = "drivers"
)

module_entry(
    name = "spss_utils",
    config_option = "CONFIG_MSM_SPSS_UTILS",
    srcs = ["spss_utils.c"],
    path = "drivers"
)


