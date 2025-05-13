load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")


def define_modules(target, variant):
    tv = "{}_{}".format(target, variant)

    ddk_module(
        name = "{}_stm_st54se_gpio".format(tv),
        out = "stm_st54se_gpio.ko",
        srcs = ["st54spi_gpio.c"],
        includes = [".", "linux"],
        deps = ["//msm-kernel:all_headers"],
        kernel_build = "//msm-kernel:{}".format(tv),
        visibility = ["//visibility:public"]
    )

    copy_to_dist_dir(
        name = "{}_stm_st54se_gpio_dist".format(tv),
        data = [":{}_stm_st54se_gpio".format(tv)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )

