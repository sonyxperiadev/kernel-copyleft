load("//build/kernel/kleaf:kernel.bzl", "ddk_module", "ddk_submodule")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//msm-kernel:target_variants.bzl", "get_all_variants")

def _define_module(target, variant):
    tv = "{}_{}".format(target, variant)
    ddk_module(
        name = "{}_msm_ext_display".format(tv),
        srcs = ["src/msm_ext_display.c"],
        out = "msm_ext_display.ko",
        defconfig = "defconfig",
        kconfig = "Kconfig",
        deps = ["//msm-kernel:all_headers",
                "//vendor/qcom/opensource/mm-drivers:mm_drivers_headers"],
        kernel_build = "//msm-kernel:{}".format(tv),
    )

    copy_to_dist_dir(
        name = "{}_msm_ext_display_dist".format(tv),
        data = [":{}_msm_ext_display".format(tv)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )

def define_msm_ext_display():
    for (t, v) in get_all_variants():
        _define_module(t, v)
