load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module")

def define_shs(target, variant):
    kernel_build_variant = "{}_{}".format(target, variant)
    include_base = "../../../{}".format(native.package_name())

    ddk_module(
        name = "{}_shs".format(kernel_build_variant),
        out = "rmnet_shs.ko",
        srcs = [
            "rmnet_shs.h",
            "rmnet_shs_common.c",
            "rmnet_shs_common.h",
            "rmnet_shs_config.c",
            "rmnet_shs_config.h",
            "rmnet_shs_freq.c",
            "rmnet_shs_freq.h",
            "rmnet_shs_ll.c",
            "rmnet_shs_ll.h",
            "rmnet_shs_main.c",
            "rmnet_shs_modules.c",
            "rmnet_shs_modules.h",
            "rmnet_shs_wq.c",
            "rmnet_shs_wq.h",
            "rmnet_shs_wq_genl.c",
            "rmnet_shs_wq_genl.h",
            "rmnet_shs_wq_mem.c",
            "rmnet_shs_wq_mem.h",
        ],
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = [
            "//msm-kernel:all_headers",
            "//vendor/qcom/opensource/datarmnet:{}_rmnet_core".format(kernel_build_variant),
            "//vendor/qcom/opensource/datarmnet:rmnet_core_headers",
        ],
        copts = ["-Wno-misleading-indentation"],
    )

    copy_to_dist_dir(
        name = "{}_datarment-ext_dist".format(kernel_build_variant),
        data = [
            ":{}_shs".format(kernel_build_variant),
        ],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )
