load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")


def define_modules(target, variant):
    tv = "{}_{}".format(target, variant)

    ddk_module(
        name = "{}_ubwcp".format(tv),
        out = "ubwcp.ko",
        srcs = [
            "ubwcp_main.c",
            "ubwcp_hw.c",
            "ubwcp_hw.h",
            "ubwcp_trace.h",
        ],
        hdrs=["include/uapi/ubwcp_ioctl.h", "include/kernel/ubwcp.h"],
        deps = ["//msm-kernel:all_headers"],
        includes = ["include", "include/kernel"],
        kernel_build = "//msm-kernel:{}".format(tv),
        visibility = ["//visibility:public"]
    )

    copy_to_dist_dir(
        name = "{}_ubwcp_dist".format(tv),
        data = [":{}_ubwcp".format(tv)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )
