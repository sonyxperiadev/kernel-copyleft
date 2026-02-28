load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")

load(
    "//build/kernel/kleaf:kernel.bzl",
    "ddk_headers",
    "ddk_module",
    "kernel_module",
    "kernel_modules_install",
)

def define_modules(target, variant):
    kernel_build_variant = "{}_{}".format(target, variant)

    ddk_module(
        name = "{}_rdtags".format(kernel_build_variant),
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = ["//msm-kernel:all_headers"],
        srcs = ["rdtags.c"],
        out = "rdtags.ko",
        hdrs = ["rdtags.h"],
    )

    copy_to_dist_dir(
        name = "{}_rdtags_dist".format(kernel_build_variant),
        data = [
            ":{}_rdtags".format(kernel_build_variant),
        ],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )
