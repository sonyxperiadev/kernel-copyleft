load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module")

def define_mem(target, variant):
    kernel_build_variant = "{}_{}".format(target, variant)
    include_base = "../../../{}".format(native.package_name())

    ddk_module(
        name = "{}_rmnet_mem".format(kernel_build_variant),
        out = "rmnet_mem.ko",
	includes = ["."],
	hdrs = [ "rmnet_mem.h" ],
        srcs = [
            "rmnet_mem_main.c",
            "rmnet_mem_nl.c",
            "rmnet_mem_nl.h",
            "rmnet_mem_pool.c",
            "rmnet_mem_priv.h",
        ],
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = [
            "//msm-kernel:all_headers",
        ],
        copts = ["-Wno-misleading-indentation"],
    )

    copy_to_dist_dir(
        name = "{}_datarment-ext_dist".format(kernel_build_variant),
        data = [
            ":{}_rmnet_mem".format(kernel_build_variant),
        ],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )
