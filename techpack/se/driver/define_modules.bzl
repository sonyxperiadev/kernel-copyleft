load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")

def define_modules(target, variant):
    tv = "{}_{}".format(target, variant)

    ddk_module(
        name = "{}_stm_nfc_i2c".format(tv),
        out = "stm_nfc_i2c.ko",
        srcs = ["nfc/st21nfc.c",
                "nfc/st21nfc.h"
               ],
        hdrs = ["include/uapi/linux/nfc/st_uapi.h"],
        includes = [".", "linux", "nfc", "include/uapi/linux/nfc"],
        deps = ["//msm-kernel:all_headers",
                "//vendor/qcom/opensource/securemsm-kernel:smcinvoke_kernel_headers",
                "//vendor/qcom/opensource/securemsm-kernel:{}_smcinvoke_dlkm".format(tv)],
        kernel_build = "//msm-kernel:{}".format(tv),
        visibility = ["//visibility:public"]
    )

    copy_to_dist_dir(
        name = "{}_stm_nfc_i2c_dist".format(tv),
        data = [":{}_stm_nfc_i2c".format(tv)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )
