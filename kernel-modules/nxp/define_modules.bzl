load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")

def define_modules(target, variant):
    tv = "{}_{}".format(target, variant)
    copts = []
    deps = ["//msm-kernel:all_headers"]

    if target == "pineapple":
       copts.append("-DNFC_SECURE_PERIPHERAL_ENABLED")
       deps += ["//vendor/qcom/opensource/securemsm-kernel:smcinvoke_kernel_headers",
                "//vendor/qcom/opensource/securemsm-kernel:{}_smcinvoke_dlkm".format(tv)
       ]

    ddk_module(
        name = "{}_nxp-nci".format(tv),
        out = "nxp-nci.ko",
        srcs = ["nfc/common.c",
                "nfc/common_nxp.c",
                "nfc/common_qcom.c",
                "nfc/ese_cold_reset.c",
                "nfc/i2c_drv.c",
                "nfc/common.h",
                "nfc/common_nxp.h",
                "nfc/ese_cold_reset.h",
                "nfc/i2c_drv.h"
               ],
        hdrs = ["include/uapi/linux/nfc/nfcinfo.h",
                "include/uapi/linux/nfc/sn_uapi.h"],
        includes = [".", "linux", "nfc", "include/uapi/linux/nfc"],
        copts = copts,
        deps = deps,
        kernel_build= "//msm-kernel:{}".format(tv),
        visibility = ["//visibility:public"]
    )

    copy_to_dist_dir(
        name = "{}_nxp-nci_dist".format(tv),
        data = [":{}_nxp-nci".format(tv)],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
    )

