load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//msm-kernel:target_variants.bzl", "get_all_variants")

_default_module_enablement_list = [
    "cnss_nl",
    "cnss_prealloc",
    "cnss_utils",
    "wlan_firmware_service"
]

_cnss2_enabled_target = ["niobe", "pineapple", "sun"]
_icnss2_enabled_target = ["blair", "pineapple", "monaco"]

def _get_module_list(target, variant):
    tv = "{}_{}".format(target, variant)

    ret = []
    is_wlan_platform_enabled = False

    if target in _cnss2_enabled_target:
        ret.extend(["cnss2", "cnss_plat_ipc_qmi_svc"])
        is_wlan_platform_enabled = True

    if target in _icnss2_enabled_target:
        ret.extend(["icnss2"])
        is_wlan_platform_enabled = True

    if is_wlan_platform_enabled:
        ret.extend(_default_module_enablement_list)

    return [":{}_{}".format(tv, mod) for mod in ret]

def _define_platform_config_rule(module, target, variant):
    tv = "{}_{}".format(target, variant)
    native.genrule(
        name = "{}/{}_defconfig_generate_perf".format(module, tv),
        outs = ["{}/{}_defconfig.generated_perf".format(module, tv)],
        srcs = [
            "{}/{}_gki_defconfig".format(module, target),
        ],
        cmd = "cat $(SRCS) > $@",
    )
    native.genrule(
        name = "{}/{}_defconfig_generate_gki".format(module, tv),
        outs = ["{}/{}_defconfig.generated_gki".format(module, tv)],
        srcs = [
            "{}/{}_gki_defconfig".format(module, target),
        ],
        cmd = "cat $(SRCS) > $@",
    )
    native.genrule(
        name = "{}/{}_defconfig_generate_consolidate".format(module, tv),
        outs = ["{}/{}_defconfig.generated_consolidate".format(module, tv)],
        srcs = [
            "{}/{}_consolidate_defconfig".format(module, target),
        ],
        cmd = "cat $(SRCS) > $@",
    )

def _define_modules_for_target_variant(target, variant):
    tv = "{}_{}".format(target, variant)

    cnss2_enabled = 0
    plat_ipc_qmi_svc_enabled = 0
    icnss2_enabled = 0

    if target in _cnss2_enabled_target:
        cnss2_enabled = 1
        plat_ipc_qmi_svc_enabled = 1

    if target in _icnss2_enabled_target:
        icnss2_enabled = 1

    print("tv=", tv)
    if cnss2_enabled:
        module = "cnss2"
        _define_platform_config_rule(module, target, variant)
        defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
        ddk_module(
            name = "{}_cnss2".format(tv),
            srcs = native.glob([
                "cnss2/main.c",
                "cnss2/bus.c",
                "cnss2/debug.c",
                "cnss2/pci.c",
                "cnss2/pci_platform.h",
                "cnss2/power.c",
                "cnss2/genl.c",
                "cnss2/*.h",
                "cnss_utils/*.h",
            ]),
            includes = ["cnss", "cnss_utils"],
            kconfig = "cnss2/Kconfig",
            defconfig = defconfig,
            conditional_srcs =  {
                "CONFIG_CNSS2_QMI": {
                    True: [
                        "cnss2/qmi.c",
                        "cnss2/coexistence_service_v01.c",
                        "cnss2/ip_multimedia_subsystem_private_service_v01.c",
                    ]
                },
                "CONFIG_PCI_MSM": {
                    True: [
                        "cnss2/pci_qcom.c",
                    ],
                },
            },
            out = "cnss2.ko",
            kernel_build = "//msm-kernel:{}".format(tv),
            deps = [
                "//vendor/qcom/opensource/securemsm-kernel:{}_smcinvoke_dlkm".format(tv),
                ":{}_cnss_utils".format(tv),
                ":{}_cnss_prealloc".format(tv),
                ":{}_wlan_firmware_service".format(tv),
                ":{}_cnss_plat_ipc_qmi_svc".format(tv),
                "//msm-kernel:all_headers",
                ":wlan-platform-headers",
            ],
        )

    if icnss2_enabled:
        module = "icnss2"
        _define_platform_config_rule(module, target, variant)
        defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
        ddk_module(
            name = "{}_icnss2".format(tv),
            srcs = native.glob([
                "icnss2/main.c",
                "icnss2/debug.c",
                "icnss2/power.c",
                "icnss2/genl.c",
                "icnss2/*.h",
                "cnss_utils/*.h",
            ]),
            includes = ["icnss2", "cnss_utils"],
            kconfig = "icnss2/Kconfig",
            copts = ["-Wno-format"],
            defconfig = defconfig,
            conditional_srcs = {
                "CONFIG_ICNSS2_QMI": {
                    True: [
                        "icnss2/qmi.c",
                    ],
                },
            },
            out = "icnss2.ko",
            kernel_build = "//msm-kernel:{}".format(tv),
            deps = [
                ":{}_cnss_utils".format(tv),
                ":{}_cnss_prealloc".format(tv),
                ":{}_wlan_firmware_service".format(tv),
                "//msm-kernel:all_headers",
                ":wlan-platform-headers",
            ],
        )
    module = "cnss_genl"
    _define_platform_config_rule(module, target, variant)
    defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
    ddk_module(
        name = "{}_cnss_nl".format(tv),
        srcs = [
            "cnss_genl/cnss_nl.c",
        ],
        kconfig = "cnss_genl/Kconfig",
        defconfig = defconfig,
        out = "cnss_nl.ko",
        kernel_build = "//msm-kernel:{}".format(tv),
        deps = [
            "//msm-kernel:all_headers",
            ":wlan-platform-headers",
        ],
    )

    module = "cnss_prealloc"
    _define_platform_config_rule(module, target, variant)
    defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
    ddk_module(
        name = "{}_cnss_prealloc".format(tv),
        srcs = native.glob([
            "cnss_prealloc/cnss_prealloc.c",
            "cnss_utils/*.h",
        ]),
        includes = ["cnss_utils"],
        kconfig = "cnss_prealloc/Kconfig",
        defconfig = defconfig,
        out = "cnss_prealloc.ko",
        kernel_build = "//msm-kernel:{}".format(tv),
        deps = [
            "//msm-kernel:all_headers",
            ":wlan-platform-headers",
        ],
    )

    module = "cnss_utils"
    _define_platform_config_rule(module, target, variant)
    defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
    ddk_module(
        name = "{}_cnss_utils".format(tv),
        srcs = native.glob([
            "cnss_utils/cnss_utils.c",
            "cnss_utils/*.h"
        ]),
        kconfig = "cnss_utils/Kconfig",
        defconfig = defconfig,
        out = "cnss_utils.ko",
        kernel_build = "//msm-kernel:{}".format(tv),
        deps = [
            "//msm-kernel:all_headers",
            ":wlan-platform-headers",
        ],
    )

    module = "cnss_utils"
    defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
    ddk_module(
        name = "{}_wlan_firmware_service".format(tv),
        srcs = native.glob([
            "cnss_utils/wlan_firmware_service_v01.c",
            "cnss_utils/device_management_service_v01.c",
            "cnss_utils/*.h"
        ]),
        kconfig = "cnss_utils/Kconfig",
        defconfig = defconfig,
        out = "wlan_firmware_service.ko",
        kernel_build = "//msm-kernel:{}".format(tv),
        deps = ["//msm-kernel:all_headers"],
    )

    module = "cnss_utils"
    defconfig = ":{}/{}_defconfig_generate_{}".format(module, tv, variant)
    if plat_ipc_qmi_svc_enabled:
        ddk_module(
            name = "{}_cnss_plat_ipc_qmi_svc".format(tv),
            srcs = native.glob([
                "cnss_utils/cnss_plat_ipc_qmi.c",
                "cnss_utils/cnss_plat_ipc_service_v01.c",
                "cnss_utils/*.h"
            ]),
            kconfig = "cnss_utils/Kconfig",
            defconfig = defconfig,
            out = "cnss_plat_ipc_qmi_svc.ko",
            kernel_build = "//msm-kernel:{}".format(tv),
            deps = ["//msm-kernel:all_headers"],
        )
    tv = "{}_{}".format(target, variant)
    copy_to_dist_dir(
        name = "{}_modules_dist".format(tv),
        data = _get_module_list(target, variant),
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info"
    )

def define_modules():
    for (t, v) in get_all_variants():
        print("v=", v)
        _define_modules_for_target_variant(t, v)
