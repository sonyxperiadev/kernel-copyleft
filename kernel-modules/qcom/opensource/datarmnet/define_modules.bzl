load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module")

def define_modules(target, variant):
    kernel_build_variant = "{}_{}".format(target, variant)
    include_base = "../../../{}".format(native.package_name())

    #The below will take care of the defconfig
    #include_defconfig = ":{}_defconfig".format(variant)

    ddk_module(
        name = "{}_rmnet_ctl".format(kernel_build_variant),
        out = "rmnet_ctl.ko",
        srcs = [
            "core/rmnet_ctl_ipa.c",
            "core/rmnet_ctl.h",
            "core/rmnet_ctl_client.h",
        ],
        kconfig = "core/Kconfig",
        conditional_srcs = {
            "CONFIG_ARCH_PINEAPPLE": {
                True: [
                    "core/rmnet_ctl_client.c",
                ],
            },
            "CONFIG_ARCH_SUN": {
                True: [
                    "core/rmnet_ctl_client.c",
                ],
	    },
            "CONFIG_ARCH_PARROT": {
                True: [
                    "core/rmnet_ctl_client.c",
                ],
	    },
            "CONFIG_ARCH_MONACO": {
                True: [
                    "core/rmnet_ctl_client.c",
                ],
            },
            "CONFIG_ARCH_TUNA": {
                True: [
                    "core/rmnet_ctl_client.c",
                ],
            },
        },
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = [
            "//vendor/qcom/opensource/dataipa:{}_ipam".format(kernel_build_variant),
            "//msm-kernel:all_headers",
            "//vendor/qcom/opensource/dataipa:include_headers",
        ],
    )

    ddk_module(
        name = "{}_rmnet_core".format(kernel_build_variant),
        out = "rmnet_core.ko",
        srcs = [
            "core/rmnet_config.c",
            "core/rmnet_descriptor.c",
            "core/rmnet_genl.c",
            "core/rmnet_handlers.c",
            "core/rmnet_map_command.c",
            "core/rmnet_map_data.c",
            "core/rmnet_module.c",
            "core/rmnet_vnd.c",
            "core/dfc_qmap.c",
            "core/dfc_qmi.c",
            "core/qmi_rmnet.c",
            "core/wda_qmi.c",
            "core/rmnet_ll.c",
            "core/rmnet_ll_ipa.c",
            "core/rmnet_qmap.c",
            "core/rmnet_ll_qmap.c",
        ],
        local_defines = [
            "RMNET_TRACE_INCLUDE_PATH={}/core".format(include_base),
        ],
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = [
            ":rmnet_core_headers",
            ":{}_rmnet_ctl".format(kernel_build_variant),
            "//vendor/qcom/opensource/dataipa:{}_ipam".format(kernel_build_variant),
            "//vendor/qcom/opensource/datarmnet-ext/mem:{}_rmnet_mem".format(kernel_build_variant),
            "//msm-kernel:all_headers",
            "//vendor/qcom/opensource/dataipa:include_headers",
            "//vendor/qcom/opensource/datarmnet-ext/mem:rmnet_mem_headers",
        ],
    )

    copy_to_dist_dir(
        name = "{}_modules_dist".format(kernel_build_variant),
        data = [
            "{}_rmnet_core".format(kernel_build_variant),
            "{}_rmnet_ctl".format(kernel_build_variant),
        ],
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )
