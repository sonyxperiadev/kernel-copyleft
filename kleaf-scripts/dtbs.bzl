load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("//build:msm_kernel_extensions.bzl", "get_dtb_list", "get_dtbo_list", "get_dtstree")
load("//build/kernel/kleaf:kernel.bzl", "kernel_build", "kernel_build_config")

def define_qcom_dtb_setup():
    write_file(
        name = "dtb_build_config",
        out = "build.config.qcom.dtbs",
        content = [
            "KERNEL_DIR=common",
            "SOC_DIR=soc-repo",
            "export DTC_INCLUDE=${ROOT_DIR}/${SOC_DIR}/include",
            "BOOT_IMAGE_HEADER_VERSION=4",
            "BUILD_INIT_BOOT_IMG=1",
            "LZ4_RAMDISK=1",
            "PAGE_SIZE=4096",
            "BASE_ADDRESS=0x80000000",
            "VENDOR_BOOTCONFIG+='androidboot.first_stage_console=1 androidboot.hardware=qcom_kp'",
            'DTC_FLAGS="-@"',
            "KERNEL_BINARY=Image",
            "",
        ],
    )

    kernel_build_config(
        name = "build.config.qcom.dtb",
        srcs = [
            ":dtb_build_config",
            "//common:build.config.common",
        ],
    )

def define_qcom_dtbs(
        stem,
        target,
        defconfig,
        cmdline = [""]):
    """Build a the dtbs for a target/variant.

    Args:
        stem: The name of the rule
        target: Configuration to use to build the DTBs
        defconfig: defconfig for the kernel_build.

    Returns:
        A tuple:
        - the first item is the list of DTBs to be built
        - the second item is the list of DTBOs to be built
    """

    write_file(
        name = "{}_cmdline_extras".format(stem),
        out = "{}.cmdline.extras".format(stem),
        content = ["KERNEL_VENDOR_CMDLINE='{}'".format(" ".join(cmdline)), ""],
    )

    kernel_build_config(
        name = "{}.build.config.qcom.dtb".format(stem),
        srcs = [
            ":{}_cmdline_extras".format(stem),
            ":build.config.qcom.dtb",
        ],
    )

    dtb_list = get_dtb_list(target)
    dtbo_list = get_dtbo_list(target)

    kernel_build(
        name = "{}_dtb_build".format(stem),
        srcs = [
            # keep sorted
            ":additional_msm_headers_aarch64_globs",
            "//common:kernel_aarch64_sources",
        ],
        build_config = ":{}.build.config.qcom.dtb".format(stem),
        dtstree = get_dtstree(target),
        outs = dtb_list + dtbo_list + ["vmlinux", "Module.symvers", "Image", "System.map", ".config"],
        base_kernel = ":{}_base_kernel".format(stem),
        kconfig_ext = ":kconfig.msm.generated",
        makefile = "//common:Makefile",
        defconfig = defconfig,
        post_defconfig_fragments = [
            ":{}_merged_defconfig".format(stem),
        ],
        make_goals = [
            "dtbs",
        ],
        visibility = ["//visibility:public"],
    )
    return [dtb_list, dtbo_list]
