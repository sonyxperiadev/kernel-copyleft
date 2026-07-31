load("//build/kernel/kleaf:kernel.bzl", "kernel_abi", "kernel_module_group")
load(":configs/malabar_consolidate.bzl", "malabar_consolidate_config")
load(":configs/malabar_perf.bzl", "malabar_perf_config")
load(":kleaf-scripts/android_build.bzl", "define_typical_android_build")
load(":kleaf-scripts/image_opts.bzl", "boot_image_opts")
load(":target_variants.bzl", "la_variants")

target_name = "malabar"

def define_malabar():
    for variant in la_variants:
        board_kernel_cmdline_extras = []
        board_bootconfig_extras = []
        kernel_vendor_cmdline_extras = ["bootconfig"]

        if variant == "consolidate":
            board_bootconfig_extras += ["androidboot.serialconsole=1"]
            board_kernel_cmdline_extras += [
                # do not sort
                "console=ttyMSM0,115200n8",
                "qcom_geni_serial.con_enabled=1",
                "earlycon",
            ]
            kernel_vendor_cmdline_extras += [
                # do not sort
                "console=ttyMSM0,115200n8",
                "qcom_geni_serial.con_enabled=1",
                "earlycon",
            ]

            consolidate_build_img_opts = boot_image_opts(
                earlycon_addr = "qcom_geni,0x994000",
                kernel_vendor_cmdline_extras = kernel_vendor_cmdline_extras,
                board_kernel_cmdline_extras = board_kernel_cmdline_extras,
                board_bootconfig_extras = board_bootconfig_extras,
            )

        else:
            board_kernel_cmdline_extras += ["nosoftlockup console=ttynull qcom_geni_serial.con_enabled=0"]
            kernel_vendor_cmdline_extras += ["nosoftlockup console=ttynull qcom_geni_serial.con_enabled=0"]
            board_bootconfig_extras += ["androidboot.serialconsole=0"]

            perf_build_img_opts = boot_image_opts(
                earlycon_addr = "qcom_geni,0x994000",
                kernel_vendor_cmdline_extras = kernel_vendor_cmdline_extras,
                board_kernel_cmdline_extras = board_kernel_cmdline_extras,
                board_bootconfig_extras = board_bootconfig_extras,
            )

    define_typical_android_build(
        name = "malabar",
        consolidate_config = malabar_consolidate_config,
        perf_config = malabar_perf_config,
        consolidate_build_img_opts = consolidate_build_img_opts,
        perf_build_img_opts = perf_build_img_opts,
        consolidate_kwargs = {
            "config_path": "configs/malabar_consolidate.bzl",
        },
        perf_kwargs = {
            "config_path": "configs/malabar_perf.bzl",
        },
    )

    kernel_abi(
        name = "malabar_perf_abi",
        kernel_build = "//common:kernel_aarch64",
        kernel_modules = [
            ":malabar_perf_all_modules",
        ],
    )
