load("//build/kernel/kleaf:kernel.bzl", "kernel_abi", "kernel_module_group")
load(":configs/bengal_consolidate.bzl", "bengal_consolidate_config")
load(":configs/bengal_perf.bzl", "bengal_perf_config")
load(":kleaf-scripts/android_build.bzl", "define_typical_android_build")
load(":kleaf-scripts/image_opts.bzl", "boot_image_opts")
load(":kleaf-scripts/vm_build.bzl", "define_typical_vm_build")
load(":target_variants.bzl", "la_variants")

target_name = "bengal"

def define_bengal():
    for variant in la_variants:
        board_kernel_cmdline_extras = []
        board_bootconfig_extras = []
        kernel_vendor_cmdline_extras = ["bootconfig"]

        board_bootconfig_extras += ["androidboot.serialconsole=1"]
        if variant == "consolidate":
            board_kernel_cmdline_extras += [
                # do not sort
                "console=ttyMSM0,115200n8",
                "qcom_geni_serial.con_enabled=1",
                "earlycon",
                "ufshcd_core.uic_cmd_timeout=2000",
            ]
            kernel_vendor_cmdline_extras += [
                # do not sort
                "console=ttyMSM0,115200n8",
                "qcom_geni_serial.con_enabled=1",
                "earlycon",
            ]

            consolidate_build_img_opts = boot_image_opts(
                earlycon_addr = "qcom_geni,0x4a90000",
                kernel_vendor_cmdline_extras = kernel_vendor_cmdline_extras,
                board_kernel_cmdline_extras = board_kernel_cmdline_extras,
                board_bootconfig_extras = board_bootconfig_extras,
            )

        else:
            board_kernel_cmdline_extras += ["nosoftlockup console=ttynull qcom_geni_serial.con_enabled=0"]
            kernel_vendor_cmdline_extras += ["nosoftlockup console=ttynull qcom_geni_serial.con_enabled=0"]
            board_bootconfig_extras += ["androidboot.serialconsole=0"]

            perf_build_img_opts = boot_image_opts(
                earlycon_addr = "qcom_geni,0x4a90000",
                kernel_vendor_cmdline_extras = kernel_vendor_cmdline_extras,
                board_kernel_cmdline_extras = board_kernel_cmdline_extras,
                board_bootconfig_extras = board_bootconfig_extras,
            )

    define_typical_android_build(
        name = "bengal",
        consolidate_config = bengal_consolidate_config,
        perf_config = bengal_perf_config,
        consolidate_build_img_opts = consolidate_build_img_opts,
        perf_build_img_opts = perf_build_img_opts,
        consolidate_kwargs = {
            "config_path": "configs/bengal_consolidate.bzl",
        },
        perf_kwargs = {
            "config_path": "configs/bengal_perf.bzl",
        },
    )

    kernel_abi(
        name = "bengal_perf_abi",
        kernel_build = "//common:kernel_aarch64",
        kernel_modules = [
            ":bengal_perf_all_modules",
        ],
    )
