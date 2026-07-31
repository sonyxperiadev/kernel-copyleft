load(
    "//build:msm_kernel_extensions.bzl",
    "define_extras",
    "get_dtb_list",
    "get_dtbo_list",
)
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:hermetic_tools.bzl", "hermetic_genrule")
load(
    "//build/kernel/kleaf:kernel.bzl",
    "ddk_headers",
    "kernel_unstripped_modules_archive",
    "merged_kernel_uapi_headers",
)
load(":kleaf-scripts/dtbs.bzl", "define_qcom_dtbs")
load(":kleaf-scripts/image_opts.bzl", "vm_image_opts")
load(":qcom_modules.bzl", "registry")

def define_make_vm_dtb_img(target, dtb_list, page_size):
    compiled_dtbs = ["//soc-repo:{}_dtb_build/{}".format(target, t) for t in dtb_list]
    dtb_cmd = "compiled_dtb_list=\"{}\"\n".format(" ".join(["$(location {})".format(d) for d in compiled_dtbs]))
    dtb_cmd += """
      set +x
      $(location //prebuilts/kernel-build-tools:mkdtboimg) \\
        create "$@" --page_size={page_size} $${{compiled_dtb_list}}
      set -x
    """.format(page_size = page_size)

    hermetic_genrule(
        name = "{}_vm_dtb_img".format(target),
        srcs = compiled_dtbs,
        outs = ["{}-dtb.img".format(target)],
        tools = ["//prebuilts/kernel-build-tools:mkdtboimg"],
        cmd = dtb_cmd,
    )

    copy_to_dist_dir(
        name = "{}_vm_dist".format(target),
        data = [
            "{}-dtb.img".format(target),
        ] + compiled_dtbs,
        dist_dir = "out/msm-kernel-{}/dist".format(target),
        flat = True,
        log = "info",
    )

def define_single_vm_build(
        name,
        config_fragment,
        base_kernel,
        dtb_target = None,
        ddk_config_deps = None,
        implicit_config_fragment = None,
        config_path = None):
    modules = registry.define_modules(
        name,
        config_fragment,
        base_kernel,
        ddk_config_deps = ddk_config_deps,
        implicit_config_fragment = implicit_config_fragment,
        config_path = config_path,
    )

    if dtb_target:
        define_qcom_dtbs(
            stem = name,
            target = dtb_target,
            defconfig = "//soc-repo:arch/arm64/configs/generic_vm_defconfig",
        )

    hermetic_genrule(
        name = "{}_merge_msm_uapi_headers".format(name),
        srcs = [
            # do not sort
            ":{}_merged_kernel_uapi_headers".format(name),
            "msm_uapi_headers",
        ],
        outs = ["{}_kernel-uapi-headers.tar.gz".format(name)],
        cmd = """
                    mkdir -p intermediate_dir
                    for file in $(SRCS)
                    do
                    tar xf $$file -C intermediate_dir
                    done
                    tar czf $(OUTS) -C intermediate_dir usr/
                    rm -rf intermediate_dir
        """,
    )

    merged_kernel_uapi_headers(
        name = "{}_merged_kernel_uapi_headers".format(name),
        kernel_build = base_kernel,
    )

    kernel_unstripped_modules_archive(
        name = "{}_unstripped_modules_tar".format(name),
        kernel_build = base_kernel,
        kernel_modules = [":{}/{}".format(name, module) for module in modules],
    )

    copy_to_dist_dir(
        name = "{}_host_dist".format(name),
        data = [
            ":gen-headers_install.sh",
            ":unifdef",
        ],
        dist_dir = "out/msm-kernel-{}/host".format(name),
        flat = True,
        log = "info",
    )

    copy_to_dist_dir(
        name = "{}_dist".format(name),
        data = [
            ":{}_modules_install".format(name),
            ":{}_signed_modules".format(name),
            ":{}_merge_msm_uapi_headers".format(name),
            ":signing_key",
            ":verity_key",
            base_kernel,
            ":{}_dtb_build/.config".format(name),
        ],
        dist_dir = "out/msm-kernel-{}/dist".format(name),
        flat = True,
        allow_duplicate_filenames = True,
        log = "info",
    )

    copy_to_dist_dir(
        name = "{}_um_dist".format(name),
        archives = [
            "{}_unstripped_modules_tar".format(name),
        ],
        dist_dir = "out/msm-kernel-{}/dist".format(name),
        flat = True,
        wipe_dist_dir = False,
        log = "info",
    )

def define_vm_build(
        name,
        configs,
        **kwargs):
    for (variant, options) in configs.items():
        define_single_vm_build(
            name = "{}_{}".format(name, variant),
            **(options | kwargs)
        )
        dtb_list = get_dtb_list(name.replace("_", "-"))
        dtbo_list = get_dtbo_list(name.replace("_", "-"))
        vm_opts = vm_image_opts()

        #use only dtbs related to the variant for dtb image creation
        if "canoe" in name:
            dtb_list = dtbo_list

        if "tuivm" in name:
            seg_dtb_list = [dtb for dtb in dtb_list if "-vm-" in dtb]
        elif "oemvm" in name:
            seg_dtb_list = [dtb for dtb in dtb_list if "-oemvm-" in dtb]
        define_make_vm_dtb_img(name + "_" + variant, seg_dtb_list, vm_opts.dummy_img_size)
        k_config = "kernel_aarch64_qtvm"
        if "debug" in variant:
            k_config = "kernel_aarch64_qtvm_debug"

        define_extras(name + "_" + variant, kbuild_config = k_config, flavor = "vm")

def define_typical_vm_build(
        name,
        config,
        debug_config,
        config_kwargs = None,
        debug_kwargs = None,
        **kwargs):
    if config_kwargs == None:
        config_kwargs = dict()
    if debug_kwargs == None:
        debug_kwargs = dict()

    # See b/370450569#comment34
    common_info = "{}_common_info".format(name)
    ddk_headers(
        name = common_info,
        kconfigs = [":kconfig.msm.generated"],
        defconfigs = [":{}_defconfig_defconfig".format(name)],
    )

    define_vm_build(
        name = name,
        configs = {
            "debug-defconfig": {
                "config_fragment": debug_config,
                "base_kernel": "//soc-repo:kernel_aarch64_qtvm_debug",
                "ddk_config_deps": [common_info],
                "implicit_config_fragment": config,
            } | debug_kwargs,
            "defconfig": {
                "config_fragment": config,
                "base_kernel": "//soc-repo:kernel_aarch64_qtvm",
                "ddk_config_deps": [common_info],
            } | config_kwargs,
        },
        **kwargs
    )
