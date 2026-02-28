load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//msm-kernel:target_variants.bzl", "get_all_variants")
load("//build/kernel/kleaf:kernel.bzl", "ddk_module")

def define_modules(target, variant):
    kernel_build_variant = "{}_{}".format(target, variant)
    include_base = "../../../{}".format(native.package_name())

    #The below will take care of the defconfig
    include_defconfig = ":{}_defconfig".format(variant)

    mod_list = []

    ddk_module(
        name = "{}_gsim".format(kernel_build_variant),
        out = "gsim.ko",
        srcs = [
            "drivers/platform/msm/gsi/gsi.c",
            "drivers/platform/msm/gsi/gsi.h",
            "drivers/platform/msm/gsi/gsi_dbg.c",
            "drivers/platform/msm/gsi/gsi_emulation.h",
            "drivers/platform/msm/gsi/gsi_emulation_stubs.h",
            "drivers/platform/msm/gsi/gsi_trace.h",
            "drivers/platform/msm/gsi/gsihal/gsihal.c",
            "drivers/platform/msm/gsi/gsihal/gsihal.h",
            "drivers/platform/msm/gsi/gsihal/gsihal_i.h",
            "drivers/platform/msm/gsi/gsihal/gsihal_reg.c",
            "drivers/platform/msm/gsi/gsihal/gsihal_reg.h",
            "drivers/platform/msm/gsi/gsihal/gsihal_reg_i.h",
        ],
        kconfig = "config/Kconfig",
        defconfig = include_defconfig,
        conditional_srcs = {
            "CONFIG_IPA_EMULATION": {
                True: [
                    "drivers/platform/msm/gsi/gsi_emulation.c",
                ],
            },
        },
        local_defines = [
            "GSI_TRACE_INCLUDE_PATH={}/drivers/platform/msm/gsi".format(include_base),
        ],
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = [
            ":gsi_headers",
            ":include_headers",
            "//msm-kernel:all_headers",
        ],
    )
    mod_list.append("{}_gsim".format(kernel_build_variant))

    ddk_module(
        name = "{}_ipam".format(kernel_build_variant),
        out = "ipam.ko",
        srcs = [
            "drivers/platform/msm/ipa/ipa_v3/ipa.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_client.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_debugfs.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_elf_dump.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_defs.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_dma.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_dp.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_emulation_stubs.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_eth_i.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_flt.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_hdr.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_hw_stats.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_elf_dump.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_interrupts.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_intf.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_mhi.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_nat.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_odl.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_odl.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_opt_log.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_opt_log.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_pm.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_pm.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_qdss.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_qmi_service.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_qmi_service.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_qmi_service_v01.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_rt.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_stats.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_stats.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_trace.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc_holb_monitor.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc_holb_monitor.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc_mhi.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc_ntn.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc_offload_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_uc_wdi.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_utils.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_wdi3_i.c",
            "drivers/platform/msm/ipa/ipa_v3/ipa_wigig_i.c",
            "drivers/platform/msm/ipa/ipa_v3/rmnet_ctl_ipa.c",
            "drivers/platform/msm/ipa/ipa_v3/rmnet_ipa.c",
            "drivers/platform/msm/ipa/ipa_v3/rmnet_ipa_fd_ioctl.c",
            "drivers/platform/msm/ipa/ipa_v3/rmnet_ll_ipa.c",
            "drivers/platform/msm/ipa/ipa_v3/teth_bridge.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_eth.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_gsb.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_mhi_client.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_uc_offload.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_usb.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_wdi3.c",
            "drivers/platform/msm/ipa/ipa_clients/ipa_wigig.c",
            "drivers/platform/msm/ipa/ipa_clients/rndis_ipa.h",
            "drivers/platform/msm/ipa/ipa_clients/rndis_ipa_trace.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal.c",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_fltrt.c",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_fltrt.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_fltrt_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_hw_stats.c",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_hw_stats.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_hw_stats_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_nat.c",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_nat.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_nat_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_reg.c",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_reg.h",
            "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_reg_i.h",
            "drivers/platform/msm/ipa/ipa_v3/ipa_tsp.h",
            "drivers/platform/msm/ipa/ipa_clients/ecm_ipa.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/gsi_hwio.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/gsi_hwio_def.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_access_control.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_gcc_hwio.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_gcc_hwio_def.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_hw_common_ex.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_hwio.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_hwio_def.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_pkt_cntxt.h",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_reg_dump.c",
            "drivers/platform/msm/ipa/ipa_v3/dump/ipa5.5/ipa_reg_dump.h",
            "drivers/platform/msm/ipa/ipa_common_i.h",
            "drivers/platform/msm/ipa/ipa_rm.c",
            "drivers/platform/msm/ipa/ipa_rm_dependency_graph.c",
            "drivers/platform/msm/ipa/ipa_rm_dependency_graph.h",
            "drivers/platform/msm/ipa/ipa_rm_i.h",
            "drivers/platform/msm/ipa/ipa_rm_inactivity_timer.c",
            "drivers/platform/msm/ipa/ipa_rm_peers_list.c",
            "drivers/platform/msm/ipa/ipa_rm_peers_list.h",
            "drivers/platform/msm/ipa/ipa_rm_resource.c",
            "drivers/platform/msm/ipa/ipa_rm_resource.h",
            "drivers/platform/msm/ipa/ipa_uc_offload_common_i.h",
        ],
        kconfig = "config/Kconfig",
        defconfig = include_defconfig,
        conditional_srcs = {
            "CONFIG_IPA3_MHI_PRIME_MANAGER": {
                True: [
                    "drivers/platform/msm/ipa/ipa_v3/ipa_mpm.c",
                ],
            },
            "CONFIG_IPA3_MHI_PROXY": {
                True: [
                    "drivers/platform/msm/ipa/ipa_v3/ipa_mhi_proxy.h",
                    "drivers/platform/msm/ipa/ipa_v3/ipa_mhi_proxy.c",
                ],
            },
            "CONFIG_IPA_TSP": {
                True: [
                    "drivers/platform/msm/ipa/ipa_v3/ipahal/ipahal_tsp.c",
                    "drivers/platform/msm/ipa/ipa_v3/ipa_tsp.c",
                ],
            },
            "CONFIG_ECM_IPA": {
                True: [
                    "drivers/platform/msm/ipa/ipa_clients/ecm_ipa.c",
                ],
            },
            "CONFIG_RNDIS_IPA": {
                True: [
                    "drivers/platform/msm/ipa/ipa_clients/rndis_ipa.c",
                ],
            },
            "CONFIG_IPA_UT": {
                True: [
                    "drivers/platform/msm/ipa/test/ipa_ut_framework.c",
                    "drivers/platform/msm/ipa/test/ipa_ut_framework.h",
                    "drivers/platform/msm/ipa/test/ipa_ut_i.h",
                    "drivers/platform/msm/ipa/test/ipa_ut_suite_list.h",
                    "drivers/platform/msm/ipa/test/ipa_test_example.c",
                    "drivers/platform/msm/ipa/test/ipa_test_mhi.c",
                    "drivers/platform/msm/ipa/test/ipa_test_dma.c",
                    "drivers/platform/msm/ipa/test/ipa_test_hw_stats.c",
                    "drivers/platform/msm/ipa/test/ipa_pm_ut.c",
                    "drivers/platform/msm/ipa/test/ipa_test_wdi3.c",
                    "drivers/platform/msm/ipa/test/ipa_test_ntn.c",
                ],
            },
        },
        local_defines = [
            "GSI_TRACE_INCLUDE_PATH={}/drivers/platform/msm/gsi".format(include_base),
            "IPA_TRACE_INCLUDE_PATH={}/drivers/platform/msm/ipa/ipa_v3".format(include_base),
            "RNDIS_TRACE_INCLUDE_PATH={}/drivers/platform/msm/ipa/ipa_clients".format(include_base),
        ],
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        deps = [
            ":{}_config_headers".format(variant),
            ":gsi_headers",
            ":include_headers",
            ":ipa_headers",
            ":ipa_clients",
            "//msm-kernel:all_headers",
            ":{}_gsim".format(kernel_build_variant),
            "//vendor/qcom/opensource/datarmnet-ext/mem:{}_rmnet_mem".format(kernel_build_variant),
        ],
    )
    mod_list.append("{}_ipam".format(kernel_build_variant))

    ddk_module(
        name = "{}_ipanetm".format(kernel_build_variant),
        out = "ipanetm.ko",
        srcs = [
            "drivers/platform/msm/ipa/ipa_v3/ipa_net.c",
        ],
        kconfig = "config/Kconfig",
        defconfig = include_defconfig,
        kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
        local_defines = [
            "RNDIS_TRACE_INCLUDE_PATH={}/drivers/platform/msm/ipa/ipa_clients".format(include_base),
        ],
        deps = [
            ":{}_config_headers".format(variant),
            ":{}_ipam".format(kernel_build_variant),
            ":gsi_headers",
            ":include_headers",
            ":ipa_headers",
            ":ipa_clients",
            "//msm-kernel:all_headers",
        ],
    )
    mod_list.append("{}_ipanetm".format(kernel_build_variant))

    if variant == "consolidate":
        ddk_module(
            name = "{}_ipatestm".format(kernel_build_variant),
            out = "ipatestm.ko",
            srcs = [
                "drivers/platform/msm/ipa/ipa_test_module/ipa_rm_ut.c",
                "drivers/platform/msm/ipa/ipa_test_module/ipa_rm_ut.h",
                "drivers/platform/msm/ipa/ipa_test_module/ipa_test_module.h",
                "drivers/platform/msm/ipa/ipa_test_module/ipa_test_module_impl.c",
                "drivers/platform/msm/ipa/ipa_test_module/ipa_test_module_tsp.h",
            ],
            kconfig = "config/Kconfig",
            defconfig = include_defconfig,
            kernel_build = "//msm-kernel:{}".format(kernel_build_variant),
            deps = [
                ":consolidate_config_headers",
                ":{}_ipam".format(kernel_build_variant),
                ":gsi_headers",
                ":include_headers",
                ":ipa_headers",
                ":ipa_clients",
                "//msm-kernel:all_headers",
                ":{}_gsim".format(kernel_build_variant),
            ],
        )
        mod_list.append("{}_ipatestm".format(kernel_build_variant))

    copy_to_dist_dir(
        name = "{}_modules_dist".format(kernel_build_variant),
        data = mod_list,
        dist_dir = "out/target/product/{}/dlkm/lib/modules/".format(target),
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )
