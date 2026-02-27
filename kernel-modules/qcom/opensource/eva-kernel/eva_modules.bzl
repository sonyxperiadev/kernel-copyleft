load(":eva_module_build.bzl", "create_module_registry")

EVA_KERNEL_ROOT = "eva-kernel"

eva_modules = create_module_registry([":eva_drivers_headers"])
register_eva_module = eva_modules.register

register_eva_module(
    name = "msm-eva",
    path = "msm",
    srcs = [
        "eva/cvp.c",
        "eva/cvp_core_hfi.c",
        "eva/cvp_dump.c",
        "eva/cvp_fw_load.c",
        "eva/cvp_hfi.c",
        "eva/cvp_power.c",
        "eva/cvp_smem.c",
        "eva/hfi_packetization.c",
        "eva/hfi_response_handler.c",
        "eva/msm_cvp.c",
        "eva/msm_cvp_buf.c",
        "eva/msm_cvp_clocks.c",
        "eva/msm_cvp_common.c",
        "eva/msm_cvp_core.c",
        "eva/msm_cvp_debug.c",
        "eva/msm_cvp_dsp.c",
        "eva/msm_cvp_ioctl.c",
        "eva/msm_cvp_platform.c",
        "eva/msm_cvp_res_parse.c",
        "eva/msm_cvp_synx.c",
        "eva/vm/cvp_vm_main.c",
        "eva/vm/cvp_vm_msgq.c",
        "eva/vm/cvp_vm_resource.c",
    ],
    config_deps = {
       "TARGET_SYNX_ENABLE": [
           "//vendor/qcom/opensource/synx-kernel:synx_headers",
           "//vendor/qcom/opensource/synx-kernel:%b_modules"
        ],
        "TARGET_DSP_ENABLE": [
             "//vendor/qcom/opensource/dsp-kernel:%b_frpc-adsprpc"
        ],
    },
)
