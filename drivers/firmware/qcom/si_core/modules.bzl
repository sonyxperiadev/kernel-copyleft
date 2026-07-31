def register_modules(registry):
    registry.register(
        name = "drivers/firmware/qcom/si_core/si_core_module",
        out = "si_core_module.ko",
        config = "CONFIG_QCOM_SI_CORE",
        srcs = [
            # do not sort
            "drivers/firmware/qcom/si_core/qcom_scm_invoke.c",
            "drivers/firmware/qcom/si_core/si_core.c",
            "drivers/firmware/qcom/si_core/si_core.h",
            "drivers/firmware/qcom/si_core/trace_si_core.h",
            "drivers/firmware/qcom/si_core/si_core_adci.h",
            "drivers/firmware/qcom/si_core/si_core_async.c",
            "drivers/firmware/qcom/si_core/si_core_helpers.c",
            "drivers/firmware/qcom/si_core/xts/mem-object.c",
            "drivers/firmware/qcom/si_core/xts/mem-object.h",
        ],
        conditional_srcs = {
            "CONFIG_QCOM_SI_CORE_ADCI": {
                True: [
                    # do not sort
                    "drivers/firmware/qcom/si_core/si_core_adci.c",
                ],
            },
            "CONFIG_QCOM_SI_CORE_WQ": {
                True: [
                    # do not sort
                    "drivers/firmware/qcom/si_core/si_core_wq.c",
                ],
            },
            "CONFIG_QCOM_SI_CORE_MEM_FFA": {
                True: [
                    # do not sort
                    "drivers/firmware/qcom/si_core/si_core_ffa.c",
                ],
            },
            "CONFIG_QCOM_SI_CORE_DOORBELL": {
                True: [
                    # do not sort
                    "drivers/firmware/qcom/si_core/si_core_irq.c",
                    "drivers/firmware/qcom/si_core/si_core_irq.h",
                ],
            },
        },
        deps = [
            # do not sort
            "drivers/firmware/arm_ffa_transport",
            "drivers/firmware/arm_ffa",
            "drivers/soc/qcom/mem_buf/mem_buf_dev",
            "drivers/soc/qcom/secure_buffer",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/misc/qseecom_proxy",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )
