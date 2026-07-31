def register_modules(registry):
    registry.register(
        name = "drivers/input/misc/pm8941-pwrkey",
        out = "pm8941-pwrkey.ko",
        config = "CONFIG_INPUT_PM8941_PWRKEY",
        srcs = [
            # do not sort
            "drivers/input/misc/pm8941-pwrkey.c",
        ],
    )

    registry.register(
        name = "drivers/input/misc/qcom-hv-haptics",
        out = "qcom-hv-haptics.ko",
        config = "CONFIG_INPUT_QCOM_HV_HAPTICS",
        srcs = [
            # do not sort
            "drivers/input/misc/qcom-hv-haptics.c",
            "drivers/input/misc/qcom-hv-haptics-debugfs.h",
        ],
        deps = [
            # do not sort
            "drivers/power/supply/qti_battery_charger",
            "drivers/soc/qcom/panel_event_notifier",
            "drivers/soc/qcom/qti_pmic_glink",
            "drivers/soc/qcom/pdr_interface",
            "drivers/soc/qcom/qmi_helpers",
            "drivers/remoteproc/rproc_qcom_common",
            "drivers/rpmsg/qcom_smd",
            "drivers/rpmsg/qcom_glink_smem",
            "drivers/rpmsg/qcom_glink",
            "kernel/trace/qcom_ipc_logging",
            "drivers/soc/qcom/minidump",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/debug_symbol",
            "drivers/dma-buf/heaps/qcom_dma_heaps",
            "drivers/iommu/msm_dma_iommu_mapping",
            "drivers/soc/qcom/mem_buf/mem_buf_dev",
            "drivers/soc/qcom/secure_buffer",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/input/misc/cs40l26-core",
        out = "cs40l26-core.ko",
        config = "CONFIG_INPUT_CS40L26",
        srcs = [
            # do not sort
            "drivers/input/misc/cs40l26.c",
            "drivers/input/misc/cs40l26-tables.c",
            "drivers/input/misc/cs40l26-sysfs.c",
            "drivers/input/misc/cs40l26-debugfs.c",
            "drivers/input/misc/cs40l26-i2c.c",
            "drivers/firmware/cirrus/cl_dsp.c",
            "drivers/firmware/cirrus/cl_dsp-debugfs.c",
        ],
    )
