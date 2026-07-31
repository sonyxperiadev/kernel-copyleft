def register_modules(registry):
    registry.register(
        name = "drivers/regulator/debug-regulator",
        out = "debug-regulator.ko",
        config = "CONFIG_REGULATOR_DEBUG_CONTROL",
        srcs = [
            # do not sort
            "drivers/regulator/debug-regulator.c",
        ],
    )

    registry.register(
        name = "drivers/regulator/proxy-consumer",
        out = "proxy-consumer.ko",
        config = "CONFIG_REGULATOR_PROXY_CONSUMER",
        srcs = [
            # do not sort
            "drivers/regulator/proxy-consumer.c",
        ],
    )

    registry.register(
        name = "drivers/regulator/qcom-amoled-regulator",
        out = "qcom-amoled-regulator.ko",
        config = "CONFIG_REGULATOR_QCOM_AMOLED",
        srcs = [
            # do not sort
            "drivers/regulator/qcom-amoled-regulator.c",
        ],
        deps = [
            # do not sort
            "drivers/regulator/debug-regulator",
        ],
    )

    registry.register(
        name = "drivers/regulator/qti-fixed-regulator",
        out = "qti-fixed-regulator.ko",
        config = "CONFIG_REGULATOR_QTI_FIXED_VOLTAGE",
        srcs = [
            # do not sort
            "drivers/regulator/qti_fixed_regulator.c",
        ],
        copts = ["-DQTI_FIXED_REGULATOR"],
        deps = [
            # do not sort
            "drivers/regulator/debug-regulator",
            "drivers/regulator/proxy-consumer",
        ],
    )

    registry.register(
        name = "drivers/regulator/qti-ocp-notifier",
        out = "qti-ocp-notifier.ko",
        config = "CONFIG_REGULATOR_QTI_OCP_NOTIFIER",
        srcs = [
            # do not sort
            "drivers/regulator/qti-ocp-notifier.c",
        ],
        deps = [
            # do not sort
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
        name = "drivers/regulator/rpmh-regulator",
        out = "rpmh-regulator.ko",
        config = "CONFIG_REGULATOR_RPMH",
        srcs = [
            # do not sort
            "drivers/regulator/rpmh-regulator.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/regulator/debug-regulator",
            "drivers/regulator/proxy-consumer",
            "drivers/soc/qcom/crm-v2",
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
        name = "drivers/regulator/rpm-smd-regulator",
        out = "rpm-smd-regulator.ko",
        config = "CONFIG_REGULATOR_RPM_SMD",
        srcs = [
            # do not sort
            "drivers/regulator/rpm-smd-regulator.c",
        ],
        deps = [
            # do not sort
            "drivers/rpmsg/rpm-smd",
            "drivers/regulator/debug-regulator",
            "drivers/regulator/proxy-consumer",
            "drivers/rpmsg/qcom_glink",
            "drivers/rpmsg/qcom_glink_smem",
            "drivers/rpmsg/qcom_glink_spss",
            "drivers/rpmsg/qcom_glink_rpm",
            "drivers/rpmsg/glink_qcom",
        ],
    )

    registry.register(
        name = "drivers/regulator/stub-regulator",
        out = "stub-regulator.ko",
        config = "CONFIG_REGULATOR_STUB",
        srcs = [
            # do not sort
            "drivers/regulator/stub-regulator.c",
        ],
        deps = [
            # do not sort
            "drivers/regulator/debug-regulator",
        ],
    )

    registry.register(
        name = "drivers/regulator/virtio_regulator",
        out = "virtio_regulator.ko",
        config = "CONFIG_VIRTIO_REGULATOR",
        srcs = [
            # do not sort
            "drivers/regulator/virtio_regulator.c",
        ],
        deps = [
            # do not sort
            "drivers/regulator/debug-regulator",
        ],
    )

    registry.register(
        name = "drivers/regulator/refgen",
        out = "refgen.ko",
        config = "CONFIG_REGULATOR_REFGEN",
        srcs = [
            # do not sort
            "drivers/regulator/refgen.c",
        ],
        deps = [
            # do not sort
            "drivers/regulator/debug-regulator",
            "drivers/regulator/proxy-consumer",
        ],
    )

    registry.register(
        name = "drivers/regulator/qpnp-lcdb-regulator",
        out = "qpnp-lcdb-regulator.ko",
        config = "CONFIG_REGULATOR_QPNP_LCDB",
        srcs = [
            # do not sort
            "drivers/regulator/qpnp-lcdb-regulator.c",
        ],
        deps = [
            # do not sort
        ],
    )

    registry.register(
        name = "drivers/regulator/qcom-pm8008-regulator",
        out = "qcom-pm8008-regulator.ko",
        config = "CONFIG_REGULATOR_QCOM_PM8008",
        srcs = [
            # do not sort
            "drivers/regulator/qcom-pm8008-regulator.c",
        ],
        deps = [
            # do not sort
        ],
    )

    registry.register(
        name = "drivers/regulator/wl28681c-regulator",
        out = "wl28681c-regulator.ko",
        config = "CONFIG_REGULATOR_WL28681C",
        srcs = [
            # do not sort
            "drivers/regulator/wl28681c-regulator.c",
            "drivers/regulator/wl28681c-regulator.h",
        ],
        deps = [
            # do not sort
        ],
    )
