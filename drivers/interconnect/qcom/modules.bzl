def register_modules(registry):
    registry.register(
        name = "drivers/interconnect/qcom/icc-bcm-voter",
        out = "icc-bcm-voter.ko",
        config = "CONFIG_INTERCONNECT_QCOM_BCM_VOTER",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/bcm-voter.c",
            "drivers/interconnect/qcom/bcm-voter.h",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/trace.h",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
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
        name = "drivers/interconnect/qcom/icc-debug",
        out = "icc-debug.ko",
        config = "CONFIG_INTERCONNECT_QCOM_DEBUG",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/icc-debug.c",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/icc-rpmh",
        out = "icc-rpmh.ko",
        config = "CONFIG_INTERCONNECT_QCOM_RPMH",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/bcm-voter.h",
            "drivers/interconnect/qcom/icc-debug.h",
            "drivers/interconnect/qcom/icc-rpmh.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/icc-rpm",
        out = "icc-rpm.ko",
        config = "CONFIG_INTERCONNECT_QCOM_RPM",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/icc-debug.h",
            "drivers/interconnect/qcom/icc-rpm.c",
            "drivers/interconnect/qcom/icc-rpm.h",
            "drivers/interconnect/qcom/qnoc-qos-rpm.h",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/qnoc-qos-rpm",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/rpmsg/rpm-smd",
            "drivers/soc/qcom/smem",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-canoe",
        out = "qnoc-canoe.ko",
        config = "CONFIG_INTERCONNECT_QCOM_CANOE",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/canoe.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-chora",
        out = "qnoc-chora.ko",
        config = "CONFIG_INTERCONNECT_QCOM_CHORA",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/chora.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-x1p42100",
        out = "qnoc-x1p42100.ko",
        config = "CONFIG_INTERCONNECT_QCOM_X1P42100",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/x1p42100.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-x1e80100",
        out = "qnoc-x1e80100.ko",
        config = "CONFIG_INTERCONNECT_QCOM_X1E80100",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/x1e80100.c",
            "drivers/interconnect/qcom/x1e80100.h",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
            "drivers/interconnect/qcom/bcm-voter.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-malabar",
        out = "qnoc-malabar.ko",
        config = "CONFIG_INTERCONNECT_QCOM_MALABAR",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/malabar.c",
            "drivers/interconnect/qcom/icc-rpm.h",
            "drivers/interconnect/qcom/qnoc-qos-rpm.h",
            "drivers/interconnect/qcom/rpm-ids.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos-rpm",
            "drivers/interconnect/qcom/icc-rpm",
            "drivers/interconnect/qcom/icc-debug",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-yupik",
        out = "qnoc-yupik.ko",
        config = "CONFIG_INTERCONNECT_QCOM_YUPIK",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/yupik.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-qos",
        out = "qnoc-qos.ko",
        config = "CONFIG_INTERCONNECT_QCOM_QOS",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.c",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-qos-rpm",
        out = "qnoc-qos-rpm.ko",
        config = "CONFIG_INTERCONNECT_QCOM_QOS_RPM",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/icc-rpm.h",
            "drivers/interconnect/qcom/qnoc-qos-rpm.c",
            "drivers/interconnect/qcom/qnoc-qos-rpm.h",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-bengal",
        out = "qnoc-bengal.ko",
        config = "CONFIG_INTERCONNECT_QCOM_BENGAL",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/bengal.c",
            "drivers/interconnect/qcom/icc-rpm.h",
            "drivers/interconnect/qcom/rpm-ids.h",
            "drivers/interconnect/qcom/qnoc-qos-rpm.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos-rpm",
            "drivers/interconnect/qcom/icc-rpm",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/soc/qcom/smem",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-monaco",
        out = "qnoc-monaco.ko",
        config = "CONFIG_INTERCONNECT_QCOM_MONACO",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/monaco.c",
            "drivers/interconnect/qcom/icc-rpm.h",
            "drivers/interconnect/qcom/rpm-ids.h",
            "drivers/interconnect/qcom/qnoc-qos-rpm.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos-rpm",
            "drivers/interconnect/qcom/icc-rpm",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/soc/qcom/smem",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-sun",
        out = "qnoc-sun.ko",
        config = "CONFIG_INTERCONNECT_QCOM_SUN",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
            "drivers/interconnect/qcom/sun.c",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/socinfo",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
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
        name = "drivers/interconnect/qcom/qnoc-vienna",
        out = "qnoc-vienna.ko",
        config = "CONFIG_INTERCONNECT_QCOM_VIENNA",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
            "drivers/interconnect/qcom/vienna.c",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/socinfo",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
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
        name = "drivers/interconnect/qcom/qnoc-alor",
        out = "qnoc-alor.ko",
        config = "CONFIG_INTERCONNECT_QCOM_ALOR",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/alor.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-seraph",
        out = "qnoc-seraph.ko",
        config = "CONFIG_INTERCONNECT_QCOM_SERAPH",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/seraph.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )

    registry.register(
        name = "drivers/interconnect/qcom/qnoc-pikachu",
        out = "qnoc-pikachu.ko",
        config = "CONFIG_INTERCONNECT_QCOM_PIKACHU",
        srcs = [
            # do not sort
            "drivers/interconnect/qcom/pikachu.c",
            "drivers/interconnect/qcom/icc-rpmh.h",
            "drivers/interconnect/qcom/qnoc-qos.h",
        ],
        deps = [
            # do not sort
            "drivers/interconnect/qcom/qnoc-qos",
            "drivers/interconnect/qcom/icc-rpmh",
            "drivers/soc/qcom/socinfo",
            "drivers/interconnect/qcom/icc-debug",
            "drivers/interconnect/qcom/icc-bcm-voter",
            "drivers/soc/qcom/qcom_rpmh",
            "drivers/soc/qcom/cmd-db",
            "drivers/soc/qcom/smem",
            "drivers/soc/qcom/crm-v2",
            "kernel/trace/qcom_ipc_logging",
        ],
    )
