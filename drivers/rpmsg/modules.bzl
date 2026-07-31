def register_modules(registry):
    registry.register(
        name = "drivers/rpmsg/glink_pkt",
        out = "glink_pkt.ko",
        config = "CONFIG_QCOM_GLINK_PKT",
        srcs = [
            # do not sort
            "drivers/rpmsg/glink_pkt.c",
            "drivers/rpmsg/qcom_glink_native.h",
        ],
        deps = [
            # do not sort
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
        name = "drivers/rpmsg/qcom_glink",
        out = "qcom_glink.ko",
        config = "CONFIG_RPMSG_QCOM_GLINK",
        srcs = [
            # do not sort
            "drivers/rpmsg/qcom_glink_memshare.c",
            "drivers/rpmsg/qcom_glink_native.c",
            "drivers/rpmsg/qcom_glink_native.h",
            "drivers/rpmsg/qcom_glink_ssr.c",
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
        name = "drivers/rpmsg/qcom_glink_smem",
        out = "qcom_glink_smem.ko",
        config = "CONFIG_RPMSG_QCOM_GLINK_SMEM",
        srcs = [
            # do not sort
            "drivers/rpmsg/qcom_glink_native.h",
            "drivers/rpmsg/qcom_glink_smem.c",
        ],
        deps = [
            # do not sort
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
        name = "drivers/rpmsg/qcom_glink_spss",
        out = "qcom_glink_spss.ko",
        config = "CONFIG_RPMSG_QCOM_GLINK_SPSS",
        srcs = [
            # do not sort
            "drivers/rpmsg/qcom_glink_native.h",
            "drivers/rpmsg/qcom_glink_spss.c",
        ],
        deps = [
            # do not sort
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
        name = "drivers/rpmsg/qcom_smd",
        out = "qcom_smd.ko",
        config = "CONFIG_RPMSG_QCOM_SMD",
        srcs = [
            # do not sort
            "drivers/rpmsg/qcom_smd.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/smem",
        ],
    )

    registry.register(
        name = "drivers/rpmsg/qcom_glink_cma",
        out = "qcom_glink_cma.ko",
        config = "CONFIG_RPMSG_QCOM_GLINK_CMA",
        srcs = [
            # do not sort
            "drivers/rpmsg/qcom_glink_cma.h",
            "drivers/rpmsg/qcom_glink_native.h",
            "drivers/rpmsg/virtio_glink_cma.c",
            "drivers/rpmsg/qcom_glink_cma_core.c",
        ],
        deps = [
            # do not sort
            "kernel/trace/qcom_ipc_logging",
            "drivers/rpmsg/qcom_glink",
            "drivers/remoteproc/rproc_qcom_common",
        ],
    )

    registry.register(
        name = "drivers/rpmsg/qcom_glink_rpm",
        out = "qcom_glink_rpm.ko",
        config = "CONFIG_RPMSG_QCOM_GLINK_RPM",
        srcs = [
            # do not sort
            "drivers/rpmsg/qcom_glink_native.h",
            "drivers/rpmsg/qcom_glink_rpm.c",
        ],
        deps = [
            # do not sort
            "drivers/rpmsg/qcom_glink",
            "kernel/trace/qcom_ipc_logging",
            "drivers/soc/qcom/debug_symbol",
        ],
    )

    registry.register(
        name = "drivers/rpmsg/rpm-smd",
        out = "rpm-smd.ko",
        config = "CONFIG_MSM_RPM_SMD",
        srcs = [
            # do not sort
            "drivers/rpmsg/rpm-smd.c",
        ],
        deps = [
            # do not sort
            "kernel/trace/qcom_ipc_logging",
            "drivers/rpmsg/qcom_glink",
            "drivers/rpmsg/qcom_glink_smem",
            "drivers/rpmsg/qcom_glink_spss",
            "drivers/rpmsg/qcom_glink_rpm",
            "drivers/irqchip/qcom-mpm",
        ],
    )
