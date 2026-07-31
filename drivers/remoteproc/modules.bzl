def register_modules(registry):
    registry.register(
        name = "drivers/remoteproc/qcom_pil_info",
        out = "qcom_pil_info.ko",
        config = "CONFIG_QCOM_PIL_INFO",
        srcs = [
            # do not sort
            "drivers/remoteproc/qcom_pil_info.c",
            "drivers/remoteproc/qcom_pil_info.h",
        ],
    )

    registry.register(
        name = "drivers/remoteproc/qcom_q6v5",
        out = "qcom_q6v5.ko",
        config = "CONFIG_QCOM_Q6V5_COMMON",
        srcs = [
            # do not sort
            "drivers/remoteproc/qcom_common.h",
            "drivers/remoteproc/qcom_q6v5.c",
            "drivers/remoteproc/qcom_q6v5.h",
        ],
        deps = [
            # do not sort
            "drivers/remoteproc/qcom_sysmon",
            "drivers/soc/qcom/qcom_aoss",
            "drivers/soc/qcom/qmi_helpers",
            "drivers/remoteproc/rproc_qcom_common",
            "drivers/rpmsg/qcom_smd",
            "drivers/rpmsg/qcom_glink_smem",
            "drivers/rpmsg/qcom_glink",
            "kernel/trace/qcom_ipc_logging",
            "drivers/soc/qcom/smem",
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
        name = "drivers/remoteproc/qcom_q6v5_pas",
        out = "qcom_q6v5_pas.ko",
        config = "CONFIG_QCOM_Q6V5_PAS",
        srcs = [
            # do not sort
            "drivers/remoteproc/qcom_common.h",
            "drivers/remoteproc/qcom_pil_info.h",
            "drivers/remoteproc/qcom_q6v5.h",
            "drivers/remoteproc/qcom_q6v5_pas.c",
        ],
        deps = [
            # do not sort
            "drivers/remoteproc/qcom_pil_info",
            "drivers/soc/qcom/mdt_loader",
            "drivers/soc/qcom/qcom_ramdump",
            "drivers/remoteproc/qcom_q6v5",
            "drivers/remoteproc/qcom_sysmon",
            "drivers/soc/qcom/qcom_aoss",
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
        name = "drivers/remoteproc/qcom_spss",
        out = "qcom_spss.ko",
        config = "CONFIG_QCOM_SPSS",
        srcs = [
            # do not sort
            "drivers/remoteproc/qcom_common.h",
            "drivers/remoteproc/qcom_spss.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/mdt_loader",
            "drivers/remoteproc/qcom_sysmon",
            "drivers/soc/qcom/qcom_aoss",
            "drivers/rpmsg/qcom_glink_spss",
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
        name = "drivers/remoteproc/qcom_sysmon",
        out = "qcom_sysmon.ko",
        config = "CONFIG_QCOM_SYSMON",
        srcs = [
            # do not sort
            "drivers/remoteproc/qcom_common.h",
            "drivers/remoteproc/qcom_sysmon.c",
        ],
        deps = [
            # do not sort
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
        name = "drivers/remoteproc/rproc_qcom_common",
        out = "rproc_qcom_common.ko",
        config = "CONFIG_QCOM_RPROC_COMMON",
        srcs = [
            # do not sort
            "drivers/remoteproc/qcom_common.c",
            "drivers/remoteproc/qcom_common.h",
            "drivers/remoteproc/qcom_tracepoints.c",
        ],
        deps = [
            # do not sort
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
