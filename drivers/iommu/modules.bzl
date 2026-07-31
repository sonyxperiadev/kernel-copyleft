load(":drivers/iommu/arm/arm-smmu/modules.bzl", register_arm_arm_smmu = "register_modules")

def register_modules(registry):
    register_arm_arm_smmu(registry)

    registry.register(
        name = "drivers/iommu/iommu-logger",
        out = "iommu-logger.ko",
        config = "CONFIG_QTI_IOMMU_SUPPORT",
        srcs = [
            # do not sort
            "drivers/iommu/iommu-logger.c",
            "drivers/iommu/iommu-logger.h",
        ],
    )

    registry.register(
        name = "drivers/iommu/msm_dma_iommu_mapping",
        out = "msm_dma_iommu_mapping.ko",
        config = "CONFIG_QCOM_LAZY_MAPPING",
        srcs = [
            # do not sort
            "drivers/iommu/msm_dma_iommu_mapping.c",
        ],
    )

    registry.register(
        name = "drivers/iommu/qcom_iommu_debug",
        out = "qcom_iommu_debug.ko",
        config = "CONFIG_QCOM_IOMMU_DEBUG",
        srcs = [
            # do not sort
            "drivers/iommu/qcom-iommu-debug-user.c",
            "drivers/iommu/qcom-iommu-debug.c",
            "drivers/iommu/qcom-iommu-debug.h",
        ],
        deps = [
            # do not sort
            "drivers/iommu/qcom_iommu_util",
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/iommu/qcom_iommu_util",
        out = "qcom_iommu_util.ko",
        config = "CONFIG_QCOM_IOMMU_UTIL",
        srcs = [
            # do not sort
            "drivers/iommu/qcom-dma-iommu-generic.h",
            "drivers/iommu/qcom-io-pgtable-alloc.h",
            "drivers/iommu/qcom-iommu-util.c",
        ],
        conditional_srcs = {
            "CONFIG_IOMMU_IO_PGTABLE_FAST": {
                True: [
                    # do not sort
                    "drivers/iommu/qcom-dma-iommu-generic.c",
                    "drivers/iommu/io-pgtable-fast.c",
                    "drivers/iommu/dma-mapping-fast.c",
                ],
            },
            "CONFIG_IOMMU_IO_PGTABLE_LPAE": {
                True: [
                    # do not sort
                    "drivers/iommu/qcom-io-pgtable-arm.c",
                    "drivers/iommu/qcom-io-pgtable-alloc.c",
                ],
            },
        },
        deps = [
            # do not sort
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )
