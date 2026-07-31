def register_modules(registry):
    registry.register(
        name = "drivers/perf/qcom_llcc_pmu",
        out = "qcom_llcc_pmu.ko",
        config = "CONFIG_QCOM_LLCC_PMU",
        srcs = [
            # do not sort
            "drivers/perf/qcom_llcc_pmu.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/cpu_phys_log_map",
        ],
    )

    registry.register(
        name = "drivers/perf/arm_smmu_pmu",
        out = "arm_smmu_pmu.ko",
        config = "CONFIG_ARM_SMMU_PMU",
        srcs = [
            # do not sort
            "drivers/perf/arm_smmu_pmu.c",
        ],
        deps = [
            # do not sort
            "drivers/iommu/arm/arm-smmu/arm_smmu",
            "drivers/firmware/qcom/qcom-scm",
        ],
    )
