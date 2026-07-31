def register_modules(registry):
    registry.register(
        name = "drivers/power/reset/qcom-dload-mode",
        out = "qcom-dload-mode.ko",
        config = "CONFIG_POWER_RESET_QCOM_DOWNLOAD_MODE",
        srcs = [
            # do not sort
            "drivers/power/reset/qcom-dload-mode.c",
        ],
        deps = [
            # do not sort
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
        name = "drivers/power/reset/qcom-pon",
        out = "qcom-pon.ko",
        config = "CONFIG_POWER_RESET_QCOM_PON",
        srcs = [
            # do not sort
            "drivers/power/reset/qcom-pon.c",
        ],
        deps = [
            # do not sort
            "drivers/power/reset/reboot-mode",
        ],
    )

    registry.register(
        name = "drivers/power/reset/qcom-reboot-reason",
        out = "qcom-reboot-reason.ko",
        config = "CONFIG_POWER_RESET_QCOM_REBOOT_REASON",
        srcs = [
            # do not sort
            "drivers/power/reset/qcom-reboot-reason.c",
        ],
    )

    registry.register(
        name = "drivers/power/reset/reboot-mode",
        out = "reboot-mode.ko",
        config = "CONFIG_REBOOT_MODE",
        srcs = [
            # do not sort
            "drivers/power/reset/reboot-mode.c",
        ],
    )

    registry.register(
        name = "drivers/power/reset/msm-vm-poweroff",
        out = "msm-vm-poweroff.ko",
        config = "CONFIG_POWER_RESET_QCOM_VM",
        srcs = [
            # do not sort
            "drivers/power/reset/msm-vm-poweroff.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/qcom_wdt_core",
            "drivers/soc/qcom/qcom_soc_wdt",
        ],
    )
