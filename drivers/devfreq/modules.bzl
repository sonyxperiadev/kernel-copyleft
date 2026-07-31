def register_modules(registry):
    registry.register(
        name = "drivers/devfreq/governor_gpubw_mon",
        out = "governor_gpubw_mon.ko",
        config = "CONFIG_DEVFREQ_GOV_QCOM_GPUBW_MON",
        srcs = [
            # do not sort
            "drivers/devfreq/governor_gpubw_mon.c",
        ],
    )

    registry.register(
        name = "drivers/devfreq/governor_msm_adreno_tz",
        out = "governor_msm_adreno_tz.ko",
        config = "CONFIG_DEVFREQ_GOV_QCOM_ADRENO_TZ",
        srcs = [
            # do not sort
            "drivers/devfreq/governor_msm_adreno_tz.c",
        ],
        deps = [
            # do not sort
            "drivers/firmware/qcom/qcom-scm",
            "drivers/virt/gunyah/gh_rm_drv",
            "drivers/virt/gunyah/gh_msgq",
            "drivers/virt/gunyah/gh_dbl",
            "arch/arm64/gunyah/gh_arm_drv",
        ],
    )

    registry.register(
        name = "drivers/devfreq/governor_msm_adreno_ro",
        out = "governor_msm_adreno_ro.ko",
        config = "CONFIG_DEVFREQ_GOV_QCOM_ADRENO_RO",
        srcs = [
            # do not sort
            "drivers/devfreq/governor_msm_adreno_ro.c",
        ],
    )
