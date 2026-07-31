def register_modules(registry):
    registry.register(
        name = "drivers/thermal/qcom/bcl_pmic5",
        out = "bcl_pmic5.ko",
        config = "CONFIG_QTI_BCL_PMIC5",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qti_bcl_pmic5.c",
            "drivers/thermal/qcom/qti_bcl_stats.c",
            "drivers/thermal/qcom/qti_bcl_common.h",
            "drivers/thermal/qcom/thermal_zone_internal.h",
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
        name = "drivers/thermal/qcom/bcl_soc",
        out = "bcl_soc.ko",
        config = "CONFIG_QTI_BCL_SOC_DRIVER",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/bcl_soc.c",
            "drivers/thermal/qcom/thermal_zone_internal.h",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/cpu_hotplug",
        out = "cpu_hotplug.ko",
        config = "CONFIG_QTI_CPU_HOTPLUG_COOLING_DEVICE",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/cpu_hotplug.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/ddr_cdev",
        out = "ddr_cdev.ko",
        config = "CONFIG_QTI_DDR_COOLING_DEVICE",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/ddr_cdev.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/max31760_fan",
        out = "max31760_fan.ko",
        config = "CONFIG_MAX31760_FAN_CONTROLLER",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/max31760_fan.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qcom-spmi-temp-alarm",
        out = "qcom-spmi-temp-alarm.ko",
        config = "CONFIG_QCOM_SPMI_TEMP_ALARM",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qcom-spmi-temp-alarm.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qcom_tsens",
        out = "qcom_tsens.ko",
        config = "CONFIG_QCOM_TSENS",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/thermal_zone_internal.h",
            "drivers/thermal/qcom/tsens-8960.c",
            "drivers/thermal/qcom/tsens-v0_1.c",
            "drivers/thermal/qcom/tsens-v1.c",
            "drivers/thermal/qcom/tsens-v2.c",
            "drivers/thermal/qcom/tsens.c",
            "drivers/thermal/qcom/tsens.h",
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
        name = "drivers/thermal/qcom/qti_cpufreq_cdev",
        out = "qti_cpufreq_cdev.ko",
        config = "CONFIG_QTI_CPUFREQ_CDEV",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qti_cpufreq_cdev.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qti_devfreq_cdev",
        out = "qti_devfreq_cdev.ko",
        config = "CONFIG_QTI_DEVFREQ_CDEV",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qti_devfreq_cdev.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qti_qmi_cdev",
        out = "qti_qmi_cdev.ko",
        config = "CONFIG_QTI_QMI_COOLING_DEVICE",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qmi_cooling.c",
            "drivers/thermal/qcom/thermal_mitigation_device_service_v01.c",
            "drivers/thermal/qcom/thermal_mitigation_device_service_v01.h",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/qmi_helpers",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qti_qmi_sensor_v2",
        out = "qti_qmi_sensor_v2.ko",
        config = "CONFIG_QTI_QMI_SENSOR_V2",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qmi_sensors.h",
            "drivers/thermal/qcom/qmi_sensors_v2.c",
            "drivers/thermal/qcom/thermal_sensor_service_v02.c",
            "drivers/thermal/qcom/thermal_sensor_service_v02.h",
            "drivers/thermal/qcom/thermal_zone_internal.h",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/qmi_helpers",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qti_userspace_cdev",
        out = "qti_userspace_cdev.ko",
        config = "CONFIG_QTI_USERSPACE_CDEV",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qti_userspace_cdev.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/thermal_config",
        out = "thermal_config.ko",
        config = "CONFIG_QTI_THERMALZONE_CONFIG_DEBUG",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/thermal_config.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/thermal_pause",
        out = "thermal_pause.ko",
        config = "CONFIG_QTI_CPU_PAUSE_COOLING_DEVICE",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/thermal_pause.c",
        ],
        deps = [
            # do not sort
            "kernel/sched/walt/sched-walt",
            "drivers/soc/qcom/socinfo",
            "drivers/soc/qcom/smem",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qti_thermal_vendor_hooks",
        out = "qti_thermal_vendor_hooks.ko",
        config = "CONFIG_QTI_THERMAL_VENDOR_HOOK",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qti_thermal_vendor_hooks.c",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qti_dynamic_hw_trip",
        out = "qti_dynamic_hw_trip.ko",
        config = "CONFIG_QTI_DYNAMIC_HW_THERMAL_TRIP",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qti_dynamic_hw_trip.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/dcvs/qcom_scmi_client",
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
        name = "drivers/thermal/qcom/lmh_stats",
        out = "lmh_stats.ko",
        config = "CONFIG_QTI_THERMAL_LMH_STATS",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/lmh_stats.c",
            "drivers/thermal/qcom/trace_lmh_stats.h",
        ],
    )

    registry.register(
        name = "drivers/thermal/qcom/qcom-spmi-adc-tm5",
        out = "qcom-spmi-adc-tm5.ko",
        config = "CONFIG_QCOM_SPMI_ADC_TM5",
        srcs = [
            # do not sort
            "drivers/thermal/qcom/qcom-spmi-adc-tm5.c",
        ],
        deps = [
            # do not sort
            "drivers/thermal/qcom/qcom-spmi-adc5",
            "drivers/iio/adc/qcom-vadc-common",
        ],
    )
