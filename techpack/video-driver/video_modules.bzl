load(":video_driver_build.bzl", "video_module_entry")

video_driver_modules = video_module_entry([":video_driver_headers"])
module_entry = video_driver_modules.register

module_entry(
    name = "msm_video",
    srcs = [
        "driver/vidc/src/msm_vidc_v4l2.c",
        "driver/vidc/src/msm_vidc_vb2.c",
        "driver/vidc/src/msm_vidc.c",
        "driver/vidc/src/msm_vdec.c",
        "driver/vidc/src/msm_venc.c",
        "driver/vidc/src/msm_vidc_driver.c",
        "driver/vidc/src/msm_vidc_state.c",
        "driver/vidc/src/msm_vidc_control.c",
        "driver/vidc/src/msm_vidc_control_ext.c",
        "driver/vidc/src/msm_vidc_buffer.c",
        "driver/vidc/src/msm_vidc_power.c",
        "driver/vidc/src/msm_vidc_probe.c",
        "driver/vidc/src/resources.c",
        "driver/vidc/src/resources_ext.c",
        "driver/vidc/src/firmware.c",
        "driver/vidc/src/msm_vidc_debug.c",
        "driver/vidc/src/msm_vidc_memory.c",
        "driver/vidc/src/msm_vidc_memory_ext.c",
        "driver/vidc/src/msm_vidc_fence.c",
        "driver/vidc/src/venus_hfi.c",
        "driver/vidc/src/venus_hfi_queue.c",
        "driver/vidc/src/hfi_packet.c",
        "driver/vidc/src/venus_hfi_response.c",
        "driver/platform/common/src/msm_vidc_platform.c",
        "driver/platform/common/src/msm_vidc_platform_ext.c",
        "driver/variant/common/src/msm_vidc_variant.c",
        "driver/vidc/src/msm_vidc_synx.c",
        ],
    config_srcs = {
        "CONFIG_MSM_VIDC_PINEAPPLE" : [
            "driver/variant/iris33/src/msm_vidc_buffer_iris33.c",
            "driver/variant/iris33/src/msm_vidc_bus_iris33.c",
            "driver/variant/iris33/src/msm_vidc_clock_iris33.c",
            "driver/variant/iris33/src/msm_vidc_power_iris33.c",
            "driver/variant/iris33/src/msm_vidc_iris33.c",
            "driver/platform/pineapple/src/msm_vidc_pineapple.c",
            "driver/platform/cliffs/src/msm_vidc_cliffs.c",
        ],
    },
    deps = [
            "//vendor/qcom/opensource/mm-drivers:mm_drivers_headers",
            "//vendor/qcom/opensource/synx-kernel:synx_headers",
        ],
)

module_entry(
    name = "video",
    srcs = [
        "driver/vidc/src/msm_vidc_v4l2.c",
        "driver/vidc/src/msm_vidc_vb2.c",
        "driver/vidc/src/msm_vidc.c",
        "driver/vidc/src/msm_vdec.c",
        "driver/vidc/src/msm_venc.c",
        "driver/vidc/src/msm_vidc_driver.c",
        "driver/vidc/src/msm_vidc_state.c",
        "driver/vidc/src/msm_vidc_control.c",
        "driver/vidc/src/msm_vidc_buffer.c",
        "driver/vidc/src/msm_vidc_power.c",
        "driver/vidc/src/msm_vidc_probe.c",
        "driver/vidc/src/resources.c",
        "driver/vidc/src/firmware.c",
        "driver/vidc/src/msm_vidc_debug.c",
        "driver/vidc/src/msm_vidc_memory.c",
        "driver/vidc/src/msm_vidc_fence.c",
        "driver/vidc/src/venus_hfi.c",
        "driver/vidc/src/venus_hfi_queue.c",
        "driver/vidc/src/hfi_packet.c",
        "driver/vidc/src/venus_hfi_response.c",
        "driver/platform/common/src/msm_vidc_platform.c",
        "driver/variant/common/src/msm_vidc_variant.c",
        ],
    config_srcs = {
        "CONFIG_MSM_VIDC_PINEAPPLE" : [
            "driver/platform/pineapple/src/pineapple.c",
            "driver/platform/cliffs/src/cliffs.c",
            "driver/variant/iris33/src/msm_vidc_buffer_iris33.c",
            "driver/variant/iris33/src/msm_vidc_power_iris33.c",
            "driver/variant/iris33/src/msm_vidc_bus_iris33.c",
            "driver/variant/iris33/src/msm_vidc_clock_iris33.c",
            "driver/variant/iris33/src/msm_vidc_iris33.c",
        ],
    }
)
