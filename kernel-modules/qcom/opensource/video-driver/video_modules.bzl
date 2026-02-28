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
        "driver/vidc/src/msm_vidc_dt.c",
        "driver/vidc/src/msm_venc.c",
        "driver/vidc/src/msm_vidc_driver.c",
        "driver/vidc/src/msm_vidc_control.c",
        "driver/vidc/src/msm_vidc_buffer.c",
        "driver/vidc/src/msm_vidc_power.c",
        "driver/vidc/src/msm_vidc_probe.c",
        "driver/vidc/src/msm_vidc_debug.c",
        "driver/vidc/src/msm_vidc_memory.c",
        "driver/vidc/src/venus_hfi.c",
        "driver/vidc/src/hfi_packet.c",
        "driver/vidc/src/venus_hfi_response.c",
        "driver/vidc/src/msm_vidc_fence.c",
        "driver/platform/common/src/msm_vidc_platform.c",
        ],
    config_srcs = {
        "CONFIG_MSM_VIDC_MONACO" : [
            "driver/platform/monaco/src/msm_vidc_monaco.c",
        ],
        "CONFIG_MSM_VIDC_BLAIR"  : [
            "driver/platform/blair/src/msm_vidc_blair.c",
        ],
        "CONFIG_MSM_VIDC_VIENNA"  : [
            "driver/platform/vienna/src/msm_vidc_vienna.c",
        ],
        "CONFIG_MSM_VIDC_AR50LT" : [
            "driver/variant/ar50lt/src/msm_vidc_power_ar50lt.c",
            "driver/variant/ar50lt/src/msm_vidc_buffer_ar50lt.c",
            "driver/variant/ar50lt/src/msm_vidc_ar50lt.c",
        ],
    },
)
