load("//build/kernel/kleaf:kernel.bzl", "ddk_module")
load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")

msm_kgsl_includes = [
    "include/linux/msm_kgsl.h",
    "include/uapi/linux/msm_kgsl.h",
]

def kgsl_get_srcs():
    srcs = [
        "adreno.c",
        "adreno_a3xx.c",
        "adreno_a3xx_perfcounter.c",
        "adreno_a3xx_ringbuffer.c",
        "adreno_a3xx_snapshot.c",
        "adreno_a5xx.c",
        "adreno_a5xx_perfcounter.c",
        "adreno_a5xx_preempt.c",
        "adreno_a5xx_ringbuffer.c",
        "adreno_a5xx_snapshot.c",
        "adreno_a6xx.c",
        "adreno_a6xx_gmu.c",
        "adreno_a6xx_gmu_snapshot.c",
        "adreno_a6xx_hfi.c",
        "adreno_a6xx_hwsched.c",
        "adreno_a6xx_hwsched_hfi.c",
        "adreno_a6xx_perfcounter.c",
        "adreno_a6xx_preempt.c",
        "adreno_a6xx_rgmu.c",
        "adreno_a6xx_ringbuffer.c",
        "adreno_a6xx_rpmh.c",
        "adreno_a6xx_snapshot.c",
        "adreno_cp_parser.c",
        "adreno_dispatch.c",
        "adreno_drawctxt.c",
        "adreno_gen7.c",
        "adreno_gen7_gmu.c",
        "adreno_gen7_gmu_snapshot.c",
        "adreno_gen7_hfi.c",
        "adreno_gen7_hwsched.c",
        "adreno_gen7_hwsched_hfi.c",
        "adreno_gen7_perfcounter.c",
        "adreno_gen7_preempt.c",
        "adreno_gen7_ringbuffer.c",
        "adreno_gen7_rpmh.c",
        "adreno_gen7_snapshot.c",
        "adreno_hwsched.c",
        "adreno_ioctl.c",
        "adreno_perfcounter.c",
        "adreno_ringbuffer.c",
        "adreno_snapshot.c",
        "adreno_sysfs.c",
        "adreno_trace.c",
        "governor_msm_adreno_tz.c",
        "governor_gpubw_mon.c",
        "kgsl.c",
        "kgsl_bus.c",
        "kgsl_drawobj.c",
        "kgsl_events.c",
        "kgsl_eventlog.c",
        "kgsl_gmu_core.c",
        "kgsl_ioctl.c",
        "kgsl_mmu.c",
        "kgsl_pwrctrl.c",
        "kgsl_pwrscale.c",
        "kgsl_regmap.c",
        "kgsl_sharedmem.c",
        "kgsl_snapshot.c",
        "kgsl_timeline.c",
        "kgsl_trace.c",
        "kgsl_util.c",
        "kgsl_vbo.c",
    ]

    srcs = srcs + native.glob(["*.h"]) + msm_kgsl_includes

    return srcs

def external_deps(target, variant):
    tv = "{}_{}".format(target, variant)
    deplist = []
    defconfigs = []

    # Add msm_hw_fence in the dependency and defconfig lists for targets that use it
    if target in [ "pineapple" ]:
        deplist = deplist + [
            "//vendor/qcom/opensource/mm-drivers/hw_fence:{}_msm_hw_fence".format(tv)
            ]
        defconfigs = defconfigs + [
            "//vendor/qcom/opensource/mm-drivers/hw_fence:defconfig"
            ]

    native.genrule(
       name = "{}_defconfig".format(tv),
       srcs = defconfigs + [ "config/{}_gpuconf".format(tv) ],
       outs = [ "{}_defconfig.generated".format(tv) ],
       cmd = "cat $(SRCS) > $@"
    )

    return deplist

def define_target_variant_module(target, variant):
    tv = "{}_{}".format(target, variant)
    rule_name = "{}_msm_kgsl".format(tv)
    kernel_build = "//msm-kernel:{}".format(tv)

    ext_deps = external_deps(target, variant)

    ddk_module(
        name = rule_name,
        out = "msm_kgsl.ko",
        srcs = kgsl_get_srcs(),
        defconfig = "{}_defconfig".format(tv),
        kconfig = "Kconfig",
        conditional_srcs = {
            "CONFIG_ARM_SMMU": { True: [ "kgsl_iommu.c" ] },
            "CONFIG_COMPAT": { True: [ "kgsl_compat.c", "adreno_compat.c" ] },
            "CONFIG_DEBUG_FS": { True: [ "kgsl_debugfs.c", "adreno_debugfs.c", "adreno_profile.c" ] },
            "CONFIG_QCOM_KGSL_CORESIGHT": { True: [
                "adreno_coresight.c",
                "adreno_a3xx_coresight.c",
                "adreno_a5xx_coresight.c",
                "adreno_a6xx_coresight.c",
                "adreno_gen7_coresight.c"] },
            "CONFIG_QCOM_KGSL_PROCESS_RECLAIM": { True: [ "kgsl_reclaim.c" ] },
            "CONFIG_QCOM_KGSL_USE_SHMEM": { False: [ "kgsl_pool.c" ] },
            "CONFIG_SYNC_FILE": { True: [ "kgsl_sync.c" ] },
        },
        deps = [ "//msm-kernel:all_headers" ] + ext_deps,
        includes = ["include", "."],
        kernel_build = kernel_build,
        visibility = ["//visibility:private"]
    )

    copy_to_dist_dir(
        name = "{}_dist".format(rule_name),
        data = [rule_name],
        dist_dir = "out/graphics-kernel",
        flat = True,
        wipe_dist_dir = False,
        allow_duplicate_filenames = False,
        mode_overrides = {"**/*": "644"},
        log = "info",
    )

def define_target_module(target):
    define_target_variant_module(target, "gki")
    define_target_variant_module(target, "consolidate")
