load(":securemsm_kernel.bzl", "define_consolidate_gki_modules")

def define_sun():
    define_consolidate_gki_modules(
        target = "sun",
        modules = [
            "smcinvoke_dlkm",
            "tz_log_dlkm",
            "qseecom_dlkm",
            "hdcp_qseecom_dlkm",
            "qce50_dlkm",
            "qcedev-mod_dlkm",
            "qrng_dlkm",
            "qcrypto-msm_dlkm",
            "smmu_proxy_dlkm"
         ],
         extra_options = [
             "CONFIG_QCOM_SMCINVOKE",
             "CONFIG_QSEECOM_COMPAT",
         ],
     )
