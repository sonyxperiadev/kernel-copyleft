load(":securemsm_kernel.bzl", "define_consolidate_gki_modules")

def define_parrot():
    define_consolidate_gki_modules(
        target = "parrot",
        modules = [
            "smcinvoke_dlkm",
            "tz_log_dlkm",
            "hdcp_qseecom_dlkm",
            "qce50_dlkm",
            "qcedev-mod_dlkm",
            "qrng_dlkm",
            "qcrypto-msm_dlkm",
            "qseecom_dlkm",
            "smmu_proxy_dlkm"
        ],
        extra_options = [
            "CONFIG_QCOM_SMCINVOKE",
            "CONFIG_QSEECOM",
            "CONFIG_TZ_LAST_LOGS",
        ],
    )
