load(":securemsm_kernel.bzl", "define_consolidate_gki_modules")

def define_sa510m():
    define_consolidate_gki_modules(
        target = "sa510m",
        modules = [
            "qce50_dlkm",
            "qcedev-mod_dlkm",
            "qcrypto-msm_dlkm",
            "qrng_dlkm",
            "qseecom_dlkm",
            "smcinvoke_dlkm",
            "tz_log_dlkm",
            "si_core_test"
        ],
        extra_options = [
            "CONFIG_QCOM_SI_CORE",
            "CONFIG_QCOM_SI_CORE_TEST",
            "CONFIG_QCOM_SMCINVOKE",
            "CONFIG_QSEECOM_COMPAT",
        ],
        arch = "arm",
    )
