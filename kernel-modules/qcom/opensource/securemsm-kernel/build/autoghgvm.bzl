load(":securemsm_kernel.bzl", "define_consolidate_gki_modules")

def define_autoghgvm():
    define_consolidate_gki_modules(
        target = "autoghgvm",
        modules = [
            "smcinvoke_dlkm",
            "qrng_dlkm",
            "qseecom_dlkm",
        ],
        extra_options = [
            "CONFIG_QCOM_SMCINVOKE",
            "CONFIG_QSEECOM_COMPAT",
        ],
    )
