SMCINVOKE_PATH = "smcinvoke"
QSEECOM_PATH = "qseecom"
TZLOG_PATH = "tz_log"
HDCP_PATH = "hdcp"
QCEDEV_PATH = "crypto-qti"
QRNG_PATH = "qrng"
SMMU_PROXY_PATH = "smmu-proxy"

# This dictionary holds all the securemsm-kernel  modules included by calling register_securemsm_module
securemsm_modules = {}
securemsm_modules_by_config = {}

# Registers securemsm module to kernel build system.
# name: The name of the module. The name of the file generated for this module will be {name}.ko.
# path: The path that will be prepended to all sources listed for this module.
# config_option: If this module is enabled, the config optiont that will get enabled if so. Not all modules have this, and this is an optional parameter.
# config_srcs: A dictionary of sources to be added to the module depending on if a configuration option is enabled or not. The keys to the dictionary are
# the name of the config option, and the value depends If it is a list, it will just be the list of sources to be added to the module if the config option
# is enabled. If the value is another dictionary, then you can specify sources to be added if the config option is DISABLED by having a list under the
# default_srcs: A list of sources to be added to the module regardless of configuration options.
# deps: A list of kernel_module or ddk_module rules that this module depends on.

def register_securemsm_module(name, path = None, config_option = None, default_srcs = [], config_srcs = {}, deps = [], srcs = [], copts = [], hdrs = []):
    processed_config_srcs = {}
    for config_src_name in config_srcs:
        config_src = config_srcs[config_src_name]

        if type(config_src) == "list":
            processed_config_srcs[config_src_name] = {True: config_src}
        else:
            processed_config_srcs[config_src_name] = config_src

    module = {
        "name": name,
        "path": path,
        "default_srcs": default_srcs,
        "config_srcs": processed_config_srcs,
        "config_option": config_option,
        "deps": deps,
        "copts": copts,
        "srcs": srcs,
        "hdrs": hdrs,
    }

    securemsm_modules[name] = module

    if config_option:
        securemsm_modules_by_config[config_option] = name

# ------------------------------------ SECUREMSM MODULE DEFINITIONS ---------------------------------
register_securemsm_module(
    name = "qseecom_dlkm",
    path = QSEECOM_PATH,
    default_srcs = [
        "qseecom.c",
        "ice.h",
    ],
    deps = [":qseecom_kernel_headers"],
    #srcs = ["config/sec-kernel_defconfig_qseecom.h"],
    #copts = ["-include", "config/sec-kernel_defconfig_qseecom.h"],
)


register_securemsm_module(
    name = "smcinvoke_dlkm",
    path = SMCINVOKE_PATH,
    default_srcs = [
        "smcinvoke.c",
        "smcinvoke_kernel.c",
        "trace_smcinvoke.h",
        "IQSEEComCompat.h",
        "IQSEEComCompatAppLoader.h",
    ],
    deps = [":smcinvoke_kernel_headers", ":qseecom_kernel_headers", "%b_qseecom_dlkm"],
    hdrs = [":smcinvoke_kernel_headers"],
)

register_securemsm_module(
    name = "tz_log_dlkm",
    path = TZLOG_PATH,
    deps = [":qseecom_kernel_headers"],
    default_srcs = [
        "last_logs.h",
        "tz_log.c",
    ],
)

register_securemsm_module(
    name = "hdcp_qseecom_dlkm",
    path = HDCP_PATH,
    default_srcs = [
        "hdcp_qseecom.c",
        "hdcp_qseecom.h",
        "hdcp_main.c",
        "smcinvoke_object.h",
        "hdcp_main.h",
        "hdcp_smcinvoke.c",
        "hdcp_smcinvoke.h",
        "CAppClient.h",
        "CAppLoader.h",
        "IAppClient.h",
        "IAppController.h",
        "IAppLoader.h",
        "IClientEnv.h",
        "IOpener.h",
        "hdcp1.h",
        "hdcp1_ops.h",
        "hdcp2p2.h",
    ],
    deps = [":hdcp_qseecom_dlkm", "%b_smcinvoke_dlkm", "%b_qseecom_dlkm"],
    srcs = ["config/sec-kernel_defconfig.h"],
    copts = [
        "-include",
        "config/sec-kernel_defconfig.h",
    ],
)

register_securemsm_module(
    name = "qce50_dlkm",
    path = QCEDEV_PATH,
    default_srcs = ["qce50.c"],
    deps = [":qcedev_local_headers"],
)

register_securemsm_module(
    name = "qcedev-mod_dlkm",
    path = QCEDEV_PATH,
    default_srcs = [
                "qcedev.c",
                "qcedev_smmu.c"],
    deps = [":qcedev_local_headers",
            "%b_qce50_dlkm"],
)

register_securemsm_module(
    name = "qrng_dlkm",
    path = QRNG_PATH,
    default_srcs = ["msm_rng.c"],
    deps = [":qcedev_local_headers"],
)

register_securemsm_module(
    name = "qcrypto-msm_dlkm",
    path = QCEDEV_PATH,
    default_srcs = ["qcrypto.c"],
    deps = [":qcedev_local_headers",
            "%b_qce50_dlkm"],
)

register_securemsm_module(
    name = "smmu_proxy_dlkm",
    path = SMMU_PROXY_PATH,
    srcs = ["qti-smmu-proxy-pvm.c", "qti-smmu-proxy-common.c"],
    deps = ["%b_smcinvoke_dlkm", ":smmu_proxy_headers"],
)
