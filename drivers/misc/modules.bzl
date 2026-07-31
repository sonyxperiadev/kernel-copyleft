load(":drivers/misc/lkdtm/modules.bzl", register_lkdtm = "register_modules")
load(":drivers/misc/hardware_info/modules.bzl", register_hardinfo = "register_modules")

def register_modules(registry):
    register_lkdtm(registry)
    register_hardinfo(registry)

    registry.register(
        name = "drivers/misc/qseecom_proxy",
        out = "qseecom_proxy.ko",
        config = "CONFIG_QSEECOM_PROXY",
        srcs = [
            # do not sort
            "drivers/misc/qseecom_proxy.c",
        ],
    )
