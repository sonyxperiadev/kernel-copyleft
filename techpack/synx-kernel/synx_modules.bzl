load(":synx_module_build.bzl", "create_module_registry")

SYNX_KERNEL_ROOT = "synx-kernel"

synx_modules = create_module_registry([":synx_headers"])
register_synx_module = synx_modules.register

register_synx_module(
    name = "synx-driver",
    path = "msm",
    srcs = [
        "synx/synx.c",
        "synx/synx_global.c",
        "synx/synx_util.c",
        "synx/synx_debugfs.c",
        "synx/synx_debugfs_util.c",
    ],
)

register_synx_module(
    name = "ipclite",
    path = "msm",
    srcs = [
        "synx/ipclite.c",
    ],
)
register_synx_module(
    name = "ipclite_test",
    path = "msm",
    srcs = [
        "synx/test/ipclite_test.c",
    ],
)
