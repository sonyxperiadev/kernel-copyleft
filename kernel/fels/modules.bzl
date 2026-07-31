def register_modules(registry):
    registry.register(
        name = "kernel/fels/fels",
        out = "fels.ko",
        config = "CONFIG_FELS",
        srcs = [
            # do not sort
            "kernel/fels/fels.c",
        ],
        hdrs = ["kernel/fels/include/fels.h"],
        includes = ["kernel/fels/include"],
    )
