def register_modules(registry):
    registry.register(
        name = "drivers/misc/hardware_info",
        out = "hardware_info.ko",
        config = "CONFIG_HARDWAREINFO",
        srcs = [
            # do not sort
            "drivers/misc/hardware_info/hardware_info.c",
        ],
    )
