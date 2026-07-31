def register_modules(registry):
    registry.register(
        name = "drivers/leds/rgb/leds-qcom-lpg",
        out = "leds-qcom-lpg.ko",
        config = "CONFIG_LEDS_QCOM_LPG",
        srcs = [
            # do not sort
            "drivers/leds/rgb/leds-qcom-lpg.c",
        ],
    )
