def register_modules(registry):
    registry.register(
        name = "drivers/input/touchscreen/lxs_ts/lxs_touchscreen",
        out = "lxs_touchscreen.ko",
        config = "CONFIG_TOUCHSCREEN_LXS",
        srcs = [
            # do not sort
            "drivers/input/touchscreen/lxs_ts/lxs_ts.h",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_hal_abt.h",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_hal_prd.h",
            "drivers/input/touchscreen/lxs_ts/lxs_ts.c",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_fn.c",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_hal.c",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_hal_fw.c",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_hal_prd.c",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_hal_abt.c",
            "drivers/input/touchscreen/lxs_ts/lxs_ts_sysfs.c",
            "drivers/input/touchscreen/lxs_ts/touch_lx82907a.c",
        ],
        deps = [
            # do not sort
            "drivers/soc/qcom/panel_event_notifier",
        ],
    )

