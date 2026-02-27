load(":bt_kernel.bzl", "define_bt_modules")

def define_pineapple():
    define_bt_modules(
        target = "pineapple",
        modules = [
            "btpower",
            "bt_fm_slim",
            "radio-i2c-rtc6226-qca",
            # "btfm_slim_codec",
        ],
        config_options = [
            "CONFIG_MSM_BT_POWER",
            "CONFIG_BTFM_SLIM",
            "CONFIG_I2C_RTC6226_QCA",
            # "CONFIG_SLIM_BTFM_CODEC",
            "CONFIG_BT_HW_SECURE_DISABLE",
        ]
    )

def define_blair():
    define_bt_modules(
        target = "blair",
        modules = [
            "btpower",
            "bt_fm_slim",
            "radio-i2c-rtc6226-qca",
        ],
        config_options = [
            "CONFIG_MSM_BT_POWER",
            "CONFIG_BTFM_SLIM",
            "CONFIG_I2C_RTC6226_QCA",
            "CONFIG_BT_HW_SECURE_DISABLE",
        ]
    )
