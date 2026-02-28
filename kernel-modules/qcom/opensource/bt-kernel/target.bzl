load(":bt_kernel.bzl", "define_bt_modules")

def define_pineapple():
    define_bt_modules(
        target = "pineapple",
        modules = [
            "btpower",
            "bt_fm_slim",
            "radio-i2c-rtc6226-qca",
        ],
        config_options = [
            "CONFIG_MSM_BT_POWER",
            "CONFIG_BTFM_SLIM",
            "CONFIG_I2C_RTC6226_QCA",
            "CONFIG_FMD_ENABLE",
            #"CONFIG_BT_HW_SECURE_DISABLE",
        ]
    )

def define_sun():
    define_bt_modules(
        target = "sun",
        modules = [
            "btpower",
            "radio-i2c-rtc6226-qca",
            "btfm_slim_codec",
            "btfmcodec",
            "bt_fm_swr",
         ],
         config_options = [
             "CONFIG_MSM_BT_POWER",
             "CONFIG_I2C_RTC6226_QCA",
             "CONFIG_SLIM_BTFM_CODEC",
             "CONFIG_BTFM_CODEC",
           #  "CONFIG_BT_HW_SECURE_DISABLE",
             "CONFIG_BTFM_SWR",
	     "CONFIG_FMD_ENABLE",
        ]
    )

def define_parrot():
    define_bt_modules(
        target = "parrot66",
        modules = [
            "btpower",
            "bt_fm_slim",
            "radio-i2c-rtc6226-qca",
        ],
        config_options = [
            "CONFIG_MSM_BT_POWER",
            "CONFIG_BTFM_SLIM",
            "CONFIG_I2C_RTC6226_QCA",
            #"CONFIG_BT_HW_SECURE_DISABLE",
        ]
    )
