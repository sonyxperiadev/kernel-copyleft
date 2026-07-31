def register_modules(registry):
    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs",
        out = "phy-qcom-ufs.ko",
        config = "CONFIG_PHY_QCOM_UFS",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs.c",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-canoe",
        out = "phy-qcom-ufs-qmp-v4-canoe.ko",
        config = "CONFIG_PHY_QCOM_UFS_V4_CANOE",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-canoe.c",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-canoe.h",
        ],
        deps = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-yupik",
        out = "phy-qcom-ufs-qmp-v4-yupik.ko",
        config = "CONFIG_PHY_QCOM_UFS_V4_YUPIK",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-yupik.c",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-yupik.h",
        ],
        deps = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-snps-eusb2",
        out = "phy-qcom-snps-eusb2.ko",
        config = "CONFIG_PHY_QCOM_SNPS_EUSB2",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-snps-eusb2.c",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-khaje",
        out = "phy-qcom-ufs-qmp-v4-khaje.ko",
        config = "CONFIG_PHY_QCOM_UFS_V4_KHAJE",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-khaje.c",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-khaje.h",
        ],
        deps = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-sun",
        out = "phy-qcom-ufs-qmp-v4-sun.ko",
        config = "CONFIG_PHY_QCOM_UFS_V4_SUN",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-sun.c",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-sun.h",
        ],
        deps = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs-qrbtc-sdm845",
        out = "phy-qcom-ufs-qrbtc-sdm845.ko",
        config = "CONFIG_PHY_QCOM_UFS_QRBTC_SDM845",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs-qrbtc-sdm845.c",
            "drivers/phy/qualcomm/phy-qcom-ufs-qrbtc-sdm845.h",
        ],
        deps = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-eusb2-repeater",
        out = "phy-qcom-eusb2-repeater.ko",
        config = "CONFIG_PHY_QCOM_EUSB2_REPEATER",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-eusb2-repeater.c",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-m31-eusb2",
        out = "phy-qcom-m31-eusb2.ko",
        config = "CONFIG_PHY_QCOM_M31_EUSB2",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-m31-eusb2.c",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-qmp-combo",
        out = "phy-qcom-qmp-combo.ko",
        config = "CONFIG_PHY_QCOM_QMP_COMBO",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-qmp-combo.c",
            "drivers/phy/qualcomm/phy-qcom-qmp-common.h",
            "drivers/phy/qualcomm/phy-qcom-qmp.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-misc-v3.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-usb-v4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-usb-v5.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-usb-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-aon-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-dp-com-v3.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-dp-phy.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-dp-phy-v3.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-dp-phy-v4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-dp-phy-v5.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-dp-phy-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-com.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-com-v3.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v3.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-com-v4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v4_20.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-com-v5.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v5.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v5_20.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v5_5nm.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-com-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v6_20.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v6_n4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-ln-shrd-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-com-v7.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-txrx-v7.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-qserdes-pll.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v2.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v3.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v4_20.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v5.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v5_20.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v6.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v6-n4.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v6_20.h",
            "drivers/phy/qualcomm/phy-qcom-qmp-pcs-v7.h",
        ],
    )

    registry.register(
        name = "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-waipio",
        out = "phy-qcom-ufs-qmp-v4-waipio.ko",
        config = "CONFIG_PHY_QCOM_UFS_V4_WAIPIO",
        srcs = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs-i.h",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-waipio.c",
            "drivers/phy/qualcomm/phy-qcom-ufs-qmp-v4-waipio.h",
        ],
        deps = [
            # do not sort
            "drivers/phy/qualcomm/phy-qcom-ufs",
        ],
    )
