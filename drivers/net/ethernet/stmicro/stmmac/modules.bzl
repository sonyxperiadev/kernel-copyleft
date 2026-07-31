def register_modules(registry):
    registry.register(
        name = "drivers/net/ethernet/stmicro/stmmac",
        out = "stmmac.ko",
        config = "CONFIG_STMMAC_ETH",
        srcs = [
            # do not sort
            "drivers/net/ethernet/stmicro/stmmac/stmmac_main.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_ethtool.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_mdio.c",
            "drivers/net/ethernet/stmicro/stmmac/ring_mode.c",
            "drivers/net/ethernet/stmicro/stmmac/chain_mode.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac_lib.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac1000_core.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac1000_dma.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac100_core.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac100_dma.c",
            "drivers/net/ethernet/stmicro/stmmac/enh_desc.c",
            "drivers/net/ethernet/stmicro/stmmac/norm_desc.c",
            "drivers/net/ethernet/stmicro/stmmac/mmc_core.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_hwtstamp.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_ptp.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4_descs.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4_dma.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4_lib.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4_core.c",
            "drivers/net/ethernet/stmicro/stmmac/dwmac5.c",
            "drivers/net/ethernet/stmicro/stmmac/hwif.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_tc.c",
            "drivers/net/ethernet/stmicro/stmmac/dwxgmac2_core.c",
            "drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c",
            "drivers/net/ethernet/stmicro/stmmac/dwxgmac2_descs.c",
            "drivers/net/ethernet/stmicro/stmmac/dw25gmac.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_xdp.c",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_est.c",
            "drivers/net/ethernet/stmicro/stmmac/common.h",
            "drivers/net/ethernet/stmicro/stmmac/descs.h",
            "drivers/net/ethernet/stmicro/stmmac/descs_com.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac100.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac1000.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4_descs.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac4_dma.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac5.h",
            "drivers/net/ethernet/stmicro/stmmac/dwmac_dma.h",
            "drivers/net/ethernet/stmicro/stmmac/dwxgmac2.h",
            "drivers/net/ethernet/stmicro/stmmac/dwxlgmac2.h",
            "drivers/net/ethernet/stmicro/stmmac/dw25gmac.h",
            "drivers/net/ethernet/stmicro/stmmac/hwif.h",
            "drivers/net/ethernet/stmicro/stmmac/mmc.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_est.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_pcs.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_platform.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_ptp.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_xdp.h",
        ],
        deps = [
            # do not sort
            "drivers/net/pcs/pcs-xpcs",
        ],
    )

    registry.register(
        name = "drivers/net/ethernet/stmicro/stmmac/stmmac_platform",
        out = "stmmac_platform.ko",
        config = "CONFIG_STMMAC_PLATFORM",
        srcs = [
            # do not sort
            "drivers/net/ethernet/stmicro/stmmac/stmmac_platform.c",
            "drivers/net/ethernet/stmicro/stmmac/descs.h",
            "drivers/net/ethernet/stmicro/stmmac/hwif.h",
            "drivers/net/ethernet/stmicro/stmmac/mmc.h",
            "drivers/net/ethernet/stmicro/stmmac/common.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_platform.h",
        ],
        deps = [
            # do not sort
            "drivers/net/pcs/pcs-xpcs",
            "drivers/net/ethernet/stmicro/stmmac",
        ],
    )

    registry.register(
        name = "drivers/net/ethernet/stmicro/stmmac/dwmac-qcom-ethqos",
        out = "dwmac-qcom-ethqos.ko",
        config = "CONFIG_DWMAC_QCOM_ETHQOS",
        srcs = [
            # do not sort
            "drivers/net/ethernet/stmicro/stmmac/dwmac-qcom-ethqos.c",
            "drivers/net/ethernet/stmicro/stmmac/descs.h",
            "drivers/net/ethernet/stmicro/stmmac/hwif.h",
            "drivers/net/ethernet/stmicro/stmmac/mmc.h",
            "drivers/net/ethernet/stmicro/stmmac/common.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac.h",
            "drivers/net/ethernet/stmicro/stmmac/stmmac_platform.h",
        ],
        deps = [
            # do not sort
            "drivers/net/ethernet/stmicro/stmmac/stmmac_platform",
            "drivers/net/ethernet/stmicro/stmmac",
            "drivers/net/pcs/pcs-xpcs-qcom",
        ],
    )
