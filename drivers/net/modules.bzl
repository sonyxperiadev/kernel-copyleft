load(":drivers/net/ethernet/stmicro/stmmac/modules.bzl", register_stmmac_eth = "register_modules")
load(":drivers/net/pcs/modules.bzl", register_pcs = "register_modules")
load(":drivers/net/phy/aquantia/modules.bzl", register_phy_aquantia = "register_modules")

def register_modules(registry):
    register_pcs(registry)
    register_stmmac_eth(registry)
    register_phy_aquantia(registry)
    registry.register(
        name = "net/core/failover",
        out = "failover.ko",
        config = "CONFIG_FAILOVER",
        srcs = [
            # do not sort
            "net/core/failover.c",
        ],
    )

    registry.register(
        name = "drivers/net/net_failover",
        out = "net_failover.ko",
        config = "CONFIG_NET_FAILOVER",
        srcs = [
            # do not sort
            "drivers/net/net_failover.c",
        ],
        deps = [
            # do not sort
            "net/core/failover",
        ],
    )

    registry.register(
        name = "drivers/net/virtio_net",
        out = "virtio_net.ko",
        config = "CONFIG_VIRTIO_NET",
        srcs = [
            # do not sort
            "drivers/net/virtio_net_fixed.c",
        ],
        deps = [
            # do not sort
            "drivers/net/net_failover",
        ],
    )
