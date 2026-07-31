# SPDX-License-Identifier: GPL-2.0

"""
Macros to define rules for sources which we want to take directly from ACK.

These files are very lightly modified or built exactly from ACK and aren't
suitable to be system_dlkm modules. We cannot reference them directly in DDK
module sources due to DDK limitations that exist today.
"""

load("@bazel_skylib//rules:copy_file.bzl", "copy_file")

COPY_FILES = [
    "drivers/leds/trigger/ledtrig-pattern.c",
    "drivers/char/virtio_console.c",
    "drivers/dma/qcom/gpi.c",
    "drivers/dma/dmaengine.h",
    "drivers/dma/virt-dma.h",
    "drivers/gpio/gpio-virtio.c",
    "drivers/i2c/busses/i2c-qcom-geni.c",
    "drivers/i2c/busses/i2c-virtio.c",
    "drivers/spi/spi-geni-qcom.c",
    "drivers/virtio/virtio_input.c",
    "net/core/failover.c",
    "lib/crc-itu-t.c",
    "drivers/net/net_failover.c",
    "drivers/net/virtio_net.c",
    "drivers/net/pcs/pcs-xpcs.c",
    "drivers/net/pcs/pcs-xpcs.h",
    "drivers/net/pcs/pcs-xpcs-nxp.c",
    "drivers/net/pcs/pcs-xpcs-plat.c",
    "drivers/net/pcs/pcs-xpcs-wx.c",
    "drivers/net/phy/aquantia/aquantia_leds.c",
    "drivers/net/phy/aquantia/aquantia_firmware.c",
    # cfg80211.ko files start
    "net/wireless/ap.c",
    "net/wireless/certs/sforshee.hex",
    "net/wireless/certs/wens.hex",
    "net/wireless/chan.c",
    "net/wireless/core.c",
    "net/wireless/core.h",
    "net/wireless/debugfs.c",
    "net/wireless/debugfs.h",
    "net/wireless/ethtool.c",
    "net/wireless/ibss.c",
    "net/wireless/Kconfig",
    "net/wireless/lib80211.c",
    "net/wireless/lib80211_crypt_ccmp.c",
    "net/wireless/lib80211_crypt_tkip.c",
    "net/wireless/lib80211_crypt_wep.c",
    "net/wireless/Makefile",
    "net/wireless/mesh.c",
    "net/wireless/mlme.c",
    "net/wireless/nl80211.c",
    "net/wireless/nl80211.h",
    "net/wireless/ocb.c",
    "net/wireless/of.c",
    "net/wireless/pmsr.c",
    "net/wireless/radiotap.c",
    "net/wireless/rdev-ops.h",
    "net/wireless/reg.c",
    "net/wireless/reg.h",
    "net/wireless/scan.c",
    "net/wireless/sme.c",
    "net/wireless/sysfs.c",
    "net/wireless/sysfs.h",
    "net/wireless/tests/chan.c",
    "net/wireless/tests/fragmentation.c",
    "net/wireless/tests/Makefile",
    "net/wireless/tests/module.c",
    "net/wireless/tests/scan.c",
    "net/wireless/tests/util.c",
    "net/wireless/tests/util.h",
    "net/wireless/trace.c",
    "net/wireless/trace.h",
    "net/wireless/util.c",
    "net/wireless/wext-compat.c",
    "net/wireless/wext-compat.h",
    "net/wireless/wext-core.c",
    "net/wireless/wext-priv.c",
    "net/wireless/wext-proc.c",
    "net/wireless/wext-sme.c",
    "net/wireless/wext-spy.c",
    "net/sched/sch_mqprio.c",
    "net/sched/sch_mqprio_lib.c",
    "net/sched/sch_mqprio_lib.h",
    "net/sched/sch_cbs.c",
    "net/sched/sch_etf.c",
    "net/sched/cls_flower.c",
    # cfg80211.ko files end
    "drivers/mfd/qcom-pm8008.c",
    "drivers/regulator/qcom-pm8008-regulator.c",
]

def define_common_upstream_files():
    for file in COPY_FILES:
        copy_file(
            name = "copied-" + file,
            src = "//common:{}".format(file),
            out = file,
        )

    # TODO: Use hermetic_genrule when prebuilt `patch` tool is available.
    native.genrule(
        name = "patched-drivers/regulator/qti_fixed_regulator.c",
        srcs = [
            "//common:drivers/regulator/fixed.c",
            ":drivers/regulator/fixed.diff",
        ],
        outs = ["drivers/regulator/qti_fixed_regulator.c"],
        cmd = "patch --follow-symlinks -o $@ -i $(execpath :drivers/regulator/fixed.diff) $(execpath //common:drivers/regulator/fixed.c)",
    )

    native.genrule(
        name = "patched-drivers/dma/qcom/gpi.c",
        srcs = [
            "//common:drivers/dma/qcom/gpi.c",
            ":drivers/dma/qcom/gpi_fix.diff",
        ],
        outs = ["drivers/dma/qcom/gpi_fixed.c"],
        cmd = "patch --follow-symlinks -o $@ -i $(execpath :drivers/dma/qcom/gpi_fix.diff) $(execpath //common:drivers/dma/qcom/gpi.c)",
    )

    native.genrule(
        name = "patched-drivers/net/virtio_net.c",
        srcs = [
            "//common:drivers/net/virtio_net.c",
            ":drivers/net/virtio_net_fix.diff",
        ],
        outs = ["drivers/net/virtio_net_fixed.c"],
        cmd = "patch --follow-symlinks -o $@ -i $(execpath :drivers/net/virtio_net_fix.diff) $(execpath //common:drivers/net/virtio_net.c)",
    )
