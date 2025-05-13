# SPDX-License-Identifier: GPL-2.0-only

CONFIG_DEBUGFS_SYNX = false

obj-m += synx/ipclite.o
obj-m += synx-driver.o
synx-driver-objs := synx/synx.o synx/synx_util.o synx/synx_debugfs.o synx/synx_global.o
synx-driver-$(CONFIG_DEBUGFS_SYNX) += synx/synx_debugfs_util.o
