# SPDX-License-Identifier: GPL-2.0-only

ifeq ($(CONFIG_CNSS_OUT_OF_TREE),y)
ccflags-y += -I$(WLAN_PLATFORM_ROOT)/inc
ccflags-y += -I$(WLAN_PLATFORM_ROOT)/cnss_utils
endif

obj-$(CONFIG_WCNSS_MEM_PRE_ALLOC) += cnss_prealloc.o
