# SPDX-License-Identifier: GPL-2.0-only


ifeq ($(CONFIG_QCOM_SPSS), m)
        include $(SPU_ROOT)/config/gki_spu.conf
        LINUXINCLUDE += -include $(SPU_ROOT)/config/gki_spuconf.h
endif

LINUXINCLUDE += -I$(srctree)/../../vendor/qcom/opensource/spu-kernel/include/uapi


obj-$(CONFIG_MSM_SPCOM) += drivers/spcom.o
obj-$(CONFIG_MSM_SPSS_UTILS) += drivers/spss_utils.o

BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/spcom.ko
BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/spss_utils.ko
