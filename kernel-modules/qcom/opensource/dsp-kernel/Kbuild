# ported from Android.mk
$(info within KBUILD file KBUILD_EXTRA_SYMBOLS = $(KBUILD_EXTRA_SYMBOLS))

ifeq ($(CONFIG_ARCH_WAIPIO), y)
$(info within KBUILD file CONFIG_ARCH_WAIPIO = $(CONFIG_ARCH_WAIPIO))
KBUILD_CPPFLAGS += -DCONFIG_DSP_WAIPIO=1
ccflags-y += -DCONFIG_DSP_WAIPIO=1
endif

ifeq ($(CONFIG_ARCH_KALAMA), y)
$(info within KBUILD file CONFIG_ARCH_KALAMA = $(CONFIG_ARCH_KALAMA))
KBUILD_CPPFLAGS += -DCONFIG_DSP_KALAMA=1
ccflags-y += -DCONFIG_DSP_KALAMA=1
endif

ifeq ($(CONFIG_ARCH_PINEAPPLE), y)
$(info within KBUILD file CONFIG_ARCH_PINEAPPLE = $(CONFIG_ARCH_PINEAPPLE))
KBUILD_CPPFLAGS += -DCONFIG_DSP_PINEAPPLE=1
ccflags-y += -DCONFIG_DSP_PINEAPPLE=1
endif

LINUXINCLUDE += -I$(DSP_ROOT)/include/linux
LINUXINCLUDE += -I$(DSP_ROOT)/include/uapi

frpc-adsprpc-y := dsp/adsprpc.o	\
                  dsp/adsprpc_rpmsg.o \

frpc-adsprpc-$(CONFIG_COMPAT) += dsp/adsprpc_compat.o \

frpc_trusted-adsprpc-y := dsp/adsprpc.o	\
                          dsp/adsprpc_compat.o \
                          dsp/adsprpc_socket.o \

cdsp-loader-y := dsp/cdsp-loader.o

obj-m := frpc-adsprpc.o cdsp-loader.o

BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/frpc-adsprpc.ko
#BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/cdsp-loader.ko
