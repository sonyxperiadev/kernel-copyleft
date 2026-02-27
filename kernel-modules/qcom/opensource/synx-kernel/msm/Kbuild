LINUXINCLUDE    += -I$(SYNX_ROOT)/include \
                   -I$(SYNX_ROOT)/include/uapi \
				   -I$(SYNX_ROOT)/include/uapi/synx/media

ccflags-y += -I$(SYNX_ROOT)/msm/synx/

# add flag to compile mmrm actual implementatio instead of stub version.
# to follow up with mmrm team if techpack users need to define this for long term?
#KBUILD_CPPFLAGS += -DCONFIG_MSM_MMRM

# ported from Android.mk
$(info within KBUILD file KBUILD_EXTRA_SYMBOLS = $(KBUILD_EXTRA_SYMBOLS))

ifeq ($(CONFIG_ARCH_WAIPIO), y)
$(info within KBUILD file CONFIG_ARCH_WAIPIO = $(CONFIG_ARCH_WAIPIO))
# include $(SYNX_ROOT)/config/waipio.mk
KBUILD_CPPFLAGS += -DCONFIG_SYNX_WAIPIO=1
ccflags-y += -DCONFIG_SYNX_WAIPIO=1
endif

ifeq ($(CONFIG_ARCH_KALAMA), y)
$(info within KBUILD file CONFIG_ARCH_KALAMA = $(CONFIG_ARCH_KALAMA))
# include $(SYNX_ROOT)/config/waipio.mk
KBUILD_CPPFLAGS += -DCONFIG_SYNX_KALAMA=1
ccflags-y += -DCONFIG_SYNX_KALAMA=1
endif

ifeq ($(CONFIG_ARCH_PINEAPPLE), y)
$(info within KBUILD file CONFIG_ARCH_PINEAPPLE = $(CONFIG_ARCH_PINEAPPLE))
# include $(SYNX_ROOT)/config/pineapple.mk
KBUILD_CPPFLAGS += -DCONFIG_SYNX_PINEAPPLE=1
ccflags-y += -DCONFIG_SYNX_PINEAPPLE=1
endif

obj-m += synx-driver.o
obj-m += synx/ipclite.o
obj-m += synx/test/ipclite_test.o
synx-driver-objs := synx/synx.o synx/synx_global.o synx/synx_util.o synx/synx_debugfs.o
