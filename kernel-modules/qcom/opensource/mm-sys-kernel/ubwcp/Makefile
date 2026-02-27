# SPDX-License-Identifier: GPL-2.0-only

#ifeq ($(KP_MODULE_ROOT),)
#KP_MODULE_ROOT=$(KERNEL_SRC)/$(M)
#endif

#KBUILD_OPTIONS+=KBUILD_DTC_INCLUDE=$(KP_MODULE_ROOT)

# Kbuild assumes devicetree source lives in arch/arm64/boot/dts, but perhaps it
# lives in some other directory in your project. Specify it with KBUILD_EXTMOD_DTS
# KBUILD_OPTIONS+=KBUILD_EXTMOD_DTS=camera

all: modules # dtbs

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)
