DSP_KERNEL_ROOT=$(ROOTDIR)vendor/qcom/opensource/dsp-kernel
KBUILD_OPTIONS := DSP_KERNEL_ROOT=$(DSP_KERNEL_ROOT) CONFIG_MSM_ADSPRPC_TRUSTED=m

ifeq ($(TARGET_SUPPORT),genericarmv8)
	KBUILD_OPTIONS += CONFIG_ARCH_PINEAPPLE=y
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions