KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

M=$(PWD)
BT_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_OPTIONS+=  BT_ROOT=$(BT_ROOT)
KBUILD_EXTRA_SYMBOLS=$(call intermediates-dir-for,DLKM,wlan-platform-module-symvers)/Module.symvers
KBUILD_EXTRA_SYMBOLS=$(OUT_DIR)/vendor/qcom/wlan/platform/Module.symvers
ccflags-y += -I$(BT_ROOT)/../wlan/platform/inc

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
