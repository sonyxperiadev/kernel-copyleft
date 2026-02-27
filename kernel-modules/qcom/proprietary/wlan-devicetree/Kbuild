ifeq ($(CONFIG_ARCH_MONACO_AUTO),y)
dtbo-y += monaco_auto-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_WAIPIO),y)
dtbo-y += waipio-cnss.dtbo
dtbo-y += waipio-kiwi-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_KALAMA),y)
dtbo-y += kalama-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_QCS405),y)
dtbo-y += qcs405-icnss.dtbo
endif

ifeq ($(CONFIG_ARCH_KHAJE),y)
dtbo-y += khaje-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_BLAIR),y)
dtbo-y += blair-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_HOLI),y)
dtbo-y += holi-cnss.dtbo
dtbo-y += holi-pm6125-cnss.dtbo
dtbo-y += holi-pm6125-nopmi-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_CROW),y)
dtbo-y += crow-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SA6155),y)
dtbo-y += sa6155p-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SA8155),y)
dtbo-y += sa8155p-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SA8195),y)
dtbo-y += sa8195p-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SDXPINN),y)
dtbo-y += sdxpinn-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_PINEAPPLE),y)
dtbo-y += pineapple-kiwi-cnss.dtbo
dtbo-y += pineapplep-hdk-kiwi-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_SUN),y)
dtbo-y += sun-peach-cnss.dtbo
dtbo-y += sun-peach-cnss-v8.dtbo
endif

ifeq ($(CONFIG_ARCH_CLIFFS),y)
dtbo-y += cliffs-kiwi-cnss.dtbo
dtbo-y += cliffs-qca6750.dtbo
endif

ifeq ($(CONFIG_ARCH_MONACO),y)
dtbo-y += monaco-cnss.dtbo
dtbo-y += monaco-standalone-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_LEMANS),y)
ifneq ($(CONFIG_ARCH_QTI_VM),y)
dtbo-y += lemans-cnss.dtbo
endif
endif

ifeq ($(CONFIG_QTI_QUIN_GVM),y)
dtbo-y += sa8155p-vm-cnss.dtbo
dtbo-y += sa8195p-vm-cnss.dtbo
dtbo-y += sa8255p-vm-cnss.dtbo
dtbo-y += sa8295p-vm-cnss.dtbo
dtbo-y += direwolf-vm-dual-cnss.dtbo
dtbo-y += lemans-vm-cnss.dtbo
endif


ifeq ($(CONFIG_ARCH_SA525),y)
dtbo-y += sa525m-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_KONA),y)
dtbo-y += kona-iot-vc-cnss.dtbo
endif

ifeq ($(CONFIG_ARCH_TRINKET),y)
dtbo-y += trinket-cnss.dtbo
endif

always-y	:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
