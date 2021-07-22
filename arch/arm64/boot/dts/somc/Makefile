ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_MACH_SONY_PDX201) += \
	trinket-seine-pdx201_generic-overlay.dtbo

ifeq ($(CONFIG_MACH_SONY_PDX201),y)
trinket-seine-pdx201_generic-overlay.dtbo-base := ../qcom/trinket.dtb
endif

else
dtb-$(CONFIG_MACH_SONY_PDX201) += \
	trinket-seine-pdx201_generic.dtb
endif

ifeq ($(CONFIG_ARM64),y)
always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif
clean-files	:= *.dtb *.dtbo
