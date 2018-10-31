dtb-$(CONFIG_MACH_SONY_AKARI)		+= sdm845-tama-akari_generic.dtb \
	sdm845-v2-tama-akari_generic.dtb \
	sdm845-v2.1-tama-akari_generic.dtb
dtb-$(CONFIG_MACH_SONY_APOLLO)		+= sdm845-tama-apollo_generic.dtb \
	sdm845-v2-tama-apollo_generic.dtb \
	sdm845-v2.1-tama-apollo_generic.dtb

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_MACH_SONY_AKARI) += \
		sdm845-tama-akari_generic-overlay.dtbo \
		sdm845-v2-tama-akari_generic-overlay.dtbo \
		sdm845-v2.1-tama-akari_generic-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_APOLLO) += \
		sdm845-tama-apollo_generic-overlay.dtbo \
		sdm845-v2-tama-apollo_generic-overlay.dtbo \
		sdm845-v2.1-tama-apollo_generic-overlay.dtbo

ifeq ($(CONFIG_MACH_SONY_AKARI),y)
sdm845-tama-akari_generic-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-akari_generic-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-akari_generic-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_APOLLO),y)
sdm845-tama-apollo_generic-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-apollo_generic-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-apollo_generic-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
