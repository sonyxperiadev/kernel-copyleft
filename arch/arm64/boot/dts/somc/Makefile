dtb-$(CONFIG_MACH_SONY_AKARI)		+= sdm845-tama-akari_generic.dtb \
	sdm845-v2-tama-akari_generic.dtb \
	sdm845-v2.1-tama-akari_generic.dtb
dtb-$(CONFIG_MACH_SONY_AKARI_DCM)	+= sdm845-tama-akari_dcm.dtb \
	sdm845-v2-tama-akari_dcm.dtb \
	sdm845-v2.1-tama-akari_dcm.dtb
dtb-$(CONFIG_MACH_SONY_AKARI_KDDI)	+= sdm845-tama-akari_kddi.dtb \
	sdm845-v2-tama-akari_kddi.dtb \
	sdm845-v2.1-tama-akari_kddi.dtb
dtb-$(CONFIG_MACH_SONY_AKARI_SBM)	+= sdm845-tama-akari_sbm.dtb \
	sdm845-v2-tama-akari_sbm.dtb \
	sdm845-v2.1-tama-akari_sbm.dtb
dtb-$(CONFIG_MACH_SONY_APOLLO)		+= sdm845-tama-apollo_generic.dtb \
	sdm845-v2-tama-apollo_generic.dtb \
	sdm845-v2.1-tama-apollo_generic.dtb
dtb-$(CONFIG_MACH_SONY_APOLLO_DCM)	+= sdm845-tama-apollo_dcm.dtb \
	sdm845-v2-tama-apollo_dcm.dtb \
	sdm845-v2.1-tama-apollo_dcm.dtb
dtb-$(CONFIG_MACH_SONY_AURORA)		+= sdm845-tama-aurora_generic.dtb \
	sdm845-v2-tama-aurora_generic.dtb \
	sdm845-v2.1-tama-aurora_generic.dtb
dtb-$(CONFIG_MACH_SONY_AURORA_DCM)	+= sdm845-tama-aurora_dcm.dtb \
	sdm845-v2-tama-aurora_dcm.dtb \
	sdm845-v2.1-tama-aurora_dcm.dtb
dtb-$(CONFIG_MACH_SONY_AURORA_KDDI)	+= sdm845-tama-aurora_kddi.dtb \
	sdm845-v2-tama-aurora_kddi.dtb \
	sdm845-v2.1-tama-aurora_kddi.dtb
dtb-$(CONFIG_MACH_SONY_AKATSUKI)	+= sdm845-tama-akatsuki_generic.dtb \
	sdm845-v2-tama-akatsuki_generic.dtb \
	sdm845-v2.1-tama-akatsuki_generic.dtb

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_MACH_SONY_AKARI) += \
		sdm845-tama-akari_generic-overlay.dtbo \
		sdm845-v2-tama-akari_generic-overlay.dtbo \
		sdm845-v2.1-tama-akari_generic-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AKARI_DCM) += \
		sdm845-tama-akari_dcm-overlay.dtbo \
		sdm845-v2-tama-akari_dcm-overlay.dtbo \
		sdm845-v2.1-tama-akari_dcm-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AKARI_KDDI) += \
		sdm845-tama-akari_kddi-overlay.dtbo \
		sdm845-v2-tama-akari_kddi-overlay.dtbo \
		sdm845-v2.1-tama-akari_kddi-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AKARI_SBM) += \
		sdm845-tama-akari_sbm-overlay.dtbo \
		sdm845-v2-tama-akari_sbm-overlay.dtbo \
		sdm845-v2.1-tama-akari_sbm-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_APOLLO) += \
		sdm845-tama-apollo_generic-overlay.dtbo \
		sdm845-v2-tama-apollo_generic-overlay.dtbo \
		sdm845-v2.1-tama-apollo_generic-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_APOLLO_DCM) += \
		sdm845-tama-apollo_dcm-overlay.dtbo \
		sdm845-v2-tama-apollo_dcm-overlay.dtbo \
		sdm845-v2.1-tama-apollo_dcm-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AURORA) += \
		sdm845-tama-aurora_generic-overlay.dtbo \
		sdm845-v2-tama-aurora_generic-overlay.dtbo \
		sdm845-v2.1-tama-aurora_generic-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AURORA_DCM) += \
		sdm845-tama-aurora_dcm-overlay.dtbo \
		sdm845-v2-tama-aurora_dcm-overlay.dtbo \
		sdm845-v2.1-tama-aurora_dcm-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AURORA_KDDI) += \
		sdm845-tama-aurora_kddi-overlay.dtbo \
		sdm845-v2-tama-aurora_kddi-overlay.dtbo \
		sdm845-v2.1-tama-aurora_kddi-overlay.dtbo
	dtbo-$(CONFIG_MACH_SONY_AKATSUKI) += \
		sdm845-tama-akatsuki_generic-overlay.dtbo \
		sdm845-v2-tama-akatsuki_generic-overlay.dtbo \
		sdm845-v2.1-tama-akatsuki_generic-overlay.dtbo

ifeq ($(CONFIG_MACH_SONY_AKARI),y)
sdm845-tama-akari_generic-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-akari_generic-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-akari_generic-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AKARI_DCM),y)
sdm845-tama-akari_dcm-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-akari_dcm-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-akari_dcm-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AKARI_KDDI),y)
sdm845-tama-akari_kddi-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-akari_kddi-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-akari_kddi-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AKARI_SBM),y)
sdm845-tama-akari_sbm-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-akari_sbm-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-akari_sbm-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_APOLLO),y)
sdm845-tama-apollo_generic-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-apollo_generic-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-apollo_generic-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_APOLLO_DCM),y)
sdm845-tama-apollo_dcm-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-apollo_dcm-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-apollo_dcm-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AURORA),y)
sdm845-tama-aurora_generic-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-aurora_generic-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-aurora_generic-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AURORA_DCM),y)
sdm845-tama-aurora_dcm-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-aurora_dcm-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-aurora_dcm-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AURORA_KDDI),y)
sdm845-tama-aurora_kddi-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-aurora_kddi-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-aurora_kddi-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif
ifeq ($(CONFIG_MACH_SONY_AKATSUKI),y)
sdm845-tama-akatsuki_generic-overlay.dtbo-base := ../qcom/sdm845.dtb
sdm845-v2-tama-akatsuki_generic-overlay.dtbo-base := ../qcom/sdm845-v2.dtb
sdm845-v2.1-tama-akatsuki_generic-overlay.dtbo-base := ../qcom/sdm845-v2.1.dtb
endif

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
