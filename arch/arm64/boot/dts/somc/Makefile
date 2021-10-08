ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_MACH_SONY_KUMANO_CDB) += \
	sm8150-kumano-cdb-overlay.dtbo \
	sm8150-sdx50m-kumano-plus-cdb-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_GRIFFIN) += \
	sm8150-kumano-griffin_generic-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_GRIFFIN_K) += \
	sm8150-kumano-griffin_k-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_GRIFFIN_N) += \
	sm8150-kumano-griffin_n-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_GRIFFIN_S) += \
	sm8150-kumano-griffin_s-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_GRIFFIN_P) += \
	sm8150-kumano-griffin_p-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_SPHINX) += \
	sm8150-sdx50m-kumano-plus-sphinx-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_HORUS) += \
        sm8150-sdx50m-kumano-plus-horus-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_BAHAMUT) += \
	sm8150-kumano-bahamut_generic-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_BAHAMUT_K) += \
	sm8150-kumano-bahamut_k-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_BAHAMUT_N) += \
	sm8150-kumano-bahamut_n-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_BAHAMUT_S) += \
	sm8150-kumano-bahamut_s-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_BAHAMUT_M) += \
	sm8150-kumano-bahamut_m-overlay.dtbo

ifeq ($(CONFIG_MACH_SONY_KUMANO_CDB),y)
sm8150-kumano-cdb-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
sm8150-sdx50m-kumano-plus-cdb-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_GRIFFIN),y)
sm8150-kumano-griffin_generic-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_GRIFFIN_K),y)
sm8150-kumano-griffin_k-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_GRIFFIN_N),y)
sm8150-kumano-griffin_n-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_GRIFFIN_S),y)
sm8150-kumano-griffin_s-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_GRIFFIN_P),y)
sm8150-kumano-griffin_p-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_SPHINX),y)
sm8150-sdx50m-kumano-plus-sphinx-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_HORUS),y)
sm8150-sdx50m-kumano-plus-horus-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_BAHAMUT),y)
sm8150-kumano-bahamut_generic-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_BAHAMUT_K),y)
sm8150-kumano-bahamut_k-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_BAHAMUT_N),y)
sm8150-kumano-bahamut_n-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_BAHAMUT_S),y)
sm8150-kumano-bahamut_s-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif
ifeq ($(CONFIG_MACH_SONY_BAHAMUT_M),y)
sm8150-kumano-bahamut_m-overlay.dtbo-base := ../qcom/sm8150.dtb ../qcom/sm8150-v2.dtb
endif

else
dtb-$(CONFIG_MACH_SONY_KUMANO_CDB) += \
	sm8150-kumano-cdb.dtb \
	sm8150-v2-kumano-cdb.dtb \
	sm8150-sdx50m-kumano-plus-cdb.dtb \
	sm8150-sdx50m-v2-kumano-plus-cdb.dtb
dtb-$(CONFIG_MACH_SONY_GRIFFIN) += \
	sm8150-kumano-griffin_generic.dtb \
	sm8150-v2-kumano-griffin_generic.dtb
dtb-$(CONFIG_MACH_SONY_GRIFFIN_K) += \
	sm8150-kumano-griffin_k.dtb \
	sm8150-v2-kumano-griffin_k.dtb
dtb-$(CONFIG_MACH_SONY_GRIFFIN_N) += \
	sm8150-kumano-griffin_n.dtb \
	sm8150-v2-kumano-griffin_n.dtb
dtb-$(CONFIG_MACH_SONY_GRIFFIN_S) += \
	sm8150-kumano-griffin_s.dtb \
	sm8150-v2-kumano-griffin_s.dtb
dtb-$(CONFIG_MACH_SONY_GRIFFIN_P) += \
	sm8150-kumano-griffin_p.dtb \
	sm8150-v2-kumano-griffin_p.dtb
dtb-$(CONFIG_MACH_SONY_SPHINX) += \
	sm8150-sdx50m-kumano-plus-sphinx.dtb \
	sm8150-sdx50m-v2-kumano-plus-sphinx.dtb
dtb-$(CONFIG_MACH_SONY_HORUS) += \
        sm8150-sdx50m-kumano-plus-horus.dtb \
        sm8150-sdx50m-v2-kumano-plus-horus.dtb
dtb-$(CONFIG_MACH_SONY_BAHAMUT) += \
	sm8150-kumano-bahamut_generic.dtb \
	sm8150-v2-kumano-bahamut_generic.dtb
dtb-$(CONFIG_MACH_SONY_BAHAMUT_K) += \
	sm8150-kumano-bahamut_k.dtb \
	sm8150-v2-kumano-bahamut_k.dtb
dtb-$(CONFIG_MACH_SONY_BAHAMUT_N) += \
	sm8150-kumano-bahamut_n.dtb \
	sm8150-v2-kumano-bahamut_n.dtb
dtb-$(CONFIG_MACH_SONY_BAHAMUT_S) += \
	sm8150-kumano-bahamut_s.dtb \
	sm8150-v2-kumano-bahamut_s.dtb
dtb-$(CONFIG_MACH_SONY_BAHAMUT_M) += \
	sm8150-kumano-bahamut_m.dtb \
	sm8150-v2-kumano-bahamut_m.dtb
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
