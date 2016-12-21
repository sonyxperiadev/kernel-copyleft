ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_ARCH_MSM8916) += msm8916-cdp.dtb \
	msm8939-mtp.dtb \
	msm8939-mtp-smb1360.dtb \
	msm8939-v3.0-mtp.dtb \
	msm8939-v3.0-mtp-smb1360.dtb \
	msm8936-mtp.dtb \
	msm8936-mtp-smb1360.dtb \
	msmtellurium-sim.dtb \
	msmtellurium-rumi.dtb \

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
