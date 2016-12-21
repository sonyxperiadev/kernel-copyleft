obj-$(CONFIG_MSM_VIDC_V4L2) := 	msm_v4l2_vidc.o \
				msm_vidc_common.o \
				msm_vidc.o \
				msm_vdec.o \
				msm_venc.o \
				msm_smem.o \
				msm_vidc_debug.o \
				msm_vidc_res_parse.o \
				venus_hfi.o \
				hfi_response_handler.o \
				hfi_packetization.o \
				vidc_hfi.o \
				q6_hfi.o \
				venus_boot.o \
				msm_vidc_dcvs.o \

ifneq ($(TARGET_BUILD_VARIANT), user)
CFLAGS_msm_vidc_common.o := \
				-DSYSTEM_RESET_WITH_VENUS_CRASH
endif

obj-$(CONFIG_MSM_VIDC_VMEM) += vmem/
