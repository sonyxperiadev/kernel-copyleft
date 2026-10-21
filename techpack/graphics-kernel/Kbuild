# SPDX-License-Identifier: GPL-2.0-only

KDIR := $(TOP)/kernel_platform/common

ifeq ($(KGSL_PATH),)
KGSL_PATH=$(src)
endif

# If we're not in an Android tree, select KGSL config
ifeq ($(ANDROID_BUILD_TOP),)
CONFIG_QCOM_KGSL = m
endif

ifeq ($(CONFIG_ARCH_WAIPIO), y)
	include $(KGSL_PATH)/config/gki_waipiodisp.conf
endif
ifeq ($(CONFIG_ARCH_KALAMA), y)
	include $(KGSL_PATH)/config/gki_kalama.conf
endif
ifeq ($(CONFIG_ARCH_SA8155), y)
	include $(KGSL_PATH)/config/gki_sa8155.conf
endif
ifeq ($(CONFIG_ARCH_KHAJE), y)
	include $(KGSL_PATH)/config/gki_khajedisp.conf
endif
ifeq ($(CONFIG_ARCH_SA8195), y)
	include $(KGSL_PATH)/config/gki_sa8155.conf
endif
ifeq ($(CONFIG_ARCH_MONACO), y)
	include $(KGSL_PATH)/config/gki_monaco.conf
endif
ifeq ($(CONFIG_ARCH_KONA), y)
        include $(KGSL_PATH)/config/gki_kona.conf
endif

ccflags-y += -I$(KGSL_PATH) -I$(KGSL_PATH)/include/linux -I$(KGSL_PATH)/include -I$(KERNEL_SRC)/drivers/devfreq -I$(KERNEL_SRC)/drivers/iommu

obj-$(CONFIG_QCOM_KGSL) += msm_kgsl.o

msm_kgsl-y = \
	kgsl.o \
	kgsl_bus.o \
	kgsl_drawobj.o \
	kgsl_events.o \
	kgsl_eventlog.o \
	kgsl_gmu_core.o \
	kgsl_ioctl.o \
	kgsl_mmu.o \
	kgsl_pwrctrl.o \
	kgsl_pwrscale.o \
	kgsl_regmap.o \
	kgsl_sharedmem.o \
	kgsl_snapshot.o \
	kgsl_timeline.o \
	kgsl_trace.o \
	kgsl_util.o \
	kgsl_vbo.o

msm_kgsl-$(CONFIG_COMPAT) += kgsl_compat.o
msm_kgsl-$(CONFIG_DEBUG_FS) += kgsl_debugfs.o
msm_kgsl-$(CONFIG_ARM_SMMU) += kgsl_iommu.o
msm_kgsl-$(CONFIG_SYNC_FILE) += kgsl_sync.o
msm_kgsl-$(CONFIG_QCOM_KGSL_PROCESS_RECLAIM) += kgsl_reclaim.o

ifndef CONFIG_QCOM_KGSL_USE_SHMEM
	msm_kgsl-y += kgsl_pool.o
endif

msm_kgsl-y += \
	adreno.o \
	adreno_a3xx.o \
	adreno_a3xx_perfcounter.o \
	adreno_a3xx_ringbuffer.o \
	adreno_a3xx_snapshot.o \
	adreno_a5xx.o \
	adreno_a5xx_perfcounter.o \
	adreno_a5xx_preempt.o \
	adreno_a5xx_ringbuffer.o \
	adreno_a5xx_snapshot.o \
	adreno_a6xx.o \
	adreno_a6xx_gmu.o \
	adreno_a6xx_gmu_snapshot.o \
	adreno_a6xx_hfi.o \
	adreno_a6xx_hwsched.o \
	adreno_a6xx_hwsched_hfi.o \
	adreno_a6xx_perfcounter.o \
	adreno_a6xx_preempt.o \
	adreno_a6xx_rgmu.o \
	adreno_a6xx_ringbuffer.o \
	adreno_a6xx_rpmh.o \
	adreno_a6xx_snapshot.o \
	adreno_cp_parser.o \
	adreno_dispatch.o \
	adreno_drawctxt.o \
	adreno_gen7.o \
	adreno_gen7_gmu.o \
	adreno_gen7_gmu_snapshot.o \
	adreno_gen7_hfi.o \
	adreno_gen7_hwsched.o \
	adreno_gen7_hwsched_hfi.o \
	adreno_gen7_perfcounter.o \
	adreno_gen7_preempt.o \
	adreno_gen7_ringbuffer.o \
	adreno_gen7_rpmh.o \
	adreno_gen7_snapshot.o \
	adreno_hwsched.o \
	adreno_ioctl.o \
	adreno_perfcounter.o \
	adreno_ringbuffer.o \
	adreno_snapshot.o \
	adreno_sysfs.o \
	adreno_trace.o \
	governor_msm_adreno_tz.o \
	governor_gpubw_mon.o

msm_kgsl-$(CONFIG_COMPAT) += adreno_compat.o
msm_kgsl-$(CONFIG_QCOM_KGSL_CORESIGHT) += adreno_coresight.o
msm_kgsl-$(CONFIG_QCOM_KGSL_CORESIGHT) += adreno_a3xx_coresight.o
msm_kgsl-$(CONFIG_QCOM_KGSL_CORESIGHT) += adreno_a5xx_coresight.o
msm_kgsl-$(CONFIG_QCOM_KGSL_CORESIGHT) += adreno_a6xx_coresight.o
msm_kgsl-$(CONFIG_QCOM_KGSL_CORESIGHT) += adreno_gen7_coresight.o
msm_kgsl-$(CONFIG_DEBUG_FS) += adreno_debugfs.o adreno_profile.o
