/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/msm_ssbi.h>
#include <linux/regulator/msm-gpio-regulator.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slimbus/slimbus.h>
#include <linux/bootmem.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_CLEARPAD
#include <linux/clearpad.h>
#endif
#include <linux/cyttsp-qc.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <linux/platform_data/qcom_wcnss_device.h>
#include <linux/leds.h>
#include <linux/leds-pm8xxx.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/msm_tsens.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#include <linux/msm_thermal.h>
#ifdef CONFIG_VIBRATOR_LC898300
#include <linux/vibrator-lc898300.h>
#endif
#ifdef CONFIG_INPUT_BMA250_NG
#include <linux/bma250_ng.h>
#endif
#ifdef CONFIG_INPUT_AKM8972
#include <linux/akm8972.h>
#endif
#if defined CONFIG_SENSORS_MPU3050 || defined CONFIG_SENSORS_MPU6050
#include <linux/mpu.h>
#include "gyro-sony_common.h"
#endif
#ifdef CONFIG_INPUT_APDS9702
#include <linux/apds9702.h>
#endif

#include <linux/persistent_ram.h>
#include <linux/list.h>

#ifdef CONFIG_LEDS_AS3676
#include <linux/leds-as3676.h>
#endif
#ifdef CONFIG_RMI4_BUS
#include <linux/rmi4/rmi4.h>
#include <linux/rmi4/rmi4_i2c.h>
#include <linux/rmi4/rmi4_function_54.h>
#include <linux/rmi4/rmi4_watchdog.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/rmi4/rmi4_early_suspend.h>
#endif
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/hardware/gic.h>
#include <asm/mach/mmc.h>

#include <mach/board.h>
#include <mach/msm_tspp.h>
#include <mach/msm_iomap.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <linux/console.h>
#include <mach/msm_serial_hs_lite.h>
#include <linux/serial_core.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#include <mach/socinfo.h>
#include <mach/rpm.h>
#include <mach/gpiomux.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_memtypes.h>
#include <mach/dma.h>
#include <mach/msm_dsps.h>
#include <mach/msm_xo.h>
#include <mach/restart.h>

#ifdef CONFIG_WCD9310_CODEC
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#endif

#include <linux/smsc3503.h>
#include <linux/msm_ion.h>
#include <mach/ion.h>
#include <mach/mdm2.h>
#include <mach/mdm-peripheral.h>
#include <mach/msm_rtb.h>
#include <mach/msm_cache_dump.h>
#include <mach/scm.h>
#include <mach/iommu_domains.h>
#ifdef CONFIG_INPUT_SIMPLE_REMOTE
#include <mach/simple_remote_msm8960_pf.h>
#endif
#include <mach/kgsl.h>
#include <linux/fmem.h>

#include "timer.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "spm.h"
#include "board-8960.h"
#include "pm.h"
#include <mach/cpuidle.h>
#include "rpm_resources.h"
#include <mach/mpm.h>
#include "clock.h"
#include "smd_private.h"
#include "pm-boot.h"
#include "board-blue-usb.h"
#include "msm_watchdog.h"

#ifdef CONFIG_LM3560_FLASHLED
#include <linux/lm3560.h>
#endif

#if defined(CONFIG_BT) && defined(CONFIG_BT_HCIUART_ATH3K)
#include <linux/wlan_plat.h>
#include <linux/mutex.h>
#endif
#ifdef CONFIG_FB_MSM_MHL_SII8334
#include <linux/mhl_sii8334.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/usb/msm_hsusb.h>
#endif

#ifdef CONFIG_USB_NCP373
#include <linux/usb/ncp373.h>
#endif

#ifdef CONFIG_NFC_PN544
#include <linux/pn544.h>
#endif

#ifdef CONFIG_RAMDUMP_TAGS
#include "board-rdtags.h"
#endif

#ifdef CONFIG_NFC_PN544
#define NXP_GPIO_NFC_EN		PM8921_GPIO_PM_TO_SYS(33)
#define NXP_GPIO_NFC_FWDL_EN	19
#define NXP_GPIO_NFC_IRQ	106
#endif

static struct platform_device msm_fm_platform_init = {
	.name = "iris_fm",
	.id   = -1,
};

#define MSM_PMEM_ADSP_SIZE         0x7800000
#define MSM_PMEM_AUDIO_SIZE        0x1000 /* (4KB) */
#define MSM_PMEM_SIZE 0x2800000 /* 40 Mbytes */
#define MSM_LIQUID_PMEM_SIZE 0x4000000 /* 64 Mbytes */
#define MSM_HDMI_PRIM_PMEM_SIZE 0x4000000 /* 64 Mbytes */

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
#define HOLE_SIZE	0x20000
#define MSM_CONTIG_MEM_SIZE  0x65000
#ifdef CONFIG_MSM_IOMMU
#define MSM_ION_MM_SIZE            0x4000000 /* Need to be multiple of 64K */
#define MSM_ION_SF_SIZE            0x0
#define MSM_ION_QSECOM_SIZE        0x780000 /* (7.5MB) */
#define MSM_ION_HEAP_NUM	8
#else
#define MSM_ION_MM_SIZE            MSM_PMEM_ADSP_SIZE
#define MSM_ION_SF_SIZE            MSM_PMEM_SIZE
#define MSM_ION_QSECOM_SIZE        0x600000 /* (6MB) */
#define MSM_ION_HEAP_NUM	8
#endif
#define MSM_ION_MM_FW_SIZE	(0x200000 - HOLE_SIZE) /* 128kb */
#define MSM_ION_MFC_SIZE	SZ_8K
#define MSM_ION_AUDIO_SIZE	0x4CF000

#define MSM_LIQUID_ION_MM_SIZE (MSM_ION_MM_SIZE + 0x600000)
#define MSM_LIQUID_ION_SF_SIZE MSM_LIQUID_PMEM_SIZE
#define MSM_HDMI_PRIM_ION_SF_SIZE MSM_HDMI_PRIM_PMEM_SIZE

#define MSM_MM_FW_SIZE		(0x200000 - HOLE_SIZE) /* 2mb -128kb*/
#define MSM8960_FIXED_AREA_START (0xa0000000 - (MSM_ION_MM_FW_SIZE + \
							HOLE_SIZE))
#define MAX_FIXED_AREA_SIZE	0x10000000
#define MSM8960_FW_START	MSM8960_FIXED_AREA_START
#define MSM_ION_ADSP_SIZE	SZ_8M

/* Section: Vibrator */
#if defined(CONFIG_VIBRATOR_LC898300)
struct lc898300_vib_cmd lc898300_vib_cmd_data = {
	.vib_cmd_intensity = VIB_CMD_PWM_10_15,
	.vib_cmd_resonance = VIB_CMD_FREQ_150,
	.vib_cmd_startup   = VIB_CMD_STTIME_5,
	.vib_cmd_brake     = VIB_CMD_ATBR | VIB_CMD_BRTIME_2 |
						VIB_CMD_BRPWR_15_15,
	.vib_cmd_stops     = VIB_CMD_ATSNUM_8_10 | VIB_CMD_ATSOFF,
};
#endif

static unsigned msm_ion_sf_size = MSM_ION_SF_SIZE;
#else
#define MSM_CONTIG_MEM_SIZE  0x110C000
#define MSM_ION_HEAP_NUM	1
#endif

#ifdef CONFIG_KERNEL_MSM_CONTIG_MEM_REGION
static unsigned msm_contig_mem_size = MSM_CONTIG_MEM_SIZE;
static int __init msm_contig_mem_size_setup(char *p)
{
	msm_contig_mem_size = memparse(p, NULL);
	return 0;
}
early_param("msm_contig_mem_size", msm_contig_mem_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_size = MSM_PMEM_SIZE;
static unsigned pmem_param_set;
static int __init pmem_size_setup(char *p)
{
	pmem_size = memparse(p, NULL);
	pmem_param_set = 1;
	return 0;
}
early_param("pmem_size", pmem_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device msm8960_android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device msm8960_android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device msm8960_android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/

struct fmem_platform_data msm8960_fmem_pdata = {
};

#define DSP_RAM_BASE_8960 0x8da00000
#define DSP_RAM_SIZE_8960 0x1800000
static int dspcrashd_pdata_8960 = 0xDEADDEAD;

static struct resource resources_dspcrashd_8960[] = {
	{
		.name   = "msm_dspcrashd",
		.start  = DSP_RAM_BASE_8960,
		.end    = DSP_RAM_BASE_8960 + DSP_RAM_SIZE_8960,
		.flags  = IORESOURCE_DMA,
	},
};

static struct platform_device msm_device_dspcrashd_8960 = {
	.name           = "msm_dspcrashd",
	.num_resources  = ARRAY_SIZE(resources_dspcrashd_8960),
	.resource       = resources_dspcrashd_8960,
	.dev = { .platform_data = &dspcrashd_pdata_8960 },
};

static struct memtype_reserve msm8960_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init reserve_rtb_memory(void)
{
#if defined(CONFIG_MSM_RTB)
	msm8960_reserve_table[MEMTYPE_EBI1].size += msm8960_rtb_pdata.size;
#endif
}

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	android_pmem_adsp_pdata.size = pmem_adsp_size;

	if (!pmem_param_set) {
		if (machine_is_msm8960_liquid())
			pmem_size = MSM_LIQUID_PMEM_SIZE;
		if (msm8960_hdmi_as_primary_selected())
			pmem_size = MSM_HDMI_PRIM_PMEM_SIZE;
	}

	android_pmem_pdata.size = pmem_size;
	android_pmem_audio_pdata.size = MSM_PMEM_AUDIO_SIZE;
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/
}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm8960_reserve_table[p->memory_type].size += p->size;
}
#endif /*CONFIG_MSM_MULTIMEDIA_USE_ION*/
#endif /*CONFIG_ANDROID_PMEM*/

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
#endif
	msm8960_reserve_table[MEMTYPE_EBI1].size += msm_contig_mem_size;
#endif
}

static int msm8960_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

#define FMEM_ENABLED 0

#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_cp_heap_pdata cp_mm_msm8960_ion_pdata = {
	.permission_type = IPT_TYPE_MM_CARVEOUT,
	.align = SZ_64K,
	.reusable = FMEM_ENABLED,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_MIDDLE,
	.iommu_map_all = 1,
	.iommu_2x_map_domain = VIDEO_DOMAIN,
	.is_cma = 1,
	.no_nonsecure_alloc = 1,
};

static struct ion_cp_heap_pdata cp_mfc_msm8960_ion_pdata = {
	.permission_type = IPT_TYPE_MFC_SHAREDMEM,
	.align = PAGE_SIZE,
	.reusable = 0,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_HIGH,
	.no_nonsecure_alloc = 1,
};

static struct ion_co_heap_pdata co_msm8960_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
	.mem_is_fmem = 0,
};

static struct ion_co_heap_pdata fw_co_msm8960_ion_pdata = {
	.adjacent_mem_id = ION_CP_MM_HEAP_ID,
	.align = SZ_128K,
	.mem_is_fmem = FMEM_ENABLED,
	.fixed_position = FIXED_LOW,
};
#endif

static u64 msm_dmamask = DMA_BIT_MASK(32);

static struct platform_device ion_mm_heap_device = {
	.name = "ion-mm-heap-device",
	.id = -1,
	.dev = {
		.dma_mask = &msm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

static struct platform_device ion_adsp_heap_device = {
	.name = "ion-adsp-heap-device",
	.id = -1,
	.dev = {
		.dma_mask = &msm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

/**
 * These heaps are listed in the order they will be allocated. Due to
 * video hardware restrictions and content protection the FW heap has to
 * be allocated adjacent (below) the MM heap and the MFC heap has to be
 * allocated after the MM heap to ensure MFC heap is not more than 256MB
 * away from the base address of the FW heap.
 * However, the order of FW heap and MM heap doesn't matter since these
 * two heaps are taken care of by separate code to ensure they are adjacent
 * to each other.
 * Don't swap the order unless you know what you are doing!
 */
struct ion_platform_heap msm8960_heaps[] = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		{
			.id	= ION_CP_MM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MM_HEAP_NAME,
			.size	= MSM_ION_MM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mm_msm8960_ion_pdata,
			.priv	= &ion_mm_heap_device.dev,
		},
		{
			.id	= ION_MM_FIRMWARE_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_MM_FIRMWARE_HEAP_NAME,
			.size	= MSM_ION_MM_FW_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &fw_co_msm8960_ion_pdata,
		},
		{
			.id	= ION_CP_MFC_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MFC_HEAP_NAME,
			.size	= MSM_ION_MFC_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mfc_msm8960_ion_pdata,
		},
#ifndef CONFIG_MSM_IOMMU
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.size	= MSM_ION_SF_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
		},
#endif
		{
			.id	= ION_IOMMU_HEAP_ID,
			.type	= ION_HEAP_TYPE_IOMMU,
			.name	= ION_IOMMU_HEAP_NAME,
		},
		{
			.id	= ION_QSECOM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_QSECOM_HEAP_NAME,
			.size	= MSM_ION_QSECOM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
		},
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.size	= MSM_ION_AUDIO_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
		},
		{
			.id     = ION_ADSP_HEAP_ID,
			.type   = ION_HEAP_TYPE_DMA,
			.name   = ION_ADSP_HEAP_NAME,
			.size   = MSM_ION_ADSP_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_msm8960_ion_pdata,
			.priv	= &ion_adsp_heap_device.dev,
		},
#endif
};

static struct ion_platform_data msm8960_ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.heaps = msm8960_heaps,
};

static struct platform_device msm8960_ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &msm8960_ion_pdata },
};
#endif

struct platform_device msm8960_fmem_device = {
	.name = "fmem",
	.id = 1,
	.dev = { .platform_data = &msm8960_fmem_pdata },
};

#ifdef CONFIG_INPUT_SIMPLE_REMOTE
#define PLUG_DET_ENA_PIN  -1
#define PLUG_DET_READ_PIN PM8921_GPIO_PM_TO_SYS(38)
#define BUTTON_DET_PIN    -1
#define PM8921_HSED_CNTR_BASE 0x134

int simple_remote_pf_initialize_gpio(struct simple_remote_platform_data *data)
{
	int err = 0;
	if (!data ||
	    -1 == data->headset_detect_read_pin) {
		pr_err("%s: Invalid inparameter (GPIO Pins)."
		       " Aborting!\n", __func__);
		return -EIO;
	}

	err = gpio_request(data->headset_detect_read_pin,
			   "Simple_remote_plug_detect_read");
	if (err) {
		pr_err("%s: %d Request hs-detect_read pin",
		       __func__, err);
		goto out;
	}

	err = gpio_direction_input(data->headset_detect_read_pin);
	if (err) {
		pr_err("%s: %d Set hs-detect pin as input\n",
		       __func__, err);
		goto out_hs_det_read;
	}

	return err;

out_hs_det_read:
	gpio_free(data->headset_detect_read_pin);
out:
	return err;
}

void simple_remote_pf_deinitialize_gpio(
	struct simple_remote_platform_data *data)
{
	gpio_free(data->headset_detect_read_pin);
}

struct simple_remote_platform_data simple_remote_pf_data = {
	.headset_detect_enable_pin = PLUG_DET_ENA_PIN,
	.headset_detect_read_pin = PLUG_DET_READ_PIN,
	.button_detect_irq = BUTTON_DET_PIN,
	.adc_channel = CHANNEL_MPP_1,
	.hsed_base = PM8921_HSED_CNTR_BASE,
	.initialize = &simple_remote_pf_initialize_gpio,
	.deinitialize = &simple_remote_pf_deinitialize_gpio,
#ifdef CONFIG_SIMPLE_REMOTE_INVERT_PLUG_DETECTION_STATE
	.invert_plug_det = 1,
#else
	.invert_plug_det = 0,
#endif
};
#endif

static void __init adjust_mem_for_liquid(void)
{
	unsigned int i;

	if (!pmem_param_set) {
		if (machine_is_msm8960_liquid())
			msm_ion_sf_size = MSM_LIQUID_ION_SF_SIZE;

		if (msm8960_hdmi_as_primary_selected())
			msm_ion_sf_size = MSM_HDMI_PRIM_ION_SF_SIZE;

		if (machine_is_msm8960_liquid() ||
			msm8960_hdmi_as_primary_selected()) {
			for (i = 0; i < msm8960_ion_pdata.nr; i++) {
				if (msm8960_ion_pdata.heaps[i].id ==
							ION_SF_HEAP_ID) {
					msm8960_ion_pdata.heaps[i].size =
						msm_ion_sf_size;
					pr_debug("msm_ion_sf_size 0x%x\n",
						msm_ion_sf_size);
					break;
				}
			}
		}
	}
}

static void __init reserve_mem_for_ion(enum ion_memory_types mem_type,
				      unsigned long size)
{
	msm8960_reserve_table[mem_type].size += size;
}

static void __init msm8960_reserve_fixed_area(unsigned long fixed_area_size)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	int ret;

	if (fixed_area_size > MAX_FIXED_AREA_SIZE)
		panic("fixed area size is larger than %dM\n",
			MAX_FIXED_AREA_SIZE >> 20);

	reserve_info->fixed_area_size = fixed_area_size;
	reserve_info->fixed_area_start = MSM8960_FW_START;

	ret = memblock_remove(reserve_info->fixed_area_start,
		reserve_info->fixed_area_size);
	BUG_ON(ret);
#endif
}

/**
 * Reserve memory for ION and calculate amount of reusable memory for fmem.
 * We only reserve memory for heaps that are not reusable. However, we only
 * support one reusable heap at the moment so we ignore the reusable flag for
 * other than the first heap with reusable flag set. Also handle special case
 * for video heaps (MM,FW, and MFC). Video requires heaps MM and MFC to be
 * at a higher address than FW in addition to not more than 256MB away from the
 * base address of the firmware. This means that if MM is reusable the other
 * two heaps must be allocated in the same region as FW. This is handled by the
 * mem_is_fmem flag in the platform data. In addition the MM heap must be
 * adjacent to the FW heap for content protection purposes.
 */
static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	unsigned int i;
	int ret;
	unsigned int fixed_size = 0;
	unsigned int fixed_low_size, fixed_middle_size, fixed_high_size;
	unsigned long fixed_low_start, fixed_middle_start, fixed_high_start;
	unsigned long cma_alignment;
	unsigned int low_use_cma = 0;
	unsigned int middle_use_cma = 0;
	unsigned int high_use_cma = 0;

	adjust_mem_for_liquid();
	fixed_low_size = 0;
	fixed_middle_size = 0;
	fixed_high_size = 0;

	cma_alignment = PAGE_SIZE << max(MAX_ORDER, pageblock_order);

	for (i = 0; i < msm8960_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap =
						&(msm8960_ion_pdata.heaps[i]);
		int align = SZ_4K;
		int iommu_map_all = 0;
		int adjacent_mem_id = INVALID_HEAP_ID;
		int use_cma = 0;

		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;

			switch ((int) heap->type) {
			case ION_HEAP_TYPE_CP:
				fixed_position = ((struct ion_cp_heap_pdata *)
					heap->extra_data)->fixed_position;
				align = ((struct ion_cp_heap_pdata *)
						heap->extra_data)->align;
				iommu_map_all =
					((struct ion_cp_heap_pdata *)
					heap->extra_data)->iommu_map_all;
				if (((struct ion_cp_heap_pdata *)
					heap->extra_data)->is_cma) {
					heap->size = ALIGN(heap->size,
							cma_alignment);
					use_cma = 1;
				}
				break;
			case ION_HEAP_TYPE_DMA:
					use_cma = 1;
				/* Purposely fall through here */
			case ION_HEAP_TYPE_CARVEOUT:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				adjacent_mem_id = ((struct ion_co_heap_pdata *)
					heap->extra_data)->adjacent_mem_id;
				break;
			default:
				break;
			}

			if (iommu_map_all) {
				if (heap->size & (SZ_64K-1)) {
					heap->size = ALIGN(heap->size, SZ_64K);
					pr_info("Heap %s not aligned to 64K. Adjusting size to %x\n",
						heap->name, heap->size);
				}
			}

			if (fixed_position != NOT_FIXED)
				fixed_size += heap->size;
			else
				reserve_mem_for_ion(MEMTYPE_EBI1, heap->size);

			if (fixed_position == FIXED_LOW) {
				fixed_low_size += heap->size;
				low_use_cma = use_cma;
			} else if (fixed_position == FIXED_MIDDLE) {
				fixed_middle_size += heap->size;
				middle_use_cma = use_cma;
			} else if (fixed_position == FIXED_HIGH) {
				fixed_high_size += heap->size;
				high_use_cma = use_cma;
			} else if (use_cma) {
				/*
				 * Heaps that use CMA but are not part of the
				 * fixed set. Create wherever.
				 */
				dma_declare_contiguous(
					heap->priv,
					heap->size,
					0,
					0xb0000000);
			}
		}
	}

	if (!fixed_size)
		return;

	/*
	 * Given the setup for the fixed area, we can't round up all sizes.
	 * Some sizes must be set up exactly and aligned correctly. Incorrect
	 * alignments are considered a configuration issue
	 */

	fixed_low_start = MSM8960_FIXED_AREA_START;
	if (low_use_cma) {
		BUG_ON(!IS_ALIGNED(fixed_low_start, cma_alignment));
		BUG_ON(!IS_ALIGNED(fixed_low_size + HOLE_SIZE, cma_alignment));
	} else {
		BUG_ON(!IS_ALIGNED(fixed_low_size + HOLE_SIZE, SECTION_SIZE));
		ret = memblock_remove(fixed_low_start,
				      fixed_low_size + HOLE_SIZE);
		BUG_ON(ret);
	}

	fixed_middle_start = fixed_low_start + fixed_low_size + HOLE_SIZE;
	if (middle_use_cma) {
		BUG_ON(!IS_ALIGNED(fixed_middle_start, cma_alignment));
		BUG_ON(!IS_ALIGNED(fixed_middle_size, cma_alignment));
	} else {
		BUG_ON(!IS_ALIGNED(fixed_middle_size, SECTION_SIZE));
		ret = memblock_remove(fixed_middle_start, fixed_middle_size);
		BUG_ON(ret);
	}

	fixed_high_start = fixed_middle_start + fixed_middle_size;
	if (high_use_cma) {
		fixed_high_size = ALIGN(fixed_high_size, cma_alignment);
		BUG_ON(!IS_ALIGNED(fixed_high_start, cma_alignment));
	} else {
		/* This is the end of the fixed area so it's okay to round up */
		fixed_high_size = ALIGN(fixed_high_size, SECTION_SIZE);
		ret = memblock_remove(fixed_high_start, fixed_high_size);
		BUG_ON(ret);
	}



	for (i = 0; i < msm8960_ion_pdata.nr; ++i) {
		struct ion_platform_heap *heap = &(msm8960_ion_pdata.heaps[i]);

		if (heap->extra_data) {
			int fixed_position = NOT_FIXED;
			struct ion_cp_heap_pdata *pdata = NULL;

			switch ((int) heap->type) {
			case ION_HEAP_TYPE_CP:
				pdata =
				(struct ion_cp_heap_pdata *)heap->extra_data;
				fixed_position = pdata->fixed_position;
				break;
			case ION_HEAP_TYPE_CARVEOUT:
			case ION_HEAP_TYPE_DMA:
				fixed_position = ((struct ion_co_heap_pdata *)
					heap->extra_data)->fixed_position;
				break;
			default:
				break;
			}

			switch (fixed_position) {
			case FIXED_LOW:
				heap->base = fixed_low_start;
				break;
			case FIXED_MIDDLE:
				heap->base = fixed_middle_start;
				if (middle_use_cma) {
					ret = dma_declare_contiguous(
						&ion_mm_heap_device.dev,
						heap->size,
						fixed_middle_start,
						0xa0000000);
					WARN_ON(ret);
				}
				pdata->secure_base = fixed_middle_start
							- HOLE_SIZE;
				pdata->secure_size = HOLE_SIZE + heap->size;
				break;
			case FIXED_HIGH:
				heap->base = fixed_high_start;
				break;
			default:
				break;
			}
		}
	}
#endif
}

static void __init reserve_mdp_memory(void)
{
	msm8960_mdp_writeback(msm8960_reserve_table);
}

static void __init reserve_cache_dump_memory(void)
{
#ifdef CONFIG_MSM_CACHE_DUMP
	unsigned int total;

	total = msm8960_cache_dump_pdata.l1_size +
		msm8960_cache_dump_pdata.l2_size;
	msm8960_reserve_table[MEMTYPE_EBI1].size += total;
#endif
}

/*
 * Default configuration of memory size for RDTAGS.
 * If we didn't set up the memory size for RDTAGS in deconfigs,
 * The default value of 512K would be used when RDTAGS is enabled.
 */
#if defined(CONFIG_RAMDUMP_TAGS) && !defined(CONFIG_RAMDUMP_TAGS_SIZE)
#define CONFIG_RAMDUMP_TAGS_SIZE	(512 * SZ_1K)
#endif

#ifdef CONFIG_RAMDUMP_TAGS
static struct resource rdtags_resources[] = {
	[0] = {
		.name   = "rdtags_mem",
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device rdtags_device = {
	.name           = "rdtags",
	.id             = -1,
	.dev = {
		.platform_data = &rdtags_platdata,
	},
};
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define MSM_RAM_CONSOLE_SIZE    (128 * SZ_1K)

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
};

static struct persistent_ram_descriptor pr_descriptor = {
	.name = "ram_console",
	.size = MSM_RAM_CONSOLE_SIZE,
};

static struct persistent_ram ram_console_pram = {
	.ecc_block_size  = 0,
	.ecc_size        = 0,
	.ecc_symsize     = 0,
	.ecc_poly        = 0,
	.num_descs       = 1,
	.descs           = &pr_descriptor,
};
#endif

#ifdef CONFIG_RAMDUMP_CRASH_LOGS
#define MSM_RAMDUMP_LOG_SIZE (16 * SZ_1K)

static struct resource ramdumplog_resources[] = {
	[0] = {
		.name = "amsslog",
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ramdumplog_device = {
	.name = "ramdumplog",
	.id = -1,
};
#endif

#if defined(CONFIG_ANDROID_RAM_CONSOLE) || defined(CONFIG_RAMDUMP_TAGS) || \
defined(CONFIG_RAMDUMP_CRASH_LOGS)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define DEBUG_RAM_CONSOLE_SIZE (MSM_RAM_CONSOLE_SIZE)
#else
#define DEBUG_RAM_CONSOLE_SIZE 0
#endif

#ifdef CONFIG_RAMDUMP_TAGS
#define DEBUG_RAMDUMP_TAGS_SIZE (CONFIG_RAMDUMP_TAGS_SIZE)
#else
#define DEBUG_RAMDUMP_TAGS_SIZE 0
#endif

#ifdef CONFIG_RAMDUMP_CRASH_LOGS
#define DEBUG_RAMDUMP_LOGS_SIZE (MSM_RAMDUMP_LOG_SIZE)
#else
#define DEBUG_RAMDUMP_LOGS_SIZE 0
#endif

#define DEBUG_MEMORY_SIZE (DEBUG_RAM_CONSOLE_SIZE + DEBUG_RAMDUMP_TAGS_SIZE \
	+ DEBUG_RAMDUMP_LOGS_SIZE)

static void reserve_debug_memory(void)
{
	struct membank *mb = &meminfo.bank[meminfo.nr_banks - 1];
	unsigned long bank_end = mb->start + mb->size;
	int ret;

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ram_console_pram.start = bank_end - MSM_RAM_CONSOLE_SIZE,
	ram_console_pram.size = MSM_RAM_CONSOLE_SIZE;
	INIT_LIST_HEAD(&ram_console_pram.node);
	ret = persistent_ram_early_init(&ram_console_pram);
	if (ret) {
		pr_err("Init of persistent RAM for ram_console failed: %d\n",
			ret);
	} else {
		pr_info("ram_console memory reserved: %#x@%#08x\n",
			(unsigned int)ram_console_pram.size,
			(unsigned int)ram_console_pram.start);
		bank_end -= ram_console_pram.size;
	}
#endif

#ifdef CONFIG_RAMDUMP_TAGS
	rdtags_resources[0].start = bank_end - CONFIG_RAMDUMP_TAGS_SIZE;
	rdtags_resources[0].end = bank_end - 1;
	rdtags_device.num_resources  = ARRAY_SIZE(rdtags_resources);
	rdtags_device.resource       = rdtags_resources;

	ret = memblock_reserve(rdtags_resources[0].start,
		CONFIG_RAMDUMP_TAGS_SIZE);
	if (ret) {
		pr_err("Failed to reserve rdtags memory %#x@%#08x\n",
			(unsigned int)CONFIG_RAMDUMP_TAGS_SIZE,
			(unsigned int)rdtags_resources[0].start);
	} else {
		memblock_free(rdtags_resources[0].start,
			CONFIG_RAMDUMP_TAGS_SIZE);
		memblock_remove(rdtags_resources[0].start,
			CONFIG_RAMDUMP_TAGS_SIZE);
		pr_info("rdtags memory reserved: %#x@%#08x\n",
			(unsigned int)CONFIG_RAMDUMP_TAGS_SIZE,
			(unsigned int)rdtags_resources[0].start);
		bank_end -= CONFIG_RAMDUMP_TAGS_SIZE;
	}
#endif
#ifdef CONFIG_RAMDUMP_CRASH_LOGS
	ramdumplog_resources[0].start = bank_end - MSM_RAMDUMP_LOG_SIZE;
	ramdumplog_resources[0].end = bank_end - 1;

	ramdumplog_device.num_resources  = ARRAY_SIZE(ramdumplog_resources);
	ramdumplog_device.resource       = ramdumplog_resources;

	ret = memblock_reserve(ramdumplog_resources[0].start,
			MSM_RAMDUMP_LOG_SIZE);
	if (ret) {
		pr_err("Failed to reserve ramdump crash log memory %#x@%#08x\n",
				(unsigned int)MSM_RAMDUMP_LOG_SIZE,
				(unsigned int)ramdumplog_resources[0].start);
	} else {
		memblock_free(ramdumplog_resources[0].start,
				MSM_RAMDUMP_LOG_SIZE);
		memblock_remove(ramdumplog_resources[0].start,
				MSM_RAMDUMP_LOG_SIZE);
		pr_info("ramdump crash log memory reserved: %#x@%#08x\n",
				(unsigned int)MSM_RAMDUMP_LOG_SIZE,
				(unsigned int)ramdumplog_resources[0].start);
		bank_end -= MSM_RAMDUMP_LOG_SIZE;
	}
#endif
}
#endif

static void __init msm8960_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
	reserve_ion_memory();
	reserve_mdp_memory();
	reserve_rtb_memory();
	reserve_cache_dump_memory();
}

static struct reserve_info msm8960_reserve_info __initdata = {
	.memtype_reserve_table = msm8960_reserve_table,
	.calculate_reserve_sizes = msm8960_calculate_reserve_sizes,
	.reserve_fixed_area = msm8960_reserve_fixed_area,
	.paddr_to_memtype = msm8960_paddr_to_memtype,
};

static int msm8960_memory_bank_size(void)
{
	return 1<<29;
}

static void __init locate_unstable_memory(void)
{
	struct membank *mb = &meminfo.bank[meminfo.nr_banks - 1];
	unsigned long bank_size;
	unsigned long low, high;

	bank_size = msm8960_memory_bank_size();
	msm8960_reserve_info.bank_size = bank_size;

	low = meminfo.bank[0].start;

#if defined(CONFIG_ANDROID_RAM_CONSOLE) || defined(CONFIG_RAMDUMP_TAGS) || \
	defined(CONFIG_RAMDUMP_CRASH_LOGS)
	high = mb->start + (mb->size - DEBUG_MEMORY_SIZE);
#else
	high = mb->start + mb->size;
#endif

	/* Check if 32 bit overflow occured */
	if (high < mb->start)
		high = ~0UL;

	if (high < MAX_FIXED_AREA_SIZE + MSM8960_FIXED_AREA_START)
		panic("fixed area extends beyond end of memory\n");

	low &= ~(bank_size - 1);

	if (high - low <= bank_size)
		goto no_dmm;

#ifdef CONFIG_ENABLE_DMM
	msm8960_reserve_info.low_unstable_address = mb->start -
					MIN_MEMORY_BLOCK_SIZE + mb->size;
	msm8960_reserve_info.max_unstable_size = MIN_MEMORY_BLOCK_SIZE;
	pr_info("low unstable address %lx max size %lx bank size %lx\n",
		msm8960_reserve_info.low_unstable_address,
		msm8960_reserve_info.max_unstable_size,
		msm8960_reserve_info.bank_size);
	return;
#endif
no_dmm:
	msm8960_reserve_info.low_unstable_address = high;
	msm8960_reserve_info.max_unstable_size = 0;
}

static void __init place_movable_zone(void)
{
#ifdef CONFIG_ENABLE_DMM
	movable_reserved_start = msm8960_reserve_info.low_unstable_address;
	movable_reserved_size = msm8960_reserve_info.max_unstable_size;
	pr_info("movable zone start %lx size %lx\n",
		movable_reserved_start, movable_reserved_size);
#endif
}

static void __init msm8960_early_memory(void)
{
	reserve_info = &msm8960_reserve_info;
	locate_unstable_memory();
	place_movable_zone();
}

static char prim_panel_name[PANEL_NAME_MAX_LEN];
static char ext_panel_name[PANEL_NAME_MAX_LEN];
static int __init prim_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(prim_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("prim_display", prim_display_setup);

static int __init ext_display_setup(char *param)
{
	if (strnlen(param, PANEL_NAME_MAX_LEN))
		strlcpy(ext_panel_name, param, PANEL_NAME_MAX_LEN);
	return 0;
}
early_param("ext_display", ext_display_setup);

static void __init msm8960_reserve(void)
{
#if defined(CONFIG_ANDROID_RAM_CONSOLE) || defined(CONFIG_RAMDUMP_TAGS) || \
	defined(CONFIG_RAMDUMP_CRASH_LOGS)
	reserve_debug_memory();
#endif
	msm8960_set_display_params(prim_panel_name, ext_panel_name);
	msm_reserve();
}

static void __init msm8960_allocate_memory_regions(void)
{
	msm8960_allocate_fb_region();
}

#ifdef CONFIG_NFC_PN544
static int pn544_chip_config(enum pn544_state state, void *not_used)
{
	switch (state) {
	case PN544_STATE_OFF:
		gpio_set_value_cansleep(NXP_GPIO_NFC_FWDL_EN, 0);
		gpio_set_value_cansleep(NXP_GPIO_NFC_EN, 0);
		usleep(50000);
		break;
	case PN544_STATE_ON:
		gpio_set_value_cansleep(NXP_GPIO_NFC_FWDL_EN, 0);
		gpio_set_value_cansleep(NXP_GPIO_NFC_EN, 1);
		usleep(10000);
		break;
	case PN544_STATE_FWDL:
		gpio_set_value_cansleep(NXP_GPIO_NFC_FWDL_EN, 1);
		gpio_set_value_cansleep(NXP_GPIO_NFC_EN, 0);
		usleep(10000);
		gpio_set_value_cansleep(NXP_GPIO_NFC_EN, 1);
		break;
	default:
		pr_err("%s: undefined state %d\n", __func__, state);
		return -EINVAL;
	}
	return 0;
}

static int pn544_gpio_request(void)
{
	int ret;

	ret = gpio_request(NXP_GPIO_NFC_IRQ, "pn544_irq");
	if (ret)
		goto err_irq;
	ret = gpio_request(NXP_GPIO_NFC_EN, "pn544_ven");
	if (ret)
		goto err_ven;
	ret = gpio_request(NXP_GPIO_NFC_FWDL_EN, "pn544_fw");
	if (ret)
		goto err_fw;
	return 0;
err_fw:
	gpio_free(NXP_GPIO_NFC_EN);
err_ven:
	gpio_free(NXP_GPIO_NFC_IRQ);
err_irq:
	pr_err("%s: gpio request err %d\n", __func__, ret);
	return ret;
}

static void pn544_gpio_release(void)
{
	gpio_free(NXP_GPIO_NFC_EN);
	gpio_free(NXP_GPIO_NFC_IRQ);
	gpio_free(NXP_GPIO_NFC_FWDL_EN);
}

static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_type = IRQF_TRIGGER_RISING,
	.chip_config = pn544_chip_config,
	.driver_loaded = pn544_gpio_request,
	.driver_unloaded = pn544_gpio_release,
};
#endif

#ifdef CONFIG_WCD9310_CODEC

#define TABLA_INTERRUPT_BASE (NR_MSM_IRQS + NR_GPIO_IRQS + NR_PM8921_IRQS)

/* Micbias setting is based on 8660 CDP/MTP/FLUID requirement
 * 4 micbiases are used to power various analog and digital
 * microphones operating at 1800 mV. Technically, all micbiases
 * can source from single cfilter since all microphones operate
 * at the same voltage level. The arrangement below is to make
 * sure all cfilters are exercised. LDO_H regulator ouput level
 * does not need to be as high as 2.85V. It is choosen for
 * microphone sensitivity purpose.
 */
static struct wcd9xxx_pdata tabla_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x10, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(62),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(34),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 2700,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 2700,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
		.bias1_ext_cap = 0,
		.bias2_ext_cap = 1,
		.bias3_ext_cap = 0,
		.bias4_ext_cap = 0,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device msm_slim_tabla = {
	.name = "tabla-slim",
	.e_addr = {0, 1, 0x10, 0, 0x17, 2},
	.dev = {
		.platform_data = &tabla_platform_data,
	},
};

#ifdef CONFIG_FB_MSM_MHL_SII8334

#define MSM_GPIO_MHL_RESET_N	(64)
#define MSM_GPIO_MHL_IRQ_N	(65)
#define MHL_SEMC_ADOPTER_ID	(0x03A7)
#define MHL_SEMC_DEVICE_ID	(0x0401)

static int mhl_sii_setup_power(int enable)
{
	int rc;
	static struct regulator *reg_8921_l18;

	if (!reg_8921_l18)
		reg_8921_l18 = regulator_get(NULL, "8921_l18");

	if (enable) {
		rc = regulator_set_voltage(reg_8921_l18, 1200000, 1200000);
		if (rc)
			goto out_setup_power;
		rc = regulator_enable(reg_8921_l18);
		if (rc)
			goto out_setup_power;
		printk(KERN_INFO "%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8921_l18);
		if (rc)
			goto out_setup_power;
		printk(KERN_INFO "%s(off): success\n", __func__);
	}

	return 0;
out_setup_power:
	printk(KERN_ERR "%s: failed to setup power\n", __func__);
	regulator_put(reg_8921_l18);
	return rc;
}

static int mhl_sii_low_power_mode(int enable)
{
	int rc;
	static struct regulator *reg_8921_l18;

	if (!reg_8921_l18)
		reg_8921_l18 = regulator_get(NULL, "8921_l18");

	if (enable)
		rc = regulator_set_optimum_mode(reg_8921_l18, 4500);
	else
		rc = regulator_set_optimum_mode(reg_8921_l18, 150000);

	if (rc < 0)
		goto out_low_power_mode;

	return 0;
out_low_power_mode:
	printk(KERN_INFO "%s(%s): failed (%d)\n",
		__func__, enable ? "enable" : "disable", rc);
	return rc;
}

static int mhl_sii_setup_gpio(int enable)
{
	int rc;

	printk(KERN_INFO "mhl_sii_setup_gpio\n");

	rc = gpio_request(MSM_GPIO_MHL_RESET_N, "sii8334_reset");
	if (rc)
		goto out_setup_gpio;

	rc = gpio_request(MSM_GPIO_MHL_IRQ_N, "sii8334_irq");
	if (rc) {
		gpio_free(MSM_GPIO_MHL_RESET_N);
		goto out_setup_gpio;
	}

	return 0;
out_setup_gpio:
	printk(KERN_ERR "%s: failed to setup gpio\n", __func__);
	return rc;
}

static int mhl_sii_charging_enable(int enable, int max_curr)
{
	int ret;
	/* max_curr not supported for blue platform */

	printk(KERN_INFO "mhl_sii_charging_enable (%s)\n",
		enable ? "enable" : "disable");

	if (enable) {
		ret = pm8921_set_usb_power_supply_type
			(POWER_SUPPLY_TYPE_USB);
		if (ret)
			return ret;
		pm8921_charger_vbus_draw(max_curr);
	} else {
		pm8921_charger_vbus_draw(0);
	}

	return 0;
}

static struct mhl_sii_platform_data mhl_sii_pdata = {
	.reset = MSM_GPIO_MHL_RESET_N,
	.hpd_pin_mode = PIN_MODE_TTL,
	.int_pin_mode = PIN_MODE_OPENDRAIN_N,
	.adopter_id = MHL_SEMC_ADOPTER_ID,
	.device_id = MHL_SEMC_DEVICE_ID,
	.setup_power = mhl_sii_setup_power,
	.setup_low_power_mode = mhl_sii_low_power_mode,
	.setup_gpio = mhl_sii_setup_gpio,
	.charging_enable = mhl_sii_charging_enable,
};
#endif /* CONFIG_FB_MSM_MHL_SII8334 */

static struct wcd9xxx_pdata tabla20_platform_data = {
	.slimbus_slave_device = {
		.name = "tabla-slave",
		.e_addr = {0, 0, 0x60, 0, 0x17, 2},
	},
	.irq = MSM_GPIO_TO_INT(62),
	.irq_base = TABLA_INTERRUPT_BASE,
	.num_irqs = NR_WCD9XXX_IRQS,
	.reset_gpio = PM8921_GPIO_PM_TO_SYS(34),
	.micbias = {
		.ldoh_v = TABLA_LDOH_2P85_V,
		.cfilt1_mv = 1800,
		.cfilt2_mv = 2700,
		.cfilt3_mv = 1800,
		.bias1_cfilt_sel = TABLA_CFILT1_SEL,
		.bias2_cfilt_sel = TABLA_CFILT2_SEL,
		.bias3_cfilt_sel = TABLA_CFILT3_SEL,
		.bias4_cfilt_sel = TABLA_CFILT3_SEL,
	},
	.regulator = {
	{
		.name = "CDC_VDD_CP",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_CP_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_RX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_RX_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_TX",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_CDC_VDDA_TX_CUR_MAX,
	},
	{
		.name = "VDDIO_CDC",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.optimum_uA = WCD9XXX_VDDIO_CDC_CUR_MAX,
	},
	{
		.name = "VDDD_CDC_D",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_D_CUR_MAX,
	},
	{
		.name = "CDC_VDDA_A_1P2V",
		.min_uV = 1225000,
		.max_uV = 1250000,
		.optimum_uA = WCD9XXX_VDDD_CDC_A_CUR_MAX,
	},
	},
};

static struct slim_device msm_slim_tabla20 = {
	.name = "tabla2x-slim",
	.e_addr = {0, 1, 0x60, 0, 0x17, 2},
	.dev = {
		.platform_data = &tabla20_platform_data,
	},
};
#endif

static struct slim_boardinfo msm_slim_devices[] = {
#ifdef CONFIG_WCD9310_CODEC
	{
		.bus_num = 1,
		.slim_slave = &msm_slim_tabla,
	},
	{
		.bus_num = 1,
		.slim_slave = &msm_slim_tabla20,
	},
#endif
	/* add more slimbus slaves as needed */
};

#define MSM_WCNSS_PHYS	0x03000000
#define MSM_WCNSS_SIZE	0x280000

static struct resource resources_wcnss_wlan[] = {
	{
		.start	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.end	= RIVA_APPS_WLAN_RX_DATA_AVAIL_IRQ,
		.name	= "wcnss_wlanrx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.end	= RIVA_APPS_WLAN_DATA_XFER_DONE_IRQ,
		.name	= "wcnss_wlantx_irq",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_WCNSS_PHYS,
		.end	= MSM_WCNSS_PHYS + MSM_WCNSS_SIZE - 1,
		.name	= "wcnss_mmio",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 84,
		.end	= 88,
		.name	= "wcnss_gpios_5wire",
		.flags	= IORESOURCE_IO,
	},
};

static struct qcom_wcnss_opts qcom_wcnss_pdata = {
	.has_48mhz_xo	= 1,
};

static struct platform_device msm_device_wcnss_wlan = {
	.name		= "wcnss_wlan",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_wcnss_wlan),
	.resource	= resources_wcnss_wlan,
	.dev		= {.platform_data = &qcom_wcnss_pdata},
};

#ifdef CONFIG_QSEECOM
/* qseecom bus scaling */
static struct msm_bus_vectors qseecom_clks_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) *  100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) * 100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = 0,
		.ab = 0,
	},
};

static struct msm_bus_vectors qseecom_enable_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = 0,
		.ab = 0,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_vectors qseecom_enable_dfab_sfpb_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) *  100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_SPS,
		.ib = (492 * 8) * 1000000UL,
		.ab = (492 * 8) * 100000UL,
	},
	{
		.src = MSM_BUS_MASTER_SPDM,
		.dst = MSM_BUS_SLAVE_SPDM,
		.ib = (64 * 8) * 1000000UL,
		.ab = (64 * 8) *  100000UL,
	},
};

static struct msm_bus_paths qseecom_hw_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(qseecom_clks_init_vectors),
		qseecom_clks_init_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_vectors),
		qseecom_enable_dfab_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_sfpb_vectors),
		qseecom_enable_sfpb_vectors,
	},
	{
		ARRAY_SIZE(qseecom_enable_dfab_sfpb_vectors),
		qseecom_enable_dfab_sfpb_vectors,
	},
};

static struct msm_bus_scale_pdata qseecom_bus_pdata = {
	qseecom_hw_bus_scale_usecases,
	ARRAY_SIZE(qseecom_hw_bus_scale_usecases),
	.name = "qsee",
};

static struct platform_device qseecom_device = {
	.name		= "qseecom",
	.id		= 0,
	.dev		= {
		.platform_data = &qseecom_bus_pdata,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0x18500000

#define QCE_HW_KEY_SUPPORT	0
#define QCE_SHA_HMAC_SUPPORT	1
#define QCE_SHARE_CE_RESOURCE	1
#define QCE_CE_SHARED		0

/* Begin Bus scaling definitions */
static struct msm_bus_vectors crypto_hw_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors crypto_hw_active_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ADM_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 70000000UL,
		.ib = 70000000UL,
	},
	{
		.src = MSM_BUS_MASTER_ADM_PORT1,
		.dst = MSM_BUS_SLAVE_GSBI1_UART,
		.ab = 2480000000UL,
		.ib = 2480000000UL,
	},
};

static struct msm_bus_paths crypto_hw_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(crypto_hw_init_vectors),
		crypto_hw_init_vectors,
	},
	{
		ARRAY_SIZE(crypto_hw_active_vectors),
		crypto_hw_active_vectors,
	},
};

static struct msm_bus_scale_pdata crypto_hw_bus_scale_pdata = {
		crypto_hw_bus_scale_usecases,
		ARRAY_SIZE(crypto_hw_bus_scale_usecases),
		.name = "cryptohw",
};
/* End Bus Scaling Definitions*/

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = &crypto_hw_bus_scale_pdata,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	.bus_scale_table = &crypto_hw_bus_scale_pdata,
};

static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

static struct mdm_platform_data sglte_platform_data = {
	.mdm_version = "4.0",
	.ramdump_delay_ms = 1000,
	.soft_reset_inverted = 1,
	.peripheral_platform_device = NULL,
	.ramdump_timeout_ms = 600000,
	.no_powerdown_after_ramdumps = 1,
	.image_upgrade_supported = 1,
};

#define MSM_TSIF0_PHYS			(0x18200000)
#define MSM_TSIF1_PHYS			(0x18201000)
#define MSM_TSIF_SIZE			(0x200)
#define MSM_TSPP_PHYS			(0x18202000)
#define MSM_TSPP_SIZE			(0x1000)
#define MSM_TSPP_BAM_PHYS		(0x18204000)
#define MSM_TSPP_BAM_SIZE		(0x2000)

#define TSIF_0_CLK       GPIO_CFG(75, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_EN        GPIO_CFG(76, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_DATA      GPIO_CFG(77, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_SYNC      GPIO_CFG(82, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_CLK       GPIO_CFG(79, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_EN        GPIO_CFG(80, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_DATA      GPIO_CFG(81, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_SYNC      GPIO_CFG(78, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_0_CLK,  .label =  "tsif0_clk", },
	{ .gpio_cfg = TSIF_0_EN,   .label =  "tsif0_en", },
	{ .gpio_cfg = TSIF_0_DATA, .label =  "tsif0_data", },
	{ .gpio_cfg = TSIF_0_SYNC, .label =  "tsif0_sync", },
	{ .gpio_cfg = TSIF_1_CLK,  .label =  "tsif1_clk", },
	{ .gpio_cfg = TSIF_1_EN,   .label =  "tsif1_en", },
	{ .gpio_cfg = TSIF_1_DATA, .label =  "tsif1_data", },
	{ .gpio_cfg = TSIF_1_SYNC, .label =  "tsif1_sync", },
};

static struct resource tspp_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = TSIF_TSPP_IRQ,
		.end   = TSIF1_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF0_PHYS,
		.end   = MSM_TSIF0_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF1_PHYS,
		.end   = MSM_TSIF1_PHYS + MSM_TSIF_SIZE - 1,
	},
	[3] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSPP_PHYS,
		.end   = MSM_TSPP_PHYS + MSM_TSPP_SIZE - 1,
	},
	[4] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSPP_BAM_PHYS,
		.end   = MSM_TSPP_BAM_PHYS + MSM_TSPP_BAM_SIZE - 1,
	},
};

static struct msm_tspp_platform_data tspp_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};

static struct platform_device msm_device_tspp = {
	.name          = "msm_tspp",
	.id            = 0,
	.num_resources = ARRAY_SIZE(tspp_resources),
	.resource      = tspp_resources,
	.dev = {
		.platform_data = &tspp_platform_data
	},
};

#define MSM_SHARED_RAM_PHYS 0x80000000

static void __init msm8960_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8960_io();

	if (socinfo_init() < 0)
		pr_err("socinfo_init() failed!\n");
}

static void __init msm8960_init_irq(void)
{
	struct msm_mpm_device_data *data = NULL;

#ifdef CONFIG_MSM_MPM
	data = &msm8960_mpm_dev_data;
#endif

	msm_mpm_irq_extn_init(data);
	gic_init(0, GIC_PPI_START, MSM_QGIC_DIST_BASE,
						(void *)MSM_QGIC_CPU_BASE);
}

static void __init msm8960_init_buses(void)
{
#ifdef CONFIG_MSM_BUS_SCALING
	msm_bus_rpm_set_mt_mask();
	msm_bus_8960_apps_fabric_pdata.rpm_enabled = 1;
	msm_bus_8960_sys_fabric_pdata.rpm_enabled = 1;
	msm_bus_apps_fabric.dev.platform_data =
		&msm_bus_8960_apps_fabric_pdata;
	msm_bus_sys_fabric.dev.platform_data = &msm_bus_8960_sys_fabric_pdata;
	if (cpu_is_msm8960ab()) {
		msm_bus_8960_sg_mm_fabric_pdata.rpm_enabled = 1;
		msm_bus_mm_fabric.dev.platform_data =
			&msm_bus_8960_sg_mm_fabric_pdata;
	} else {
		msm_bus_8960_mm_fabric_pdata.rpm_enabled = 1;
		msm_bus_mm_fabric.dev.platform_data =
			&msm_bus_8960_mm_fabric_pdata;
	}
	msm_bus_sys_fpb.dev.platform_data = &msm_bus_8960_sys_fpb_pdata;
	msm_bus_cpss_fpb.dev.platform_data = &msm_bus_8960_cpss_fpb_pdata;
#endif
}

#ifdef CONFIG_USB_MSM_OTG_72K
static struct msm_otg_platform_data msm_otg_pdata;
#else
static int msm_hsusb_vbus_power(bool on)
{
	int ret = 0;
#ifdef CONFIG_USB_NCP373
	if (on) {
		ret = pm8921_disable_source_current(1);
		if (unlikely(ret < 0)) {
			pr_err("%s: failed to notify boost the VBUS ret=%d\n",
								__func__, ret);
			goto do_vbus_off;
		}
		ret = ncp373_vbus_switch(1);
		if (unlikely(ret < 0)) {
			pr_err("%s: failed to switch the vbus load ret=%d\n",
								__func__, ret);
			goto do_vbus_off;
		}
		return ret;
	}
do_vbus_off:
	ncp373_vbus_switch(0);
	pm8921_disable_source_current(0);
#endif
	return ret;
}

#ifdef CONFIG_MSM_BUS_SCALING
/* Bandwidth requests (zero) if no vote placed */
static struct msm_bus_vectors usb_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

/* Bus bandwidth requests in Bytes/sec */
static struct msm_bus_vectors usb_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_SPS,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 60000000,		/* At least 480Mbps on bus. */
		.ib = 960000000,	/* MAX bursts rate */
	},
};

static struct msm_bus_paths usb_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(usb_init_vectors),
		usb_init_vectors,
	},
	{
		ARRAY_SIZE(usb_max_vectors),
		usb_max_vectors,
	},
};

static struct msm_bus_scale_pdata usb_bus_scale_pdata = {
	usb_bus_scale_usecases,
	ARRAY_SIZE(usb_bus_scale_usecases),
	.name = "usb",
};
#endif

#define MSM_MPM_PIN_USB1_OTGSESSVLD	40

static struct msm_otg_platform_data msm_otg_pdata = {
	.mode			= USB_OTG,
	.otg_control		= OTG_PMIC_CONTROL,
	.phy_type		= SNPS_28NM_INTEGRATED_PHY,
	.pmic_id_irq		= PM8921_USB_ID_IN_IRQ(PM8921_IRQ_BASE),
	.vbus_power		= msm_hsusb_vbus_power,
	.power_budget		= CONFIG_USB_MSM_OTG_POWER_BUDGET,
	.phy_init_seq	= somc_usb_phy_init_seq,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table	= &usb_bus_scale_pdata,
	.mpm_otgsessvld_int	= MSM_MPM_PIN_USB1_OTGSESSVLD,
#endif
#ifdef CONFIG_FB_MSM_HDMI_MHL_8334
	.mhl_dev_name		= "sii8334",
#endif
#ifdef CONFIG_FB_MSM_MHL_SII8334
	.mhl_dev_name		= SII_DEV_NAME,
#endif
};
#endif

#ifdef CONFIG_USB_NCP373
#define GPIO_USB_OTG_EN		51
#define GPIO_OTG_OVRCUR_DET_N	37
#define GPIO_OTG_OVP_CNTL	PM8921_GPIO_PM_TO_SYS(42)

struct ncp373_res_hdl {
	int en;
	int in;
	int flg;
};
static struct ncp373_res_hdl ncp373_hdl;

static int ncp373_gpio_request(int gpio, char *name)
{
	int ret;

	ret = gpio_request(gpio, name);

	if (unlikely(ret < 0)) {
		pr_err("%s: failed to request gpio=%d name=%s\n", __func__,
								gpio, name);
		ret = 0;
	} else {
		pr_err("%s: got gpio gpio=%d name=%s\n", __func__,
								gpio, name);
		ret = gpio;
	}

	return ret;
}

static int ncp373_en_request(void)
{
	return ncp373_gpio_request(GPIO_OTG_OVP_CNTL, "ncp373_en");
}

static int ncp373_in_request(void)
{
	return ncp373_gpio_request(GPIO_USB_OTG_EN, "ncp373_in");
}

static int ncp373_flg_request(void)
{
	return ncp373_gpio_request(GPIO_OTG_OVRCUR_DET_N, "ncp373_flg");
}

static int ncp373_probe(struct platform_device *pdev)
{
	/* It may not be got a resource here,
	 * due to the timeliness of the device initialization.
	*/
	if (likely(!ncp373_hdl.en))
		ncp373_hdl.en = ncp373_en_request();

	if (likely(!ncp373_hdl.in))
		ncp373_hdl.in = ncp373_in_request();

	if (likely(!ncp373_hdl.flg))
		ncp373_hdl.flg = ncp373_flg_request();

	return 0;
}

static int ncp373_en_set(int on)
{
	int ret = -EIO;
	struct pm_gpio param = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.vin_sel	= PM_GPIO_VIN_VPH,
		.out_strength	= PM_GPIO_STRENGTH_LOW,
		.function	= PM_GPIO_FUNC_NORMAL,
	};

	if (unlikely(!ncp373_hdl.en))
		ncp373_hdl.en = ncp373_en_request();

	if (likely(ncp373_hdl.en)) {
		param.output_value = !on;
		ret = pm8xxx_gpio_config(ncp373_hdl.en, &param);

		if (unlikely(ret < 0))
			pr_err("%s: failed to switch %s gpio %d ret=%d\n",
						__func__, on ? "on " : "off",
						ncp373_hdl.en, ret);
	}

	return ret;
}

static int ncp373_in_set(int on)
{
	int ret = -EIO;

	if (unlikely(!ncp373_hdl.in))
		ncp373_hdl.in = ncp373_in_request();

	if (likely(ncp373_hdl.in)) {
		gpio_set_value(ncp373_hdl.in, !!on);
		ret = 0;
	}

	return ret;
}

static int ncp373_flg_get(void)
{
	int ret = -EIO;

	if (unlikely(!ncp373_hdl.flg))
		ncp373_hdl.flg = ncp373_flg_request();

	if (likely(ncp373_hdl.flg)) {
		ret = gpio_get_value(ncp373_hdl.flg);
		if (unlikely(ret < 0))
			pr_err("%s: failed to read GPIO=%d ret=%d\n",
						__func__, ncp373_hdl.flg, ret);
	}

	return ret;
}

static void ncp373_remove(void)
{
	if (likely(ncp373_hdl.en))
		gpio_free(ncp373_hdl.en);

	if (likely(ncp373_hdl.in))
		gpio_free(ncp373_hdl.in);

	if (likely(ncp373_hdl.flg))
		gpio_free(ncp373_hdl.flg);

	ncp373_hdl.en = 0;
	ncp373_hdl.in = 0;
	ncp373_hdl.flg = 0;
}

static void ncp373_notify_flg_int(void)
{
	pr_info("%s: received over current notify\n", __func__);
	msm_otg_notify_vbus_drop();
}

static void ncp373_check_pin_state(void)
{
	int en = -1;
	int in = -1;
	int flg = -1;

	if (likely(ncp373_hdl.en))
		en = gpio_get_value(ncp373_hdl.en);

	if (likely(ncp373_hdl.in))
		in = gpio_get_value(ncp373_hdl.in);

	if (likely(ncp373_hdl.flg))
		flg = gpio_get_value(ncp373_hdl.flg);

	pr_debug("%s: EN=%d, IN=%d, FLG=%d\n", __func__, en, in, flg);
}

struct ncp373_platform_data ncp373_pdata = {
	.probe		= ncp373_probe,
	.remove		= ncp373_remove,
	.en_set		= ncp373_en_set,
	.in_set		= ncp373_in_set,
	.flg_get	= ncp373_flg_get,
	.notify_flg_int	= ncp373_notify_flg_int,
	.check_pin_state = ncp373_check_pin_state,
	.oc_delay_time	= 300000,
};

static struct resource ncp373_resources[] = {
	/* OTG_OVERCUR_INT */
	{
		.start	= MSM_GPIO_TO_INT(GPIO_OTG_OVRCUR_DET_N),
		.end	= MSM_GPIO_TO_INT(GPIO_OTG_OVRCUR_DET_N),
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device ncp373_device = {
	.name		= NCP373_DRIVER_NAME,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ncp373_resources),
	.resource	= ncp373_resources,
	.dev		= {
		.platform_data = &ncp373_pdata,
	},
};
#endif

#ifdef CONFIG_USB_EHCI_MSM_HSIC
#define HSIC_HUB_RESET_GPIO	91
static struct msm_hsic_host_platform_data msm_hsic_pdata = {
	.strobe		= 150,
	.data		= 151,
};

static struct smsc_hub_platform_data hsic_hub_pdata = {
	.hub_reset		= HSIC_HUB_RESET_GPIO,
};
#else
static struct msm_hsic_host_platform_data msm_hsic_pdata;
static struct smsc_hub_platform_data hsic_hub_pdata;
#endif

static struct platform_device smsc_hub_device = {
	.name	= "msm_smsc_hub",
	.id	= -1,
	.dev	= {
		.platform_data = &hsic_hub_pdata,
	},
};

#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A03F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum) {
		memset(dload->serial_number, 0, SERIAL_NUMBER_LENGTH);
		goto out;
	}

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strlcpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
out:
	iounmap(dload);
	return 0;
}

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
	.can_stall = 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static uint8_t spm_wfi_cmd_sequence[] __initdata = {
			0x03, 0x0f,
};

static uint8_t spm_retention_cmd_sequence[] __initdata = {
			0x00, 0x05, 0x03, 0x0D,
			0x0B, 0x00, 0x0f,
};

static uint8_t spm_retention_with_krait_v3_cmd_sequence[] __initdata = {
	0x42, 0x1B, 0x00,
	0x05, 0x03, 0x0D, 0x0B,
	0x00, 0x42, 0x1B,
	0x0f,
};

static uint8_t spm_power_collapse_without_rpm[] __initdata = {
			0x00, 0x24, 0x54, 0x10,
			0x09, 0x03, 0x01,
			0x10, 0x54, 0x30, 0x0C,
			0x24, 0x30, 0x0f,
};

static uint8_t spm_power_collapse_with_rpm[] __initdata = {
			0x00, 0x24, 0x54, 0x10,
			0x09, 0x07, 0x01, 0x0B,
			0x10, 0x54, 0x30, 0x0C,
			0x24, 0x30, 0x0f,
};

/* 8960AB has a different command to assert apc_pdn */
static uint8_t spm_power_collapse_without_rpm_krait_v3[] __initdata = {
	0x00, 0x30, 0x24, 0x30,
	0x84, 0x10, 0x09, 0x03,
	0x01, 0x10, 0x84, 0x30,
	0x0C, 0x24, 0x30, 0x0f,
};

static uint8_t spm_power_collapse_with_rpm_krait_v3[] __initdata = {
	0x00, 0x30, 0x24, 0x30,
	0x84, 0x10, 0x09, 0x07,
	0x01, 0x0B, 0x10, 0x84,
	0x30, 0x0C, 0x24, 0x30,
	0x0f,
};

static struct msm_spm_seq_entry msm_spm_boot_cpu_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},

	[1] = {
		.mode = MSM_SPM_MODE_POWER_RETENTION,
		.notify_rpm = false,
		.cmd = spm_retention_cmd_sequence,
	},

	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},
	[3] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};

static struct msm_spm_seq_entry msm_spm_nonboot_cpu_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_MODE_CLOCK_GATING,
		.notify_rpm = false,
		.cmd = spm_wfi_cmd_sequence,
	},

	[1] = {
		.mode = MSM_SPM_MODE_POWER_RETENTION,
		.notify_rpm = false,
		.cmd = spm_retention_cmd_sequence,
	},

	[2] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = false,
		.cmd = spm_power_collapse_without_rpm,
	},

	[3] = {
		.mode = MSM_SPM_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = spm_power_collapse_with_rpm,
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x58589464,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00020000,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_boot_cpu_seq_list),
		.modes = msm_spm_boot_cpu_seq_list,
	},
	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_CFG] = 0x1F,
#if defined(CONFIG_MSM_AVS_HW)
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_CTL] = 0x58589464,
		.reg_init_values[MSM_SPM_REG_SAW2_AVS_HYSTERESIS] = 0x00020000,
#endif
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x03020004,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x0084009C,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A4001C,
		.vctl_timeout_us = 50,
		.num_modes = ARRAY_SIZE(msm_spm_nonboot_cpu_seq_list),
		.modes = msm_spm_nonboot_cpu_seq_list,
	},
};

static uint8_t l2_spm_wfi_cmd_sequence[] __initdata = {
			0x00, 0x20, 0x03, 0x20,
			0x00, 0x0f,
};

static uint8_t l2_spm_gdhs_cmd_sequence[] __initdata = {
			0x00, 0x20, 0x34, 0x64,
			0x48, 0x07, 0x48, 0x20,
			0x50, 0x64, 0x04, 0x34,
			0x50, 0x0f,
};
static uint8_t l2_spm_power_off_cmd_sequence[] __initdata = {
			0x00, 0x10, 0x34, 0x64,
			0x48, 0x07, 0x48, 0x10,
			0x50, 0x64, 0x04, 0x34,
			0x50, 0x0F,
};

static struct msm_spm_seq_entry msm_spm_l2_seq_list[] __initdata = {
	[0] = {
		.mode = MSM_SPM_L2_MODE_RETENTION,
		.notify_rpm = false,
		.cmd = l2_spm_wfi_cmd_sequence,
	},
	[1] = {
		.mode = MSM_SPM_L2_MODE_GDHS,
		.notify_rpm = true,
		.cmd = l2_spm_gdhs_cmd_sequence,
	},
	[2] = {
		.mode = MSM_SPM_L2_MODE_POWER_COLLAPSE,
		.notify_rpm = true,
		.cmd = l2_spm_power_off_cmd_sequence,
	},
};

static struct msm_spm_platform_data msm_spm_l2_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW_L2_BASE,
		.reg_init_values[MSM_SPM_REG_SAW2_SPM_CTL] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DLY] = 0x02020204,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_0] = 0x00A000AE,
		.reg_init_values[MSM_SPM_REG_SAW2_PMIC_DATA_1] = 0x00A00020,
		.modes = msm_spm_l2_seq_list,
		.num_modes = ARRAY_SIZE(msm_spm_l2_seq_list),
	},
};

#ifdef CONFIG_LEDS_AS3676
extern struct as3676_platform_data as3676_platform_data;
#endif

#define SYNAPTICS_TOUCH_GPIO_IRQ 11

#ifdef CONFIG_TOUCHSCREEN_CLEARPAD
static struct regulator *vreg_touch_vdd;

struct synaptics_pointer_data pointer_data_0x19 = {
	.offset_x = 0,
	.offset_y = 49,
};

struct synaptics_pointer_data pointer_data_0x1A = {
	.offset_x = 0,
	.offset_y = 55,
};


struct synaptics_funcarea clearpad_funcarea_array[] = {
	{
		{ 0, 0, 719, 1279 }, { 0, 0, 719, 1279 },
		SYN_FUNCAREA_POINTER, NULL
	},
	{ .func = SYN_FUNCAREA_END }
};

struct synaptics_funcarea clearpad_funcarea_array_0x19[] = {
	{
		{ 0, 0, 719, 36 }, { 0, 0, 719, 36 },
		SYN_FUNCAREA_INSENSIBLE, NULL
	},
	{
		{ 0, 49, 719, 1328 }, { 0, 37, 719, 1332 },
		SYN_FUNCAREA_POINTER, &pointer_data_0x19
	},
	{ .func = SYN_FUNCAREA_END }
};

struct synaptics_funcarea clearpad_funcarea_array_0x1A[] = {
	{
		{ 0, 0, 719, 42 }, { 0, 0, 719, 42 },
		SYN_FUNCAREA_INSENSIBLE, NULL
	},
	{
		{ 0, 55, 719, 1334 }, { 0, 43, 719, 1336 },
		SYN_FUNCAREA_POINTER, &pointer_data_0x1A
	},
	{ .func = SYN_FUNCAREA_END }
};

struct synaptics_funcarea *clearpad_funcarea_get(u8 module_id, u8 rev)
{
	struct synaptics_funcarea *funcarea = NULL;

	pr_info("%s: module_id=0x%02x rev=0x%02x\n", __func__, module_id, rev);
	switch (module_id) {
	case 0x19:
		funcarea = clearpad_funcarea_array_0x19;
		break;
	case 0x1A:
	case 0xFF:
		funcarea = clearpad_funcarea_array_0x1A;
		break;
	default:
		funcarea = clearpad_funcarea_array;
		break;
	}

	return funcarea;
}

static int clearpad_vreg_low_power_mode(int enable)
{
	int rc = 0;

	if (IS_ERR(vreg_touch_vdd)) {
		pr_err("%s: vreg_touch_vdd is not initialized\n", __func__);
		return -ENODEV;
	}

	if (enable)
		rc = regulator_set_optimum_mode(vreg_touch_vdd, 1000);
	else
		rc = regulator_set_optimum_mode(vreg_touch_vdd, 15000);

	if (rc < 0) {
		pr_err("%s: vdd: set mode (%s) failed, rc=%d\n",
			__func__, (enable ? "LPM" : "HPM"), rc);
		return rc;
	} else {
		pr_debug("%s: vdd: set mode (%s) ok, new mode=%d\n",
				__func__, (enable ? "LPM" : "HPM"), rc);
		return 0;
	}
}

static int clearpad_vreg_configure(int enable)
{
	int rc = 0;

	vreg_touch_vdd = regulator_get(NULL, "8921_l17");
	if (IS_ERR(vreg_touch_vdd)) {
		pr_err("%s: get vdd failed\n", __func__);
		return -ENODEV;
	}

	if (enable) {
		rc = regulator_set_voltage(vreg_touch_vdd, 3000000, 3000000);
		if (rc) {
			pr_err("%s: set voltage failed, rc=%d\n", __func__, rc);
			goto clearpad_vreg_configure_err;
		}
		rc = regulator_enable(vreg_touch_vdd);
		if (rc) {
			pr_err("%s: enable vdd failed, rc=%d\n", __func__, rc);
			goto clearpad_vreg_configure_err;
		}
		rc = clearpad_vreg_low_power_mode(0);
		if (rc) {
			pr_err("%s: set vdd mode failed, rc=%d\n",
				__func__, rc);
			goto clearpad_vreg_configure_err;
		}
	} else {
		rc = regulator_set_voltage(vreg_touch_vdd, 0, 3000000);
		if (rc) {
			pr_err("%s: set voltage failed, rc=%d\n", __func__, rc);
			goto clearpad_vreg_configure_err;
		}
		rc = regulator_disable(vreg_touch_vdd);
		if (rc)
			pr_err("%s: disable vdd failed, rc=%d\n",
								__func__, rc);
	}
	return rc;
clearpad_vreg_configure_err:
	regulator_put(vreg_touch_vdd);
	return rc;
}

static int clearpad_vreg_reset(void)
{
	int rc = 0;

	rc = clearpad_vreg_configure(0);
	if (rc)
		return rc;
	usleep(20000);
	rc = clearpad_vreg_configure(1);
	return rc;
}

static int clearpad_gpio_configure(int enable)
{
	int rc = 0;

	if (enable) {
		rc = gpio_request(SYNAPTICS_TOUCH_GPIO_IRQ, CLEARPAD_NAME);
		if (rc)
			pr_err("%s: gpio_requeset failed, "
					"rc=%d\n", __func__, rc);
	} else {
		gpio_free(SYNAPTICS_TOUCH_GPIO_IRQ);
	}
	return rc;
}

static int clearpad_gpio_export(struct device *dev, bool export)
{
	int rc = 0;

	if (!dev)
		return -ENODEV;

	if (export) {
		rc = gpio_export(SYNAPTICS_TOUCH_GPIO_IRQ, false);
		if (rc) {
			pr_err("%s: Failed to export gpio, rc=%d\n",
				__func__, rc);
		} else {
			rc = gpio_export_link(dev, "attn",
					SYNAPTICS_TOUCH_GPIO_IRQ);
			if (rc)
				pr_err("%s: Failed to symlink gpio, "
					"rc=%d\n", __func__, rc);
		}
	} else {
		gpio_unexport(SYNAPTICS_TOUCH_GPIO_IRQ);
		sysfs_remove_link(&dev->kobj, "attn");
	}

	return rc;
}

static struct clearpad_platform_data clearpad_platform_data = {
	.irq = MSM_GPIO_TO_INT(SYNAPTICS_TOUCH_GPIO_IRQ),
	.funcarea_get = clearpad_funcarea_get,
	.vreg_configure = clearpad_vreg_configure,
	.vreg_suspend = clearpad_vreg_low_power_mode,
	.vreg_reset = clearpad_vreg_reset,
	.gpio_configure = clearpad_gpio_configure,
	.gpio_export = clearpad_gpio_export,
};
#endif

#ifdef CONFIG_RMI4_BUS
#define RMI4_VDD "touch_vdd"
#define RMI4_VIO "touch_vio"
static struct regulator *vreg_touch_vdd;
static struct regulator *vreg_touch_vio;


static int rmi4_vreg_config(struct device *dev, int enable)
{
	int rc;

	dev_dbg(dev, "%s - Called (enable=%d)\n", __func__, enable);

	if (enable) {
		if (vreg_touch_vdd)
			return 0;
		vreg_touch_vdd = regulator_get(dev, RMI4_VDD);
		if (IS_ERR(vreg_touch_vdd)) {
			dev_err(dev, "get '%s' failed\n", RMI4_VDD);
			rc = -ENODEV;
			goto no_vdd;
		}
		rc = regulator_set_voltage(vreg_touch_vdd, 3000000, 3000000);
		if (rc) {
			dev_err(dev, "vdd set voltage failed, rc=%d\n", rc);
			goto vdd_config_err;
		}
		rc = regulator_enable(vreg_touch_vdd);
		if (rc) {
			dev_err(dev, "vdd enable vdd failed, rc=%d\n", rc);
			goto vdd_config_err;
		}
		vreg_touch_vio = regulator_get(dev, RMI4_VIO);
		if (IS_ERR(vreg_touch_vio)) {
			dev_err(dev, "get '%s' failed\n", RMI4_VIO);
			rc = -ENODEV;
			goto no_vio;
		}
		rc = regulator_set_voltage(vreg_touch_vio, 1800000, 1800000);
		if (rc) {
			dev_err(dev, "vio set voltage failed, rc=%d\n", rc);
			goto vio_config_err;
		}
		rc = regulator_enable(vreg_touch_vio);
		if (rc) {
			dev_err(dev, "vio enable vdd failed, rc=%d\n", rc);
			goto vio_config_err;
		}
		return -EAGAIN;
	} else {
		if (!IS_ERR(vreg_touch_vdd)) {
			regulator_disable(vreg_touch_vdd);
			regulator_put(vreg_touch_vdd);
			vreg_touch_vdd = NULL;
		}
		if (!IS_ERR(vreg_touch_vio)) {
			regulator_disable(vreg_touch_vio);
			regulator_put(vreg_touch_vio);
			vreg_touch_vio = NULL;
		}
	}
	return 0;

vio_config_err:
	regulator_put(vreg_touch_vio);
	vreg_touch_vio = NULL;
no_vio:
	regulator_disable(vreg_touch_vdd);
vdd_config_err:
	regulator_put(vreg_touch_vdd);
	vreg_touch_vdd = NULL;
no_vdd:
	return rc;
}

static int rmi4_hard_reset(struct device *dev)
{
	int rc;

	dev_info(dev, "%s\n", __func__);
	if (IS_ERR_OR_NULL(vreg_touch_vdd) || IS_ERR_OR_NULL(vreg_touch_vio)) {
		dev_info(dev, "%s: no vregs are present\n", __func__);
		return -ENODEV;
	}
	regulator_disable(vreg_touch_vio);
	regulator_disable(vreg_touch_vdd);
	usleep(1000);
	rc = regulator_enable(vreg_touch_vdd);
	rc = rc ? rc : regulator_enable(vreg_touch_vio);
	return rc;
}

static struct rmi4_wd_platform_data rmi4_wd_data = {
	.reset_func = rmi4_hard_reset,
	.poll_t_ms = 1000,
};

static struct rmi4_function_data fdata[] = {
	{
		.func_name = RMI4_WATCHDOG_NAME,
		.func_id = 0x11,
		.func_data = &rmi4_wd_data,
	},
#ifdef CONFIG_HAS_EARLYSUSPEND
	{
		.func_name = RMI4_E_SUSP_NAME,
		.func_id = 0x01,
		.func_data = NULL,
	},
#endif

};

static struct rmi4_core_device_data rmi4_core_data = {
	.core_name = RMI4_CORE_DRIVER_NAME,
	.attn_gpio = SYNAPTICS_TOUCH_GPIO_IRQ,
	.irq_polarity = IRQF_TRIGGER_FALLING,
	.irq_is_shared = false,
	.num_functions = ARRAY_SIZE(fdata),
	.func_data = fdata,
};

static struct rmi4_i2c_adapter_platform_data rmi4_platform_data = {
	.irq = MSM_GPIO_TO_INT(SYNAPTICS_TOUCH_GPIO_IRQ),
	.vreg_config = rmi4_vreg_config,
	.cdev_data = &rmi4_core_data,
};
#endif

#ifdef CONFIG_LM3560_FLASHLED
#define LM3560_HW_RESET_GPIO 2
#define NTC_VREG_ID "lm3560_ntc"
static struct regulator *lm3560_ntc_vreg;

static int lm356x_pwr(struct device *dev, bool request)
{
	int rc = 0;

	dev_dbg(dev, "%s: request %d\n", __func__, request);

	if (IS_ERR_OR_NULL(lm3560_ntc_vreg))
		return -ENODEV;

	if (request) {
		rc = regulator_enable(lm3560_ntc_vreg);
		if (rc)
			dev_err(dev, "failed to enable vreg '%s'\n",
					NTC_VREG_ID);
		else {
			gpio_set_value(LM3560_HW_RESET_GPIO, 1);
			udelay(20);
		}
	} else {
		gpio_set_value(LM3560_HW_RESET_GPIO, 0);
		rc = regulator_disable(lm3560_ntc_vreg);
	}
	return rc;
}


static int lm356x_platform_init(struct device *dev, bool request)
{
	int rc ;

	if (request) {
		rc = gpio_request(LM3560_HW_RESET_GPIO, "LM3560 hw reset");
		if (rc)
			goto err;

		rc = gpio_direction_output(LM3560_HW_RESET_GPIO, 0);
		if (rc) {
			dev_err(dev, "failed to set direction rc %d\n", rc);
			gpio_free(LM3560_HW_RESET_GPIO);
			return -EIO;
		}

		lm3560_ntc_vreg = regulator_get(dev, NTC_VREG_ID);
		if (IS_ERR_OR_NULL(lm3560_ntc_vreg)) {
			dev_err(dev, "failed to get vreg '%s'\n", NTC_VREG_ID);
			rc = -ENODEV;
			goto free_gpio;
		}
	} else {
		rc = 0;
		if (!IS_ERR_OR_NULL(lm3560_ntc_vreg))
			regulator_put(lm3560_ntc_vreg);
		lm3560_ntc_vreg = NULL;
free_gpio:
		gpio_free(LM3560_HW_RESET_GPIO);
	}
err:
	if (rc)
		dev_err(dev, "%s: failed rc %d\n", __func__, rc);
	return rc;
}

static struct lm3560_platform_data lm3560_platform_data = {
	.power			= lm356x_pwr,
	.platform_init          = lm356x_platform_init,
	.led_nums		= 2,
	.strobe_trigger		= LM3560_STROBE_TRIGGER_EDGE,
	.privacy_terminate	= LM3560_PRIVACY_MODE_TURN_BACK,
	.privacy_led_nums	= 1,
	.privacy_blink_period	= 0, /* No bliking */
	.current_limit		= 2300000, /* uA */
	.flash_sync		= LM3560_SYNC_ON,
	.strobe_polarity	= LM3560_STROBE_POLARITY_HIGH,
	.ledintc_pin_setting	= LM3560_LEDINTC_NTC_THERMISTOR_INPUT,
	.tx1_polarity		= LM3560_TX1_POLARITY_HIGH,
	.tx2_polarity		= LM3560_TX2_POLARITY_HIGH,
	.hw_torch_mode		= LM3560_HW_TORCH_MODE_DISABLE,
};
#endif /* CONFIG_LM3560_FLASHLED */

static DEFINE_MUTEX(sensor_status_lock);
#define VREGL9_IDDIO_LCURRENT 1000
#define VREGL9_IDDIO_HCURRENT 30000

static int sensor_power(struct device *dev, bool enable,
				struct regulator **reg, const char *id)
{
	int rc;

	mutex_lock(&sensor_status_lock);
	if (!*reg)
		*reg = regulator_get(dev, id);
	if (IS_ERR_OR_NULL(*reg)) {
		if (*reg == NULL)
			rc = -ENODEV;
		else
			rc = PTR_ERR(*reg);
		dev_err(dev, "%s: regulator_get failed. rc=%d\n",
			__func__, rc);
		goto err;
	}

	rc = regulator_set_optimum_mode(*reg, enable ?
			VREGL9_IDDIO_HCURRENT : VREGL9_IDDIO_LCURRENT);
	if (rc < 0)
		dev_err(dev, "%s: set_optimum_mode l9 failed, rc=%d", __func__,
									rc);
	else
		rc = 0;

err:
	mutex_unlock(&sensor_status_lock);

	return rc;
}

#if defined CONFIG_INPUT_BMA250_NG || defined CONFIG_INPUT_BMA250
#define BMA250_GPIO 10
#define BMA250_DEFAULT_RATE 50
static int bma250_power_mode(struct device *dev, int enable)
{
	static int powered;
	static struct regulator *reg_acc;
	int rc = 0;

	if ((enable && !powered) || (!enable && powered)) {
		rc = sensor_power(dev, enable, &reg_acc, "acc_vdd");
		if (rc) {
			dev_err(dev, "%s: power setup failed\n", __func__);
			goto out;
		}
		powered = enable;
	}
out:
	return rc;
}

static struct registers bma250_reg_setup = {
	.range                = BMA250_RANGE_2G,
	.bw_sel               = BMA250_BW_250HZ,
};

static struct bma250_platform_data bma250_platform_data = {
	.hw_config	      = bma250_power_mode,
	.reg                  = &bma250_reg_setup,
#ifdef CONFIG_INPUT_BMA250_NG
	.bypass_state         = mpu3050_bypassmode,
	.read_axis_data       = bma250_read_axis_from_mpu3050,
	.check_sleep_status   = check_bma250_sleep_state,
	.vote_sleep_status    = vote_bma250_sleep_state,
#endif
	.rate                 = BMA250_DEFAULT_RATE,
};
#endif

#ifdef CONFIG_INPUT_AKM8972
#define AKM897X_GPIO 70
static int akm897x_gpio_setup(void)
{
	int rc;

	rc = gpio_request(AKM897X_GPIO, "akm8972_drdy");
	if (rc)
		pr_err("%s: gpio_request failed rc=%d\n", __func__, rc);
	return rc;
}

static void akm897x_gpio_shutdown(void)
{
	gpio_free(AKM897X_GPIO);
}

static void akm897x_hw_config(struct device *dev, int enable)
{
	return;
}

static void akm897x_power_mode(struct device *dev, int enable)
{
	static int powered;
	static struct regulator *reg_mag;
	int rc = 0;

	if ((enable && !powered) || (!enable && powered))  {
		rc = sensor_power(dev, enable, &reg_mag, "mag_vdd");
		if (rc) {
			dev_err(dev, "%s: power setup failed\n", __func__);
			return;
		}
		powered = enable;
	}
}


static struct akm8972_platform_data akm8972_platform_data = {
	.setup		= akm897x_gpio_setup,
	.shutdown	= akm897x_gpio_shutdown,
	.hw_config	= akm897x_hw_config,
	.power_mode	= akm897x_power_mode,
};
#endif

#ifdef CONFIG_SENSORS_MPU3050
#define MPU3050_GPIO 69
int mpu3050_gpio_setup(struct device *dev, int enable)
{
	int rc = 0;

	if (enable) {
		rc = gpio_request(MPU3050_GPIO, "MPUIRQ");
		if (rc)
			pr_err("%s: gpio_request failed. rc=%d\n",
					__func__, rc);
	} else {
		gpio_free(MPU3050_GPIO);
	}

	return rc;
}

int mpu3050_power_mode(struct device *dev, int enable)
{
	static int powered;
	static struct regulator *reg_gyro;
	int rc = 0;

	if ((enable && !powered) || (!enable && powered))  {
		rc = sensor_power(dev, enable, &reg_gyro, "gyro_vdd");
		if (rc) {
			dev_err(dev, "%s: power setup failed\n", __func__);
			goto out;
		}
		powered = enable;
	}
out:
	return rc;
}

extern struct mpu3050_platform_data mpu_data;
#endif /* CONFIG_SENSORS_MPU3050 */

#ifdef CONFIG_INPUT_APDS9702
#define APDS9702_DOUT_GPIO   49

static int apds9702_gpio_setup(struct device *dev, int request)
{
	if (request) {
		return gpio_request(APDS9702_DOUT_GPIO, "apds9702_dout");
	} else {
		gpio_free(APDS9702_DOUT_GPIO);
		return 0;
	}
}

static void apds9702_power_mode(struct device *dev, int enable)
{
	static int powered;
	static struct regulator *reg_prox;
	int rc = 0;

	if ((enable && !powered) || (!enable && powered)) {
		rc = sensor_power(dev, enable, &reg_prox, "prox_vdd");
		if (rc) {
			dev_err(dev, "%s: power setup failed\n", __func__);
			goto out;
		}
		powered = enable;
	}
out:
	return;
}

static struct apds9702_platform_data apds9702_pdata = {
	.gpio_dout      = APDS9702_DOUT_GPIO,
	.is_irq_wakeup  = 1,
	.hw_config      = apds9702_power_mode,
	.gpio_setup     = apds9702_gpio_setup,
	.ctl_reg = {
		.trg   = 1,
		.pwr   = 1,
		.burst = 7,
		.frq   = 3,
		.dur   = 2,
		.th    = 15,
		.rfilt = 0,
	},
	.phys_dev_path = "/sys/devices/i2c-12/12-0054"
};
#endif

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi4_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.keep_ahb_clk_on = 1,
};

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi3_pdata = {
	.clk_freq = 400000,
	.src_clk_rate = 24000000,
};

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi10_pdata = {
	.clk_freq = 355000,
	.src_clk_rate = 24000000,
	.use_gsbi_shared_mode = 1,
};

static struct msm_i2c_platform_data msm8960_i2c_qup_gsbi12_pdata = {
	.clk_freq = 355000,
	.src_clk_rate = 24000000,
#ifdef CONFIG_MSM_GSBI12_UART
	.use_gsbi_shared_mode = 1,
#endif
};

static struct spi_board_info spi_board_info[] __initdata = {
	/* No devices yet */
};

static struct platform_device msm_device_saw_core0 = {
	.name          = "saw-regulator",
	.id            = 0,
	.dev	= {
		.platform_data = &msm_saw_regulator_pdata_s5,
	},
};

static struct platform_device msm_device_saw_core1 = {
	.name          = "saw-regulator",
	.id            = 1,
	.dev	= {
		.platform_data = &msm_saw_regulator_pdata_s6,
	},
};

static struct tsens_platform_data msm_tsens_pdata  = {
		.slope			= {910, 910, 910, 910, 910},
		.tsens_factor		= 1000,
		.hw_type		= MSM_8960,
		.tsens_num_sensor	= 5,
};

static struct platform_device msm_tsens_device = {
	.name   = "tsens8960-tm",
	.id = -1,
};

static struct msm_thermal_data msm_thermal_pdata = {
	.sensor_id = 0,
	.poll_ms = 250,
	.limit_temp_degC = 60,
	.temp_hysteresis_degC = 10,
	.freq_step = 2,
};

#ifdef CONFIG_MSM_FAKE_BATTERY
static struct platform_device fish_battery_device = {
	.name = "fish_battery",
};
#endif

#ifdef CONFIG_MACH_BLUE_CDB
static struct platform_device msm8960_device_ext_5v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_MPP_PM_TO_SYS(7),
	.dev	= {
		.platform_data = &msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_5V],
	},
};
#endif

static struct platform_device msm8960_device_ext_l2_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= 91,
	.dev	= {
		.platform_data = &msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_L2],
	},
};

static struct platform_device msm8960_device_ext_3p3v_vreg __devinitdata = {
	.name	= GPIO_REGULATOR_DEV_NAME,
	.id	= PM8921_GPIO_PM_TO_SYS(17),
	.dev	= {
		.platform_data =
			&msm_gpio_regulator_pdata[GPIO_VREG_ID_EXT_3P3V],
	},
};



static struct platform_device msm8960_device_rpm_regulator __devinitdata = {
	.name	= "rpm-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_regulator_pdata,
	},
};

#define CONSOLE_NAME "ttyHSL"
#define CONSOLE_IX 0
#define CONSOLE_OPTIONS "115200,n8"
static int __init setup_serial_console(char *console_flag)
{
	if (console_flag &&
		strlen(console_flag) >= 2 &&
		(console_flag[0] != '0' || console_flag[1] != '0'))
		add_preferred_console(CONSOLE_NAME,
			CONSOLE_IX,
			CONSOLE_OPTIONS);
	return 1;
}

/*
* The S1 Boot configuration TA unit can specify that the serial console
* enable flag will be passed as Kernel boot arg with tag babe09A9.
*/
__setup("semcandroidboot.babe09a9=", setup_serial_console);

#ifdef CONFIG_SERIAL_MSM_HS

#else
static struct msm_serial_hs_platform_data msm_uart_dm8_pdata;
static struct msm_serial_hs_platform_data msm_uart_dm9_pdata;
#endif

#if defined(CONFIG_BT) && defined(CONFIG_BT_HCIUART_ATH3K)
enum WLANBT_STATUS {
	WLANOFF_BTOFF = 1,
	WLANOFF_BTON,
	WLANON_BTOFF,
	WLANON_BTON
};

static DEFINE_MUTEX(ath_wlanbt_mutex);
static int gpio_wlan_sys_rest_en = 26;
static int ath_wlanbt_status = WLANOFF_BTOFF;

static int ath6kl_power_control(int on)
{
	int rc;

	if (on) {
		rc = gpio_request(gpio_wlan_sys_rest_en, "wlan sys_rst_n");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
				__func__, gpio_wlan_sys_rest_en, rc);
			return rc;
		}
		rc = gpio_direction_output(gpio_wlan_sys_rest_en, 0);
		msleep(200);
		rc = gpio_direction_output(gpio_wlan_sys_rest_en, 1);
		msleep(100);
	} else {
		gpio_set_value(gpio_wlan_sys_rest_en, 0);
		rc = gpio_direction_input(gpio_wlan_sys_rest_en);
		msleep(100);
		gpio_free(gpio_wlan_sys_rest_en);
	}
	return 0;
};

static int ath6kl_wlan_power(int on)
{
	int ret = 0;

	mutex_lock(&ath_wlanbt_mutex);
	if (on) {
		if (ath_wlanbt_status == WLANOFF_BTOFF) {
			ret = ath6kl_power_control(1);
			ath_wlanbt_status = WLANON_BTOFF;
		} else if (ath_wlanbt_status == WLANOFF_BTON)
			ath_wlanbt_status = WLANON_BTON;
	} else {
		if (ath_wlanbt_status == WLANON_BTOFF) {
			ret = ath6kl_power_control(0);
			ath_wlanbt_status = WLANOFF_BTOFF;
		} else if (ath_wlanbt_status == WLANON_BTON)
			ath_wlanbt_status = WLANOFF_BTON;
	}
	mutex_unlock(&ath_wlanbt_mutex);
	pr_debug("%s on= %d, wlan_status= %d\n",
		__func__, on, ath_wlanbt_status);
	return ret;
};

static struct wifi_platform_data ath6kl_wifi_control = {
	.set_power      = ath6kl_wlan_power,
};

static struct platform_device msm_wlan_power_device = {
	.name = "ath6kl_power",
	.dev            = {
		.platform_data = &ath6kl_wifi_control,
	},
};

static struct resource bluesleep_resources[] = {
	{
		.name   = "gpio_host_wake",
		.start  = 27,
		.end    = 27,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "gpio_ext_wake",
		.start  = 29,
		.end    = 29,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "host_wake",
		.start  = MSM_GPIO_TO_INT(27),
		.end    = MSM_GPIO_TO_INT(27),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name		= "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

static int gpio_bt_sys_rest_en = 28;

static int bluetooth_power(int on)
{
	int rc;

	mutex_lock(&ath_wlanbt_mutex);
	if (on) {
		if (ath_wlanbt_status == WLANOFF_BTOFF) {
			ath6kl_power_control(1);
			ath_wlanbt_status = WLANOFF_BTON;
		} else if (ath_wlanbt_status == WLANON_BTOFF)
			ath_wlanbt_status = WLANON_BTON;

		rc = gpio_request(gpio_bt_sys_rest_en, "bt sys_rst_n");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
				__func__, gpio_bt_sys_rest_en, rc);
			mutex_unlock(&ath_wlanbt_mutex);
			return rc;
		}
		rc = gpio_direction_output(gpio_bt_sys_rest_en, 0);
		msleep(20);
		rc = gpio_direction_output(gpio_bt_sys_rest_en, 1);
		msleep(100);
	} else {
		gpio_set_value(gpio_bt_sys_rest_en, 0);
		rc = gpio_direction_input(gpio_bt_sys_rest_en);
		msleep(100);
		gpio_free(gpio_bt_sys_rest_en);

		if (ath_wlanbt_status == WLANOFF_BTON) {
			ath6kl_power_control(0);
			ath_wlanbt_status = WLANOFF_BTOFF;
		} else if (ath_wlanbt_status == WLANON_BTON)
			ath_wlanbt_status = WLANON_BTOFF;
	}
	mutex_unlock(&ath_wlanbt_mutex);
	pr_debug("%s on= %d, wlan_status= %d\n",
		__func__, on, ath_wlanbt_status);
	return 0;
};

static void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
	return;
};
#else
#define bt_power_init(x) do {} while (0)
#endif

#ifdef CONFIG_MSM_GSBI12_UART
static struct msm_serial_hslite_platform_data msm_uart_gsbi12_pdata = {
	.config_gpio	= 1,
	.uart_tx_gpio	= 42,
	.uart_rx_gpio	= 43,
	.line		= 3,
	.type		= PORT_IRDA,
};
#endif

static struct platform_device *common_devices[] __initdata = {
	&msm8960_device_dmov,
	&msm_device_smd,
	&msm8960_device_uart_gsbi8,
#ifdef CONFIG_MSM_GSBI12_UART
	&msm8960_device_uart_gsbi12,
#endif
//	&msm_device_uart_dm6,
	&msm_device_saw_core0,
	&msm_device_saw_core1,
#ifdef CONFIG_MACH_BLUE_CDB
	&msm8960_device_ext_5v_vreg,
#endif
	&msm8960_device_ssbi_pmic,
	&msm8960_device_qup_i2c_gsbi3,
	&msm8960_device_qup_i2c_gsbi4,
	&msm8960_device_qup_i2c_gsbi10,
#ifndef CONFIG_MSM_DSPS
	&msm8960_device_qup_i2c_gsbi12,
#endif
	&msm_slim_ctrl,
	&msm_device_wcnss_wlan,
#if defined(CONFIG_BT) && defined(CONFIG_BT_HCIUART_ATH3K)
	&msm_bluesleep_device,
	&msm_bt_power_device,
	&msm_wlan_power_device,
#endif
#if defined(CONFIG_QSEECOM)
	&qseecom_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_device_sps,
#ifdef CONFIG_MSM_FAKE_BATTERY
	&fish_battery_device,
#endif
	&msm8960_fmem_device,
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	&msm8960_android_pmem_device,
	&msm8960_android_pmem_adsp_device,
	&msm8960_android_pmem_audio_device,
#endif
#endif
	&msm_device_bam_dmux,
	&msm_fm_platform_init,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
#ifdef CONFIG_MSM_USE_TSIF1
	&msm_device_tsif[1],
#else
	&msm_device_tsif[0],
#endif
#endif
	&msm_device_tspp,
#ifdef CONFIG_HW_RANDOM_MSM
	&msm_device_rng,
#endif
#ifdef CONFIG_ION_MSM
	&msm8960_ion_dev,
#endif
	&msm8960_rpm_device,
	&msm8960_rpm_log_device,
	&msm8960_rpm_stat_device,
	&msm8960_rpm_master_stat_device,
	&msm_device_tz_log,
	&coresight_tpiu_device,
	&coresight_etb_device,
	&coresight_funnel_device,
	&coresight_etm0_device,
	&coresight_etm1_device,
	&msm_device_dspcrashd_8960,
	&msm8960_device_watchdog,
	&msm8960_rtb_device,
	&msm8960_device_cache_erp,
	&msm8960_device_ebi1_ch0_erp,
	&msm8960_device_ebi1_ch1_erp,
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
#ifdef CONFIG_RAMDUMP_TAGS
	&rdtags_device,
#endif
#ifdef CONFIG_RAMDUMP_CRASH_LOGS
	&ramdumplog_device,
#endif
	&msm8960_cache_dump_device,
	&msm8960_iommu_domain_device,
	&msm_tsens_device,
	&msm8960_pc_cntr,
};

static struct platform_device *cdp_devices[] __initdata = {
#ifdef CONFIG_USB_NCP373
	&ncp373_device,
#endif
	&msm_8960_q6_lpass,
	&msm_8960_riva,
	&msm_pil_tzapps,
	&msm_pil_dsps,
	&msm_pil_vidc,
	&msm8960_device_otg,
	&msm8960_device_gadget_peripheral,
	&msm_device_hsusb_host,
	&android_usb_device,
	&msm_pcm,
	&msm_multi_ch_pcm,
	&msm_lowlatency_pcm,
	&msm_pcm_routing,
	&msm_cpudai0,
	&msm_cpudai1,
	&msm8960_cpudai_slimbus_2_rx,
	&msm8960_cpudai_slimbus_2_tx,
	&msm_cpudai_hdmi_rx,
	&msm_cpudai_bt_rx,
	&msm_cpudai_bt_tx,
	&msm_cpudai_fm_rx,
	&msm_cpudai_fm_tx,
	&msm_cpudai_auxpcm_rx,
	&msm_cpudai_auxpcm_tx,
	&msm_cpu_fe,
	&msm_stub_codec,
#ifdef CONFIG_MSM_GEMINI
	&msm8960_gemini_device,
#endif
#ifdef CONFIG_MSM_MERCURY
	&msm8960_mercury_device,
#endif
	&msm_voice,
	&msm_voip,
	&msm_lpa_pcm,
	&msm_cpudai_afe_01_rx,
	&msm_cpudai_afe_01_tx,
	&msm_cpudai_afe_02_rx,
	&msm_cpudai_afe_02_tx,
	&msm_pcm_afe,
	&msm_compr_dsp,
	&msm_cpudai_incall_music_rx,
	&msm_cpudai_incall_record_rx,
	&msm_cpudai_incall_record_tx,
	&msm_pcm_hostless,
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
};

static void __init msm8960_i2c_init(void)
{
	void *gsbi10_mem = ioremap_nocache(0x1A200000, 4);
#ifdef CONFIG_MSM_GSBI12_UART
	void *gsbi12_mem = ioremap_nocache(0x12480000, 4);
#endif

	/* Setting protocol code to 0x60 for dual UART/I2C in GSBI10 */
	writel_relaxed(0x6 << 4, gsbi10_mem);
	iounmap(gsbi10_mem);

#ifdef CONFIG_MSM_GSBI12_UART
	/* Setting protocol code to 0x60 for dual UART/I2C in GSBI12 */
	writel_relaxed(0x6 << 4, gsbi12_mem);
	iounmap(gsbi12_mem);
#endif

	msm8960_device_qup_i2c_gsbi4.dev.platform_data =
					&msm8960_i2c_qup_gsbi4_pdata;

	msm8960_device_qup_i2c_gsbi3.dev.platform_data =
					&msm8960_i2c_qup_gsbi3_pdata;

	msm8960_device_qup_i2c_gsbi10.dev.platform_data =
					&msm8960_i2c_qup_gsbi10_pdata;

	msm8960_device_qup_i2c_gsbi12.dev.platform_data =
					&msm8960_i2c_qup_gsbi12_pdata;
}

static void __init msm8960_gfx_init(void)
{
	struct kgsl_device_platform_data *kgsl_3d0_pdata =
		msm_kgsl_3d0.dev.platform_data;
	uint32_t soc_platform_version = socinfo_get_version();

	/* Fixup data that needs to change based on GPU ID */
	if (cpu_is_msm8960ab()) {
		kgsl_3d0_pdata->chipid = ADRENO_CHIPID(3, 2, 1, 0);
		/* 8960PRO nominal clock rate is 320Mhz */
		kgsl_3d0_pdata->pwrlevel[1].gpu_freq = 320000000;
	} else {
		kgsl_3d0_pdata->iommu_count = 1;
		if (SOCINFO_VERSION_MAJOR(soc_platform_version) == 1) {
			kgsl_3d0_pdata->pwrlevel[0].gpu_freq = 320000000;
			kgsl_3d0_pdata->pwrlevel[1].gpu_freq = 266667000;
		}
		if (SOCINFO_VERSION_MAJOR(soc_platform_version) >= 3) {
			/* 8960v3 GPU registers returns 5 for patch release
			 * but it should be 6, so dummy up the chipid here
			 * based the platform type
			 */
			kgsl_3d0_pdata->chipid = ADRENO_CHIPID(2, 2, 0, 6);
		}
	}

	/* Register the 3D core */
	platform_device_register(&msm_kgsl_3d0);

	/* Register the 2D cores if we are not 8960PRO */
	if (!cpu_is_msm8960ab()) {
		platform_device_register(&msm_kgsl_2d0);
		platform_device_register(&msm_kgsl_2d1);
	}
}

static struct msm_rpmrs_level msm_rpmrs_levels[] = {
	{
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1, 784, 180000, 100,
	},

	{
		MSM_PM_SLEEP_MODE_RETENTION,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		415, 715, 340827, 475,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE,
		MSM_RPMRS_LIMITS(ON, ACTIVE, MAX, ACTIVE),
		true,
		1300, 228, 1200000, 2000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, GDHS, MAX, ACTIVE),
		false,
		2000, 138, 1208400, 3200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(ON, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		6000, 119, 1850300, 9000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, GDHS, MAX, ACTIVE),
		false,
		9200, 68, 2839200, 16400,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, MAX, ACTIVE),
		false,
		10300, 63, 3128000, 18200,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, ACTIVE, RET_HIGH),
		false,
		18000, 10, 4602600, 27000,
	},

	{
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
		MSM_RPMRS_LIMITS(OFF, HSFS_OPEN, RET_HIGH, RET_LOW),
		false,
		20000, 2, 5752000, 32000,
	},
};


static struct msm_rpmrs_platform_data msm_rpmrs_data __initdata = {
	.levels = &msm_rpmrs_levels[0],
	.num_levels = ARRAY_SIZE(msm_rpmrs_levels),
	.vdd_mem_levels  = {
		[MSM_RPMRS_VDD_MEM_RET_LOW]	= 750000,
		[MSM_RPMRS_VDD_MEM_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_MEM_ACTIVE]	= 1050000,
		[MSM_RPMRS_VDD_MEM_MAX]		= 1150000,
	},
	.vdd_dig_levels = {
		[MSM_RPMRS_VDD_DIG_RET_LOW]	= 500000,
		[MSM_RPMRS_VDD_DIG_RET_HIGH]	= 750000,
		[MSM_RPMRS_VDD_DIG_ACTIVE]	= 950000,
		[MSM_RPMRS_VDD_DIG_MAX]		= 1150000,
	},
	.vdd_mask = 0x7FFFFF,
	.rpmrs_target_id = {
		[MSM_RPMRS_ID_PXO_CLK]		= MSM_RPM_ID_PXO_CLK,
		[MSM_RPMRS_ID_L2_CACHE_CTL]	= MSM_RPM_ID_LAST,
		[MSM_RPMRS_ID_VDD_DIG_0]	= MSM_RPM_ID_PM8921_S3_0,
		[MSM_RPMRS_ID_VDD_DIG_1]	= MSM_RPM_ID_PM8921_S3_1,
		[MSM_RPMRS_ID_VDD_MEM_0]	= MSM_RPM_ID_PM8921_L24_0,
		[MSM_RPMRS_ID_VDD_MEM_1]	= MSM_RPM_ID_PM8921_L24_1,
		[MSM_RPMRS_ID_RPM_CTL]		= MSM_RPM_ID_RPM_CTL,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_TZ,
};

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_FLUID (1 << 4)
#define I2C_LIQUID (1 << 5)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

/* AVTimer */
static struct platform_device msm_dev_avtimer_device = {
	.name = "dev_avtimer",
	.dev = { .platform_data = &dev_avtimer_pdata },
};

/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS
#define DSPS_PIL_GENERIC_NAME		"dsps"
#endif /* CONFIG_MSM_DSPS */

static void __init msm8960_init_dsps(void)
{
#ifdef CONFIG_MSM_DSPS
	struct msm_dsps_platform_data *pdata =
		msm_dsps_device.dev.platform_data;
	pdata->pil_name = DSPS_PIL_GENERIC_NAME;
	pdata->gpios = NULL;
	pdata->gpios_num = 0;

	platform_device_register(&msm_dsps_device);
#endif /* CONFIG_MSM_DSPS */
}

#ifdef CONFIG_VIBRATOR_LC898300
#define VIB_RSTB	79
#define VIB_EN		53

static struct regulator *vib_supply_ldo;

static int lc898300_rstb_gpio_setup(bool value)
{
	gpio_set_value(VIB_RSTB, value);
	return 0;
}

static int lc898300_en_gpio_setup(bool value)
{
	gpio_set_value(VIB_EN, value);
	return 0;
}

static int lc898300_gpio_allocate(struct device *dev)
{
	int rc;

	rc = gpio_request(VIB_RSTB, "vibrator reset_gpio");
	if (rc) {
		dev_err(dev, "%s: GPIO %d: request failed. rc=%d\n",
			      __func__, VIB_RSTB, rc);
		return rc;
	}

	rc = gpio_direction_output(VIB_RSTB, 0);
	if (rc) {
		dev_err(dev, "%s: GPIO %d: direction out failed rc=%d\n",
			      __func__, VIB_RSTB, rc);
		goto error1;
	}

	rc = gpio_request(VIB_EN, "vibrator enable_gpio");
	if (rc) {
		dev_err(dev, "%s: GPIO %d: request failed. rc=%d\n",
			      __func__, VIB_EN, rc);
		goto error1;
	}

	rc = gpio_direction_output(VIB_EN, 0);
	if (rc) {
		dev_err(dev, "%s: GPIO %d: direction out failed rc=%d\n",
			__func__, VIB_EN, rc);
		goto error2;
	}

	return rc;

error2:
	gpio_free(VIB_EN);
error1:
	gpio_free(VIB_RSTB);
	return rc;
}

static void lc898300_gpio_release(struct device *dev)
{
	gpio_free(VIB_RSTB);
	gpio_free(VIB_EN);
}

static const char *vibrator_reg_name = "lc898300_vdd";
static int lc898300_power_config(struct device *dev, bool on)
{
	int rc = 0;

	if (on) {
		vib_supply_ldo = regulator_get(dev, vibrator_reg_name);
		if (IS_ERR_OR_NULL(vib_supply_ldo)) {
			dev_err(dev, "%s: could not get %s, rc = %ld\n",
				__func__, vibrator_reg_name,
				PTR_ERR(vib_supply_ldo));
			vib_supply_ldo = NULL;
			return -ENODEV;
		}
		rc = regulator_set_voltage(vib_supply_ldo, 3050000, 3050000);
		if (rc) {
			dev_err(dev, "%s: unable to set vibrator supply voltage"
				" to 3.05V, rc = %d!\n", __func__, rc);
			rc = -ENODEV;
			goto error;
		}
		return 0;
	}

error:
	regulator_put(vib_supply_ldo);
	vib_supply_ldo = NULL;
	return rc;
}

static int lc898300_power_enable(struct device *dev, bool on)
{
	int rc = 0;

	if (vib_supply_ldo == NULL) {
		dev_err(dev, "%s: vib_supply_ldo = NULL\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	if (on) {
		rc = regulator_enable(vib_supply_ldo);
		if (rc)
			dev_err(dev, "%s: enable regulator %s failed\n",
					__func__ , vibrator_reg_name);
	} else {
		rc = regulator_disable(vib_supply_ldo);
		if (rc)
			dev_err(dev, "%s: disable regulator %s failed\n",
					__func__ , vibrator_reg_name);
	}
out:
	return rc;
}

static struct lc898300_platform_data lc898300_platform_data = {
	.name = "vibrator",
	.power_config = lc898300_power_config,
	.power_enable = lc898300_power_enable,
	.gpio_allocate = lc898300_gpio_allocate,
	.gpio_release = lc898300_gpio_release,
	.rstb_gpio_setup = lc898300_rstb_gpio_setup,
	.en_gpio_setup = lc898300_en_gpio_setup,
	.vib_cmd_info = &lc898300_vib_cmd_data,
};
#endif

static int hsic_peripheral_status = 1;
static DEFINE_MUTEX(hsic_status_lock);

void peripheral_connect()
{
	int rc = 0;
	mutex_lock(&hsic_status_lock);
	if (hsic_peripheral_status)
		goto out;
	rc = platform_device_add(&msm_device_hsic_host);
	if (rc)
		pr_err("%s: failed to add a platform device, rc=%d\n",
			__func__, rc);
	else
		hsic_peripheral_status = 1;
out:
	mutex_unlock(&hsic_status_lock);
}
EXPORT_SYMBOL(peripheral_connect);

void peripheral_disconnect()
{
	mutex_lock(&hsic_status_lock);
	if (!hsic_peripheral_status)
		goto out;
	platform_device_del(&msm_device_hsic_host);
	hsic_peripheral_status = 0;
out:
	mutex_unlock(&hsic_status_lock);
}
EXPORT_SYMBOL(peripheral_disconnect);

static void __init msm8960_init_smsc_hub(void)
{
	uint32_t version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) == 1)
		return;

	if (machine_is_msm8960_liquid())
		platform_device_register(&smsc_hub_device);
}

static struct i2c_board_info gsbi3_peripherals_info[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_CLEARPAD
	{
		/* Config-spec is 8-bit = 0x58, src-code need 7-bit => 0x2c */
		I2C_BOARD_INFO(CLEARPADI2C_NAME, 0x58 >> 1),
		.platform_data = &clearpad_platform_data,
	}
#endif
#ifdef CONFIG_RMI4_BUS
	{
		/* Config-spec is 8-bit = 0x58, src-code need 7-bit => 0x2c */
		I2C_BOARD_INFO(RMI4_I2C_ADAPTER_NAME, 0x58 >> 1),
		.platform_data = &rmi4_platform_data,
	},
#endif
};

static struct i2c_board_info gsbi10_peripherals_info[] __initdata = {
#ifdef CONFIG_LM3560_FLASHLED
	{
		/* Config-spec is 8-bit = 0xa6, src-code need 7-bit => 0x53 */
		I2C_BOARD_INFO(LM3560_DRV_NAME, 0xa6 >> 1),
		.platform_data = &lm3560_platform_data,
	},
#endif
#ifdef CONFIG_LM3561_FLASHLED
	{
		/* Config-spec is 8-bit = 0xa6, src-code need 7-bit => 0x53 */
		I2C_BOARD_INFO(LM3561_DRV_NAME, 0xa6 >> 1),
		.platform_data = &lm3561_platform_data,
	},
#endif
#ifdef CONFIG_LEDS_AS3676
	{
		/* Config-spec is 8-bit = 0x80, src-code need 7-bit => 0x40 */
		I2C_BOARD_INFO("as3676", 0x80 >> 1),
		.platform_data = &as3676_platform_data,
	},
#endif
#ifdef CONFIG_FB_MSM_MHL_SII8334
	{
		I2C_BOARD_INFO("sii8334", 0x72 >> 1),
		.irq = MSM_GPIO_TO_INT(MSM_GPIO_MHL_IRQ_N),
		.platform_data = &mhl_sii_pdata,
	},
#endif /* CONFIG_FB_MSM_MHL_SII8334 */
#ifdef CONFIG_NFC_PN544
	{
		/* Config-spec is 8-bit = 0x50, src-code need 7-bit => 0x28 */
		I2C_BOARD_INFO(PN544_DEVICE_NAME, 0x50 >> 1),
		.irq = MSM_GPIO_TO_INT(NXP_GPIO_NFC_IRQ),
		.platform_data = &pn544_pdata,
	},
#endif
};

static struct i2c_board_info gsbi12_peripherals_info[] __initdata = {
#ifdef CONFIG_VIBRATOR_LC898300
	{
		/* Config-spec is 8-bit = 0x92, src-code need 7-bit => 0x49 */
		I2C_BOARD_INFO(LC898300_I2C_NAME, 0x92 >> 1),
		.platform_data = &lc898300_platform_data,
	},
#endif
#if defined CONFIG_INPUT_BMA250 || defined CONFIG_INPUT_BMA250_NG
	{
		/* Config-spec is 8-bit = 0x30, src-code need 7-bit => 0x18 */
		I2C_BOARD_INFO("bma250", 0x30 >> 1),
		.irq = MSM_GPIO_TO_INT(BMA250_GPIO),
		.platform_data = &bma250_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_AKM8972
	{
		/* Config-spec is 8-bit = 0x18, src-code need 7-bit => 0x0c */
		I2C_BOARD_INFO(AKM8972_I2C_NAME, 0x18 >> 1),
		.irq = MSM_GPIO_TO_INT(AKM897X_GPIO),
		.platform_data = &akm8972_platform_data,
	},
#endif
#ifdef CONFIG_SENSORS_MPU3050
	{
		/* Config-spec is 8-bit = 0xD0, src-code need 7-bit => 0x68 */
		I2C_BOARD_INFO(MPU_NAME, 0xD0 >> 1),
		.irq = MSM_GPIO_TO_INT(MPU3050_GPIO),
		.platform_data = &mpu_data,
	},
#endif
#ifdef CONFIG_INPUT_APDS9702
	{
		/* Config-spec is 8-bit = 0xA8, src-code need 7-bit => 0x54 */
		I2C_BOARD_INFO(APDS9702_NAME, 0xA8 >> 1),
		.platform_data = &apds9702_pdata,
	},
#endif
};

static struct i2c_registry msm8960_i2c_devices[] __initdata = {
	{
		I2C_SURF | I2C_FFA,
		MSM_8960_GSBI3_QUP_I2C_BUS_ID,
		gsbi3_peripherals_info,
		ARRAY_SIZE(gsbi3_peripherals_info),
	},
	{
		0,
		MSM_8960_GSBI10_QUP_I2C_BUS_ID,
		gsbi10_peripherals_info,
		ARRAY_SIZE(gsbi10_peripherals_info),
	},
	{
		0,
		MSM_8960_GSBI12_QUP_I2C_BUS_ID,
		gsbi12_peripherals_info,
		ARRAY_SIZE(gsbi12_peripherals_info),
	}
};
#endif /* CONFIG_I2C */

static void __init register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	int i;
#ifdef CONFIG_MSM_CAMERA
	struct i2c_registry msm8960_camera_i2c_devices = {
		I2C_SURF | I2C_FFA | I2C_FLUID | I2C_LIQUID | I2C_RUMI,
		MSM_8960_GSBI4_QUP_I2C_BUS_ID,
		msm8960_camera_board_info.board_info,
		msm8960_camera_board_info.num_i2c_board_info,
	};
#endif

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8960_i2c_devices); ++i) {
		i2c_register_board_info(msm8960_i2c_devices[i].bus,
					msm8960_i2c_devices[i].info,
					msm8960_i2c_devices[i].len);
	}
#ifdef CONFIG_MSM_CAMERA
	i2c_register_board_info(msm8960_camera_i2c_devices.bus,
				msm8960_camera_i2c_devices.info,
				msm8960_camera_i2c_devices.len);
#endif
#endif
}

static void setup_init_vregs(void)
{
	struct regulator *vreg;
	int rc;

	vreg = regulator_get(NULL, "8921_l9");
	if (IS_ERR(vreg)) {
		pr_err("%s: failed to get 8921_l9! rc = %ld\n", __func__,
			PTR_ERR(vreg));
		return;
	}

	rc = regulator_set_voltage(vreg, 2850000, 2850000);
	if (rc) {
		pr_err("%s: failed to set voltage on 8921_l9!\n", __func__);
		return;
	}

	rc = regulator_enable(vreg);
	if (rc) {
		pr_err("%s: failed to enable 8921_l9!\n", __func__);
		return;
	}
	mdelay(10);
	vreg = regulator_get(NULL, "8921_lvs4");
	if (IS_ERR(vreg)) {
		pr_err("%s: failed to get vreg 8921_lvs4!\n", __func__);
		return;
	}

	rc = regulator_enable(vreg);
	if (rc) {
		pr_err("%s: failed to enable 8921_lvs4!\n", __func__);
		return;
	}

#ifdef CONFIG_SONY_FELICA_SUPPORT
#ifndef CONFIG_SONY_FELICA_NFC_SUPPORT
	vreg = regulator_get(NULL, "8921_lvs6");
	if (IS_ERR(vreg)) {
		pr_err("%s: failed to get vreg 8921_lvs6!\n", __func__);
		return;
	}

	rc = regulator_enable(vreg);
	if (rc) {
		pr_err("%s: failed to enable 8921_lvs6!\n", __func__);
		return;
	}
#endif
#endif
}

static void __init msm8960_tsens_init(void)
{
	if (cpu_is_msm8960())
		if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1)
			return;

	msm_tsens_early_init(&msm_tsens_pdata);
}

static void __init msm8960ab_update_krait_spm(void)
 {
 	int i;
 

	/* Update the SPM sequences for SPC and PC */
	for (i = 0; i < ARRAY_SIZE(msm_spm_data); i++) {
		int j;
		struct msm_spm_platform_data *pdata = &msm_spm_data[i];
		for (j = 0; j < pdata->num_modes; j++) {
			if (pdata->modes[j].cmd ==
					spm_power_collapse_without_rpm)
				pdata->modes[j].cmd =
				spm_power_collapse_without_rpm_krait_v3;
			else if (pdata->modes[j].cmd ==
					spm_power_collapse_with_rpm)
				pdata->modes[j].cmd =
				spm_power_collapse_with_rpm_krait_v3;
		}
	}
}

static void __init msm8960ab_update_retention_spm(void)
{
	int i;

	/* Update the SPM sequences for krait retention on all cores */
	for (i = 0; i < ARRAY_SIZE(msm_spm_data); i++) {
		int j;
		struct msm_spm_platform_data *pdata = &msm_spm_data[i];
		for (j = 0; j < pdata->num_modes; j++) {
			if (pdata->modes[j].cmd ==
					spm_retention_cmd_sequence)
				pdata->modes[j].cmd =
				spm_retention_with_krait_v3_cmd_sequence;
		}
	}
}

#ifdef CONFIG_MSM_GSBI12_UART
#define UART_RX_IRDA_GPIO 43
static void __init setup_gsbi12_uart_gpio(void)
{
	static struct gpiomux_setting gpio_2ma_no_pull_in = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir  = GPIOMUX_IN,
	};

	if (msm_gpiomux_write(UART_RX_IRDA_GPIO, GPIOMUX_SUSPENDED,
				&gpio_2ma_no_pull_in, NULL)) {
		pr_err("%s: msm_gpiomux_write to GPIO %d failed.",
			__func__, UART_RX_IRDA_GPIO);
	}
}
#endif

static void __init msm8960_cdp_init(void)
{
	if (meminfo_init(SYS_MEMORY, SZ_256M) < 0)
		pr_err("meminfo_init() failed!\n");

	platform_device_register(&msm_gpio_device);
	msm8960_tsens_init();
	msm_thermal_init(&msm_thermal_pdata);
	BUG_ON(msm_rpm_init(&msm8960_rpm_data));
	BUG_ON(msm_rpmrs_levels_init(&msm_rpmrs_data));

	regulator_suppress_info_printing();
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");
	configure_msm8960_power_grid();
	platform_device_register(&msm8960_device_rpm_regulator);
	msm_clock_init(&msm8960_clock_init_data);
	msm8960_device_otg.dev.platform_data = &msm_otg_pdata;
	android_usb_pdata.swfi_latency =
		msm_rpmrs_levels[0].latency_us;
	msm_device_hsic_host.dev.platform_data = &msm_hsic_pdata;
	msm8960_init_gpiomux();
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));


	msm8960_init_pmic();
	msm8960_i2c_init();
	msm8960_gfx_init();
 	if (cpu_is_msm8960ab())
		msm8960ab_update_krait_spm();
	if (cpu_is_krait_v3()) {
		msm_pm_set_tz_retention_flag(0);
		msm8960ab_update_retention_spm();
	} else {
		msm_pm_set_tz_retention_flag(1);
	}
	msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	msm_spm_l2_init(msm_spm_l2_data);
	msm8960_init_buses();
	if (cpu_is_msm8960ab()) {
		platform_add_devices(msm8960ab_footswitch,
				     msm8960ab_num_footswitch);
	} else {
		platform_add_devices(msm8960_footswitch,
				     msm8960_num_footswitch);
	}
	if (machine_is_msm8960_liquid())
		platform_device_register(&msm8960_device_ext_3p3v_vreg);
	if (machine_is_msm8960_cdp())
		platform_device_register(&msm8960_device_ext_l2_vreg);

	//if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE)
		//platform_device_register(&msm8960_device_uart_gsbi8);
	//else
		//platform_device_register(&msm8960_device_uart_gsbi8);




	if (cpu_is_msm8960ab())
		platform_device_register(&msm8960ab_device_acpuclk);
	else
		platform_device_register(&msm8960_device_acpuclk);
	platform_add_devices(common_devices, ARRAY_SIZE(common_devices));
	msm8960_add_vidc_device();
#ifdef CONFIG_MSM_GSBI12_UART
	msm8960_device_uart_gsbi12.dev.platform_data = &msm_uart_gsbi12_pdata;
#endif
	msm8960_pm8921_gpio_mpp_init();
	/* Don't add modem devices on APQ targets */
	if (socinfo_get_id() != 124) {
		platform_device_register(&msm_8960_q6_mss_fw);
		platform_device_register(&msm_8960_q6_mss_sw);
	}
	platform_add_devices(cdp_devices, ARRAY_SIZE(cdp_devices));
	msm8960_init_smsc_hub();
#ifdef CONFIG_MSM_CAMERA
	msm8960_init_cam();
#endif
	bt_power_init();
	msm8960_init_mmc();
	setup_init_vregs();
#ifdef CONFIG_MSM_GSBI12_UART
	setup_gsbi12_uart_gpio();
#endif
	register_i2c_devices();
	msm8960_init_fb();
	slim_register_board_info(msm_slim_devices,
		ARRAY_SIZE(msm_slim_devices));
	msm8960_init_dsps();
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE) {
		mdm_sglte_device.dev.platform_data = &sglte_platform_data;
		platform_device_register(&mdm_sglte_device);
	}
	if (machine_is_msm8960_mtp() || machine_is_msm8960_fluid() ||
		machine_is_msm8960_cdp()) {
		platform_device_register(&msm_dev_avtimer_device);
	}
}

MACHINE_START(BLUE_CDB, "BLUE CDB")
	.map_io = msm8960_map_io,
	.reserve = msm8960_reserve,
	.init_irq = msm8960_init_irq,
	.handle_irq = gic_handle_irq,
	.timer = &msm_timer,
	.init_machine = msm8960_cdp_init,
	.init_early = msm8960_allocate_memory_regions,
	.init_very_early = msm8960_early_memory,
	.restart = msm_restart
MACHINE_END
