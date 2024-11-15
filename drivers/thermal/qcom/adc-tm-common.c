// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include "adc-tm.h"

/*
 * Voltage to temperature table for NTCG104EF104 thermistor with
 * 1.875V reference and 100k pull-up.
 */
static const struct adc_tm_map_pt adcmap_100k_104ef_104fb_1875_vref[] = {
	{ 1831000,	-40000 },
	{ 1814000,	-35000 },
	{ 1791000,	-30000 },
	{ 1761000,	-25000 },
	{ 1723000,	-20000 },
	{ 1675000,	-15000 },
	{ 1616000,	-10000 },
	{ 1545000,	-5000 },
	{ 1463000,	0 },
	{ 1370000,	5000 },
	{ 1268000,	10000 },
	{ 1160000,	15000 },
	{ 1049000,	20000 },
	{ 937000,	25000 },
	{ 828000,	30000 },
	{ 726000,	35000 },
	{ 630000,	40000 },
	{ 544000,	45000 },
	{ 467000,	50000 },
	{ 399000,	55000 },
	{ 340000,	60000 },
	{ 290000,	65000 },
	{ 247000,	70000 },
	{ 209000,	75000 },
	{ 179000,	80000 },
	{ 153000,	85000 },
	{ 130000,	90000 },
	{ 112000,	95000 },
	{ 96000,	100000 },
	{ 82000,	105000 },
	{ 71000,	110000 },
	{ 62000,	115000 },
	{ 53000,	120000 },
	{ 46000,	125000 },
};

/*
 * Resistance to temperature table for NTCG104EF104 thermistor with
 * 100k pull-up.
 */
static const struct adc_tm_map_pt adcmap_100k_adc7[] = {
   {4251000,-40000 },
   {3962000,-39000 },
   {3695000,-38000 },
   {3447000,-37000 },
   {3218000,-36000 },
   {3005000,-35000 },
   {2807000,-34000 },
   {2624000,-33000 },
   {2454000,-32000 },
   {2296000,-31000 },
   {2149000,-30000 },
   {2012000,-29000 },
   {1885000,-28000 },
   {1767000,-27000 },
   {1656000,-26000 },
   {1554000,-25000 },
   {1458000,-24000 },
   {1369000,-23000 },
   {1286000,-22000 },
   {1208000,-21000 },
   {1135000,-20000 },
   {1068000,-19000 },
   {1004000,-18000 },
   {945000,-17000 },
   {889600,-16000 },
   {837800,-15000 },
   {789300,-14000 },
   {743900,-13000 },
   {701300,-12000 },
   {661500,-11000 },
   {624100,-10000 },
   {589000,-9000 },
   {556200,-8000 },
   {525300,-7000 },
   {496300,-6000 },
   {469100,-5000 },
   {443500,-4000 },
   {419500,-3000 },
   {396900,-2000 },
   {375600,-1000 },
   {355600,0 },
   {336800,1000 },
   {319100,2000 },
   {302400,3000 },
   {286700,4000 },
   {271800,5000 },
   {257800,6000 },
   {244700,7000 },
   {232200,8000 },
   {220500,9000 },
   {209400,10000 },
   {198900,11000 },
   {189000,12000 },
   {179700,13000 },
   {170900,14000 },
   {162500,15000 },
   {154600,16000 },
   {147200,17000 },
   {140100,18000 },
   {133400,19000 },
   {127000,20000 },
   {121000,21000 },
   {115400,22000 },
   {110000,23000 },
   {104800,24000 },
   {100000,25000 },
   {95400,26000 },
   {91040,27000 },
   {86900,28000 },
   {82970,29000 },
   {79230,30000 },
   {75690,31000 },
   {72320,32000 },
   {69120,33000 },
   {66070,34000 },
   {63180,35000 },
   {60420,36000 },
   {57810,37000 },
   {55310,38000 },
   {52940,39000 },
   {50680,40000 },
   {48530,41000 },
   {46490,42000 },
   {44530,43000 },
   {42670,44000 },
   {40900,45000 },
   {39210,46000 },
   {37600,47000 },
   {36060,48000 },
   {34600,49000 },
   {33190,50000 },
   {31860,51000 },
   {30580,52000 },
   {29360,53000 },
   {28200,54000 },
   {27090,55000 },
   {26030,56000 },
   {25010,57000 },
   {24040,58000 },
   {23110,59000 },
   {22220,60000 },
   {21370,61000 },
   {20560,62000 },
   {19780,63000 },
   {19040,64000 },
   {18320,65000 },
   {17640,66000 },
   {16990,67000 },
   {16360,68000 },
   {15760,69000 },
   {15180,70000 },
   {14630,71000 },
   {14100,72000 },
   {13600,73000 },
   {13110,74000 },
   {12640,75000 },
   {12190,76000 },
   {11760,77000 },
   {11350,78000 },
   {10960,79000 },
   {10580,80000 },
   {10210,81000 },
   {9859,82000 },
   {9522,83000 },
   {9198,84000 },
   {8887,85000 },
   {8587,86000 },
   {8299,87000 },
   {8022,88000 },
   {7756,89000 },
   {7500,90000 },
   {7254,91000 },
   {7016,92000 },
   {6788,93000 },
   {6568,94000 },
   {6357,95000 },
   {6153,96000 },
   {5957,97000 },
   {5768,98000 },
   {5586,99000 },
   {5410,100000 },
   {5241,101000 },
   {5078,102000 },
   {4921,103000 },
   {4769,104000 },
   {4623,105000 },
   {4482,106000 },
   {4346,107000 },
   {4215,108000 },
   {4088,109000 },
   {3966,110000 },
   {3848,111000 },
   {3734,112000 },
   {3624,113000 },
   {3518,114000 },
   {3415,115000 },
   {3316,116000 },
   {3220,117000 },
   {3128,118000 },
   {3038,119000 },
   {2952,120000 },
   {2868,121000 },
   {2787,122000 },
   {2709,123000 },
   {2634,124000 },
   {2561,125000 }
};

static void adc_tm_map_voltage_temp(const struct adc_tm_map_pt *pts,
				      size_t tablesize, int input, int *output)
{
	unsigned int descending = 1;
	u32 i = 0;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending) && (pts[i].x < input)) {
			/* table entry is less than measured*/
			 /* value and table is descending, stop */
			break;
		} else if ((!descending) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured*/
			/*value and table is ascending, stop */
			break;
		}
		i++;
	}

	if (i == 0) {
		*output = pts[0].y;
	} else if (i == tablesize) {
		*output = pts[tablesize - 1].y;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t)((pts[i].y - pts[i - 1].y) *
			(input - pts[i - 1].x)) /
			(pts[i].x - pts[i - 1].x)) +
			pts[i - 1].y);
	}
}

static void adc_tm_map_temp_voltage(const struct adc_tm_map_pt *pts,
		size_t tablesize, int input, int64_t *output)
{
	unsigned int i = 0, descending = 1;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if (descending && (pts[i].y < input)) {
			/*
			 * Table entry is less than measured value.
			 * Table is descending, stop.
			 */
			break;
		} else if (!descending && (pts[i].y > input)) {
			/*
			 * Table entry is greater than measured value.
			 * Table is ascending, stop.
			 */
			break;
		}
		i++;
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/*
		 * Result is between search_index and search_index-1.
		 * Interpolate linearly.
		 */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x) *
			(input - pts[i-1].y)) /
			(pts[i].y - pts[i-1].y)) +
			pts[i-1].x);
	}
}

/* Used by thermal clients to read ADC channel temperature*/
int adc_tm_get_temp_vadc(struct adc_tm_sensor *sensor, int *temp)
{
	int rc;

	if (!sensor || !sensor->adc)
		return -EINVAL;

	if (sensor->last_temp_set) {
		pr_debug("last_temp: %d\n", sensor->last_temp);
		sensor->last_temp_set = false;
		*temp = sensor->last_temp;
		return 0;
	}

	rc = iio_read_channel_processed(sensor->adc, temp);
	if (rc < 0)
		return rc;

	return 0;
}

int32_t adc_tm_read_reg(struct adc_tm_chip *chip,
					int16_t reg, u8 *data, int len)
{
	int ret;

	ret = regmap_bulk_read(chip->regmap, (chip->base + reg), data, len);
	if (ret < 0)
		pr_err("adc-tm read reg %d failed with %d\n", reg, ret);

	return ret;
}

int32_t adc_tm_write_reg(struct adc_tm_chip *chip,
					int16_t reg, u8 *data, int len)
{
	int ret;

	ret = regmap_bulk_write(chip->regmap, (chip->base + reg), data, len);
	if (ret < 0)
		pr_err("adc-tm write reg %d failed with %d\n", reg, ret);

	return ret;
}

int adc_tm_is_valid(struct adc_tm_chip *chip)
{
	struct adc_tm_chip *adc_tm_chip = NULL;

	list_for_each_entry(adc_tm_chip, chip->device_list, list)
		if (chip == adc_tm_chip)
			return 0;

	return -EINVAL;
}

int therm_fwd_scale(int64_t code, uint32_t adc_hc_vdd_ref_mv,
			const struct adc_tm_data *data)
{
	int64_t volt = 0;
	int result = 0;

	volt = (s64) code * adc_hc_vdd_ref_mv;
	volt = div64_s64(volt, (data->full_scale_code_volt));

	/* Same API can be used for resistance-temperature table */
	adc_tm_map_voltage_temp(adcmap_100k_104ef_104fb_1875_vref,
				 ARRAY_SIZE(adcmap_100k_104ef_104fb_1875_vref),
				 (int) volt, &result);

	return result;
}

int therm_fwd_scale_adc7(int64_t code)
{
	int64_t resistance = 0;
	int result = 0;

	if (code >= RATIO_MAX_ADC7)
		return -EINVAL;

	resistance = (s64) code * R_PU_100K;
	resistance = div64_s64(resistance, (RATIO_MAX_ADC7 - code));

	adc_tm_map_voltage_temp(adcmap_100k_adc7,
				 ARRAY_SIZE(adcmap_100k_adc7),
				 (int) resistance, &result);

	return result;
}

void adc_tm_scale_therm_voltage_100k(struct adc_tm_config *param,
				const struct adc_tm_data *data)
{
	int temp;

	/* High temperature maps to lower threshold voltage */
	adc_tm_map_temp_voltage(
		adcmap_100k_104ef_104fb_1875_vref,
		ARRAY_SIZE(adcmap_100k_104ef_104fb_1875_vref),
		param->high_thr_temp, &param->low_thr_voltage);

	param->low_thr_voltage *= data->full_scale_code_volt;
	param->low_thr_voltage = div64_s64(param->low_thr_voltage,
						ADC_HC_VDD_REF);

	temp = therm_fwd_scale(param->low_thr_voltage,
				ADC_HC_VDD_REF, data);

	if (temp < param->high_thr_temp)
		param->low_thr_voltage--;

	/* Low temperature maps to higher threshold voltage */
	adc_tm_map_temp_voltage(
		adcmap_100k_104ef_104fb_1875_vref,
		ARRAY_SIZE(adcmap_100k_104ef_104fb_1875_vref),
		param->low_thr_temp, &param->high_thr_voltage);

	param->high_thr_voltage *= data->full_scale_code_volt;
	param->high_thr_voltage = div64_s64(param->high_thr_voltage,
						ADC_HC_VDD_REF);

	temp = therm_fwd_scale(param->high_thr_voltage,
				ADC_HC_VDD_REF, data);

	if (temp > param->low_thr_temp)
		param->high_thr_voltage++;

}

void adc_tm_scale_therm_voltage_100k_adc7(struct adc_tm_config *param)
{
	int temp;
	int64_t resistance = 0;

	/* High temperature maps to lower threshold voltage */
	/* Same API can be used for resistance-temperature table */
	adc_tm_map_temp_voltage(
		adcmap_100k_adc7,
		ARRAY_SIZE(adcmap_100k_adc7),
		param->high_thr_temp, &resistance);

	param->low_thr_voltage = resistance * RATIO_MAX_ADC7;
	param->low_thr_voltage = div64_s64(param->low_thr_voltage,
						(resistance + R_PU_100K));

	temp = therm_fwd_scale_adc7(param->low_thr_voltage);
	if (temp == -EINVAL)
		return;

	if (temp < param->high_thr_temp)
		param->low_thr_voltage--;

	/* Low temperature maps to higher threshold voltage */
	/* Same API can be used for resistance-temperature table */
	adc_tm_map_temp_voltage(
		adcmap_100k_adc7,
		ARRAY_SIZE(adcmap_100k_adc7),
		param->low_thr_temp, &resistance);

	param->high_thr_voltage = resistance * RATIO_MAX_ADC7;
	param->high_thr_voltage = div64_s64(param->high_thr_voltage,
						(resistance + R_PU_100K));

	temp = therm_fwd_scale_adc7(param->high_thr_voltage);

	if (temp > param->low_thr_temp)
		param->high_thr_voltage++;
}

int32_t adc_tm_absolute_rthr(const struct adc_tm_data *data,
			struct adc_tm_config *tm_config)
{
	int64_t low_thr = 0, high_thr = 0;

	low_thr =  div_s64(tm_config->low_thr_voltage, tm_config->prescal);
	low_thr *= data->full_scale_code_volt;

	low_thr = div64_s64(low_thr, ADC_HC_VDD_REF);
	tm_config->low_thr_voltage = low_thr;

	high_thr =  div_s64(tm_config->high_thr_voltage, tm_config->prescal);
	high_thr *= data->full_scale_code_volt;

	high_thr = div64_s64(high_thr, ADC_HC_VDD_REF);
	tm_config->high_thr_voltage = high_thr;

	return 0;
}

int32_t adc_tm_absolute_rthr_adc7(struct adc_tm_config *tm_config)
{
	int64_t low_thr = 0, high_thr = 0;

	low_thr =  div_s64(tm_config->low_thr_voltage, tm_config->prescal);
	low_thr *= MAX_CODE_VOLT;

	low_thr = div64_s64(low_thr, ADC_HC_VDD_REF);
	tm_config->low_thr_voltage = low_thr;

	high_thr =  div_s64(tm_config->high_thr_voltage, tm_config->prescal);
	high_thr *= MAX_CODE_VOLT;

	high_thr = div64_s64(high_thr, ADC_HC_VDD_REF);
	tm_config->high_thr_voltage = high_thr;

	return 0;
}

MODULE_DESCRIPTION("Qualcomm Technologies Inc. PMIC ADC_TM common driver");
MODULE_LICENSE("GPL v2");
