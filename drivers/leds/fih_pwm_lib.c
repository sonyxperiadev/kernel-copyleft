/********************************************************************
@
@ Foxconn, Inc
@
@ 20120901, OwenHuang, Initial Version
@ 20121019, BennetJau, Modify for PM8038 support
@ 20130722, BennetJau, Modify for PM8226 support and referenc from QCT's QPNP_LED solution
@ 20130903, BennetJau, Solve blinking, fade in/out issues and add smooth solution.
********************************************************************/
#include <linux/fih_pwm_lib.h>

#define LED_PWM_PERIOD 1000
#define MAX_BRIGHTNESS 511
#define LUT_TABLE_SIZE 64
#define FADE_IN_OUT_DURATION 1000 //ms

#define PWM_ERR(fmt, args...) printk(KERN_ERR "[%s][ERR] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#define PWM_WAN(fmt, args...) printk(KERN_WARNING"[%s][WAN] PWM_LIB_LED : "fmt" \n", __func__, ##args)

//#define PWM_FIH_DEBUG
#ifdef PWM_FIH_DEBUG
#define DO_REG_DUMP
#define PWM_MSG(fmt, args...) printk(KERN_INFO"[%s][MSG] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#define PWM_DBG(fmt, args...) printk(KERN_INFO"[%s][DBG] PWM_LIB_LED : "fmt" \n", __func__, ##args)
#else
#define PWM_MSG(fmt, args...)
#define PWM_DBG(fmt, args...) 
#endif

#define Config_LED_PM8226_FIH //for PM8226

#ifdef Config_LED_PM8226_FIH
/*
reference 
80-NC833-2_A_PM8X26 POWER MANAGEMENT IC SOFTWARE INTERFACE.pdf
80-NC833-5_A_PM8X26 POWER MANAGEMENT IC DESIGN GUIDELINES - TRAINING SLIDES.pdf

For 8226 connection:
LPG4 -> DTEST4 -> MPP2 (RED) 
LPG1 -> DTEST1 -> MPP4 (Green) 
LPG6 -> DTEST2 -> MPP6 (BLUE) 

*/

#define MPP_MAX_LEVEL			LED_FULL
#define LED_MPP_MODE_CTRL(base)		(base + 0x40)
#define LED_MPP_VIN_CTRL(base)		(base + 0x41)
#define LED_MPP_EN_CTRL(base)		(base + 0x46)
#define LED_MPP_SINK_CTRL(base)		(base + 0x4C)

#define LED_MPP_CURRENT_DEFAULT		5
#define LED_MPP_CURRENT_PER_SETTING	5
#define LED_MPP_SOURCE_SEL_DEFAULT	LED_MPP_MODE_ENABLE
#define LED_MPP_VIN_CTRL_DEFAULT	0 //PERI-BJ-Do_MPP_INIT-00+

#define LED_MPP_VIN_MASK		0x03 //PERI-BJ-Do_MPP_INIT-00+
#define LED_MPP_SINK_MASK		0x07
#define LED_MPP_MODE_MASK		0x7F
#define LED_MPP_EN_MASK			0x80
#define LED_MPP_SRC_MASK		0x0F
#define LED_MPP_MODE_CTRL_MASK		0x70

#define LED_MPP_MODE_SINK		(0x06 << 4)
#define LED_MPP_MODE_ENABLE		0x01
#define LED_MPP_MODE_OUTPUT		0x10
#define LED_MPP_MODE_DISABLE		0x00
#define LED_MPP_EN_ENABLE		0x80
#define LED_MPP_EN_DISABLE		0x00

#define QPNP_LUT_RAMP_STEP_DEFAULT	255
#define QPNP_LED_PWM_FLAGS	(PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP)

#ifdef DO_REG_DUMP
static u8 mpp_debug_regs[] = {
	0x40, 0x41, 0x42, 0x45, 0x46, 0x4c,
};
#endif

//customerized define
//#define LED_MPP_SRC_RED_MASK	0x0F
//#define LED_MPP_SRC_GREEN_MASK	0x0F
//#define LED_MPP_SRC_BLUE_MASK	0x0F

#endif

static int local_pwm_lut_table_diff[3][PM_PWM_LUT_SIZE] = {{0},{0},{0}};

typedef int (* control_led)(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data);

typedef struct _led_functions {
	char *name;
	control_led led_actions[LED_ACTION_MAX];
} led_functions;

/*calculate brightness percentage for lut used*/
static int cal_brightness_percentage(int raw_brightness)
{
	return (raw_brightness * 100 / MAX_BRIGHTNESS);
}

/*calculate each ramp up size*/
static int cal_each_ramp_size(int size, int raw_brightness)
{
	return (raw_brightness * 100 / MAX_BRIGHTNESS) / size;
}

#ifdef Config_LED_PM8226_FIH
static int qpnp_led_masked_write(struct spmi_device* spmi_dev, u16 addr, u8 mask, u8 val)
{
	int rc;
	u8 reg;


	rc = spmi_ext_register_readl(spmi_dev->ctrl, spmi_dev->sid,
		addr, &reg, 1);

	if (rc) {
		dev_err(&spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n", addr, rc);
	}
	else
		PWM_DBG("read from addr=%x, rc(%d)\n", addr, rc);

	reg &= ~mask;
	reg |= val;

	rc = spmi_ext_register_writel(spmi_dev->ctrl, spmi_dev->sid,
		addr, &reg, 1);
	if (rc)
		dev_err(&spmi_dev->dev,
			"Unable to write to addr=%x, rc(%d)\n", addr, rc);
	else
		PWM_DBG("write to addr=%x, rc(%d)\n", addr, rc);

	return rc;
}

#ifdef DO_REG_DUMP
static void qpnp_dump_regs(struct spmi_device* spmi_dev, struct led_pmic_data *pmic_data, u8 regs[], u8 array_size)
{
	int i;
	u8 val;

	PWM_DBG("===== DEST LED register dump start =====\n");
	for (i = 0; i < array_size; i++) {
		spmi_ext_register_readl(spmi_dev->ctrl,
					spmi_dev->sid,
					pmic_data->base + regs[i],
					&val, sizeof(val));
		PWM_DBG("DEST: 0x%x = 0x%x\n",
					pmic_data->base + regs[i], val);
	}
	PWM_DBG("===== DEST LED register dump end =====\n");
}
#endif
#endif

/*Based on hardware type to configure related pin, eg PMIC_GPIO, MSM_GPIO, LPG, MPP...*/
static int configure_hw_pin(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, int on_off, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int val;
	
	if (pmic_data == NULL)
	{
		PWM_ERR("invalid pmic_data");
		return (-EIO);
	}
	
	switch(pmic_data->hardware)
	{

#ifdef Config_LED_PM8226_FIH
				/*Currently, these are not needed!*/
		case LED_HW_PMIC_MPP:

			if (on_off) {

				//Per QCT's comment:
				//	PM8226 PWM channel has two flip flops for enabling the output.
				//	So need to enable it twice in order for it to work
				pwm_enable(pwm_dev);
				pwm_enable(pwm_dev);

				val = (pmic_data->source_sel & LED_MPP_SRC_MASK) |
					(pmic_data->mode_ctrl & LED_MPP_MODE_CTRL_MASK);
				PWM_DBG("val=%d",val);	
				ret = qpnp_led_masked_write(spmi_dev,
					LED_MPP_MODE_CTRL(pmic_data->base), LED_MPP_MODE_MASK,
					val);
				if (ret) {
					PWM_DBG("Failed to write led mode reg\n");

					dev_err(&spmi_dev->dev,
							"Failed to write led mode reg\n");

					return ret;
				}

				ret = qpnp_led_masked_write(spmi_dev,
						LED_MPP_EN_CTRL(pmic_data->base), LED_MPP_EN_MASK,
						LED_MPP_EN_ENABLE);
				if (ret) {

					PWM_DBG("Failed to write led enable reg");

					dev_err(&spmi_dev->dev,
							"Failed to write led enable " \
							"reg\n");
					return ret;
				}

			} else {

				pwm_disable(pwm_dev);

				ret = qpnp_led_masked_write(spmi_dev,
							LED_MPP_MODE_CTRL(pmic_data->base),
							LED_MPP_MODE_MASK,
							LED_MPP_MODE_DISABLE);
				if (ret) {
					PWM_DBG("Failed to write led mode reg");

					dev_err(&spmi_dev->dev,
							"Failed to write led mode reg\n");
					return ret;
				}

				ret = qpnp_led_masked_write(spmi_dev,
							LED_MPP_EN_CTRL(pmic_data->base),
							LED_MPP_EN_MASK,
							LED_MPP_EN_DISABLE);
				if (ret) {

					PWM_DBG("Failed to write led enable reg");

					dev_err(&spmi_dev->dev,
							"Failed to write led enable reg\n");
					return ret;
				}

			}

#ifdef DO_REG_DUMP
			qpnp_dump_regs(spmi_dev, pmic_data, mpp_debug_regs, ARRAY_SIZE(mpp_debug_regs));
#endif
			break;

		case LED_HW_PMIC_GPIO:
		case LED_HW_MSM_GPIO:
		case LED_HW_PMIC_LPG:
#endif

		default:
			PWM_ERR("No such IO pin");
			break;

	}

	return ret;
}

static int pmic_gpio_led_on(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int duty_us = 0;
	int brightness = 0;

	/*Setup pwm config*/
	brightness = pmic_data->on_off_pwm;

	duty_us = (brightness * LED_PWM_PERIOD) / MAX_BRIGHTNESS;
	PWM_MSG("duty_us = %d", duty_us);

	ret = pwm_config_us(pwm_dev, duty_us, LED_PWM_PERIOD);
	if (ret < 0) {
		dev_err(&spmi_dev->dev, "Failed to " \
			"configure pwm for new values\n");
		return ret;
	}

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev, 1, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d ", ret);
	}

	return ret;
}

static int pmic_gpio_led_off(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev, 0, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d ", ret);
	}
	return ret;
}

//PERI-BJ-Correct_Blinking_behavior-00*{
static int pmic_gpio_led_blinking_on(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	struct lut_params	lut_params;
	struct pwm_period_config pwm_period;
	int * duty_pcts;
	int ret = 0;
	int selection = 0;

	if(pmic_data->color == COLOR_BLUE)
		selection = 2;
	else if(pmic_data->color == COLOR_GREEN)
		selection = 1;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;

	ret = pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return -EIO;
	}

	/*Setup lut*/
	lut_params.start_idx = selection * 2; //different color has different lut start idx
	lut_params.idx_len = 2;
	lut_params.lut_pause_hi = pmic_data->blinking_time1;
	lut_params.lut_pause_lo = pmic_data->blinking_time2;
	lut_params.ramp_step_ms = 50; //LED duty cycles change to set 0.22sec
	lut_params.flags = QPNP_LED_PWM_FLAGS | PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN;
	//flags = PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN;

	duty_pcts = devm_kzalloc(&spmi_dev->dev, sizeof(int) * lut_params.idx_len, GFP_KERNEL);

	if (!duty_pcts) {
		dev_err(&spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	/*Calculate Blink low and high level brightness percentage*/
	duty_pcts[0] = cal_brightness_percentage(pmic_data->blinking_pwm2); //PWM_ON_Value
	duty_pcts[1] = cal_brightness_percentage(pmic_data->blinking_pwm1); //PWM_OFF_Value

	PWM_DBG("lut_params.lut_pause_hi = %d, lut_params.lut_pause_lo = %d", lut_params.lut_pause_hi, lut_params.lut_pause_lo);
	PWM_DBG("duty_pcts[0] = %d, duty_pcts[1] = %d", duty_pcts[0], duty_pcts[1]);
	PWM_DBG("lut_params.start_idx = %d", lut_params.start_idx);
	PWM_DBG("lut_params.idx_len = %d", lut_params.idx_len);
	PWM_DBG("lut_params.ramp_step_ms = %d", lut_params.ramp_step_ms);
	PWM_DBG("lut_params.flags = %d", lut_params.flags);

	ret = pwm_lut_config(pwm_dev,
		LED_PWM_PERIOD, /* ignored by hardware */
		duty_pcts,
		lut_params);
	if (ret < 0) {
		dev_err(&spmi_dev->dev, "Failed to " \
			"configure pwm LUT\n");
		return ret;
	}
	
	PWM_DBG("configure_hw_pin");

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev, 1, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}
	
	return ret;
}
//PERI-BJ-Correct_Blinking_behavior-00*}

static int pmic_gpio_led_blinking_off(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev, 0, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	return ret;
}

//PERI-BJ-Avoid_pwm_size_exceed_limit-00*{
static void set_lut_table_index(struct led_pmic_data *pmic_data)
{
	//These value are from QCT's suggestion for the real time being approach the expected interval

	if (pmic_data->interval == 500 || pmic_data->interval == 1000){
		if(pmic_data->color == COLOR_RED){
			pmic_data->lut_table_start= 15;
			pmic_data->lut_table_end= 30;
		}

		if(pmic_data->color == COLOR_GREEN){
			pmic_data->lut_table_start= 31;
			pmic_data->lut_table_end= 46;
		}

		if(pmic_data->color == COLOR_BLUE){
			pmic_data->lut_table_start=47;
			pmic_data->lut_table_end=62;
		}
	}else if(pmic_data->interval == 1300){
		if(pmic_data->color == COLOR_RED){
			pmic_data->lut_table_start= 1;
			pmic_data->lut_table_end= 20;
		}

		if(pmic_data->color == COLOR_GREEN){ 
			pmic_data->lut_table_start= 21;
			pmic_data->lut_table_end= 40;
		}
		if(pmic_data->color == COLOR_BLUE){		
			pmic_data->lut_table_start=41;
			pmic_data->lut_table_end=60;
		}
	}
	else{//3000,1500,800, or others
		if(pmic_data->color == COLOR_RED){
			pmic_data->lut_table_start= 0;
			pmic_data->lut_table_end= 20;
		}	
		if(pmic_data->color == COLOR_GREEN){
			pmic_data->lut_table_start= 21;
			pmic_data->lut_table_end= 41;
		}

		if(pmic_data->color == COLOR_BLUE){
			pmic_data->lut_table_start=42;
			pmic_data->lut_table_end=62;
		}
	}
}
//PERI-BJ-Avoid_pwm_size_exceed_limit-00*}

static int pmic_gpio_led_fade_on(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int index = 0;
	int ramp_up_percentage_size = 0;
	int *duty_pcts;
	struct lut_params	lut_params;

	struct pwm_period_config pwm_period;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data->pwm_clock;
	pwm_period.pre_div = pmic_data->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data->pwm_div_exp;

	/*Setup PWM period*/
	ret = pwm_config_period(pwm_dev, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm period error, ret = %d", ret);
		return -EIO;
	}

//PERI-BJ-Allow_to_fade_in_out_ZERO-00*{
//need to do more check to avoid side effect or consider other method
#if 0
	if (pmic_data->fade_in_out_pwm == 0)
	{
		PWM_ERR("Please setup fade in out pwm value firstly!");
		return -EIO;
	}
#endif
//PERI-BJ-Allow_to_fade_in_out_ZERO-00*}


	/*Set lut table index value*/
	set_lut_table_index(pmic_data);

	lut_params.idx_len = pmic_data->lut_table_end - pmic_data->lut_table_start + 1;

	duty_pcts = devm_kzalloc(&spmi_dev->dev, sizeof(int) * lut_params.idx_len, GFP_KERNEL);

	if (!duty_pcts) {
		dev_err(&spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}


	/*Calculate lut table values*/
	ramp_up_percentage_size = cal_each_ramp_size(pmic_data->lut_table_end - pmic_data->lut_table_start - 1,
												 pmic_data->fade_in_out_pwm);

	duty_pcts[0] = 0;
	duty_pcts[pmic_data->lut_table_end - pmic_data->lut_table_start] = 
								cal_brightness_percentage(pmic_data->fade_in_out_pwm);

	lut_params.start_idx = pmic_data->lut_table_start;//PERI-BJ-Fade_on_Start_Index-00*
	lut_params.lut_pause_hi = pmic_data->blinking_time1;
	lut_params.lut_pause_lo = pmic_data->blinking_time2;
	lut_params.ramp_step_ms = QPNP_LUT_RAMP_STEP_DEFAULT;
	lut_params.flags = 0;

	PWM_DBG("lut_params->start_idx = %d", lut_params.start_idx);
	PWM_DBG("lut_params.lut_pause_hi = %d, lut_params.lut_pause_lo = %d", lut_params.lut_pause_hi, lut_params.lut_pause_lo);
	PWM_DBG("lut_params.idx_len = %d", lut_params.idx_len);
	PWM_DBG("lut_params.ramp_step_ms = %d", lut_params.ramp_step_ms);
	PWM_DBG("lut_params.flags = %d", lut_params.flags);
	PWM_DBG("duty_pcts[0], value = %d", duty_pcts[0]);

	for (index = 1; index < (pmic_data->lut_table_end - pmic_data->lut_table_start); index ++)
	{
		/* assign value to loop up table for fade in/out */
		duty_pcts[index] = index * ramp_up_percentage_size;

		/* if value is out of range, re-assign max value as our PWM control*/
		if (duty_pcts[index] > 100)
			duty_pcts[index]  = 100;

		PWM_DBG("duty_pcts[%d], value = %d", index, duty_pcts[index]);
	}


	PWM_DBG("duty_pcts[%d], value = %d", pmic_data->lut_table_end - pmic_data->lut_table_start,
								duty_pcts[pmic_data->lut_table_end - pmic_data->lut_table_start]);

	/*Setup flags*/
	if (pmic_data->ramp_loop)
	{	
		lut_params.flags |= PM_PWM_LUT_LOOP;
	}

	if (pmic_data->ramp_up_down)
	{
		lut_params.flags |= PM_PWM_LUT_RAMP_UP;
	}

	if (pmic_data->toggle_up_down)
	{
		lut_params.flags |= PM_PWM_LUT_REVERSE;
	}

	lut_params.flags |= (PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN);

	PWM_DBG("PWM control flags = 0x%02X ", lut_params.flags);

	/*Set ramp_step_ms according to interval value*/
	if (pmic_data->interval == 3000){
		lut_params.ramp_step_ms = 128;
	}else if(pmic_data->interval == 1500 || pmic_data->interval == 1300 || pmic_data->interval == 1000){
		lut_params.ramp_step_ms = 64;
	}else if(pmic_data->interval == 800){
		lut_params.ramp_step_ms = 36;
	}
	else if(pmic_data->interval == 500){
		lut_params.ramp_step_ms = 32;
	}
	else
	{
		/*Calcuate interval of fade-in or fade-out*/
		//interval value not list in QCT's spec may cause the timing not accurate
		lut_params.ramp_step_ms = pmic_data->interval / (pmic_data->lut_table_end - pmic_data->lut_table_start + 1);
	}

	PWM_DBG("PWM control lut_params.ramp_step_ms = %d ", lut_params.ramp_step_ms);

	/*Config lut*/	
	ret = pwm_lut_config(pwm_dev,
		LED_PWM_PERIOD, /* ignored by hardware */
		duty_pcts,
		lut_params);
	if (ret < 0) {
		dev_err(&spmi_dev->dev, "Failed to " \
			"configure pwm LUT\n");
		return ret;
	}

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev, 1, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}

	return ret;
}

//PERI-BJ-Fade_in_out_diff_time-00+{
static void set_lut_params(struct lut_params* lut_params, struct led_pmic_data * pmic_data)
{

	lut_params->idx_len = pmic_data->lut_table_end - pmic_data->lut_table_start + 1;

	lut_params->start_idx = pmic_data->lut_table_start;
	lut_params->lut_pause_hi = pmic_data->blinking_time1;
	lut_params->lut_pause_lo = pmic_data->blinking_time2;
	lut_params->ramp_step_ms = QPNP_LUT_RAMP_STEP_DEFAULT;
	lut_params->flags = 0;

	PWM_DBG("lut_params->start_idx = %d", lut_params->start_idx);
	PWM_DBG("lut_params->lut_pause_hi = %d, lut_params->lut_pause_lo = %d", lut_params->lut_pause_hi, lut_params->lut_pause_lo);
	PWM_DBG("lut_params->idx_len = %d", lut_params->idx_len);
	PWM_DBG("lut_params->ramp_step_ms = %d", lut_params->ramp_step_ms);
	PWM_DBG("lut_params->flags = %d", lut_params->flags);

	/*Setup flags*/
	if (pmic_data->ramp_loop)
	{	
		lut_params->flags |= PM_PWM_LUT_LOOP;
	}

	if (pmic_data->ramp_up_down)
	{
		lut_params->flags |= PM_PWM_LUT_RAMP_UP;
	}

	if (pmic_data->toggle_up_down)
	{
		lut_params->flags |= PM_PWM_LUT_REVERSE;
	}

	lut_params->flags |= (PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN);

	PWM_DBG("PWM control flags = 0x%02X ", lut_params->flags);

	/*Set ramp_step_ms according to interval value*/
	if (pmic_data->interval == 3000){
		lut_params->ramp_step_ms = 128;
	}else if(pmic_data->interval == 1500 || pmic_data->interval == 1300 || pmic_data->interval == 1000){
		lut_params->ramp_step_ms = 64;
	}else if(pmic_data->interval == 800){
		lut_params->ramp_step_ms = 36;
	}
	else if(pmic_data->interval == 500){
		lut_params->ramp_step_ms = 32;
	}
	else
	{
		/*Calcuate interval of fade-in or fade-out*/
		//interval value not list in QCT's spec may cause the timing not accurate
		lut_params->ramp_step_ms = pmic_data->interval / (pmic_data->lut_table_end - pmic_data->lut_table_start + 1);
	}

	PWM_DBG("PWM control lut_params.ramp_step_ms = %d ", lut_params->ramp_step_ms);


}

static void cal_component_val(int size, int ramp_up_percentage_size, int raw_percent, int *cX, int *cY)
{
	PWM_DBG("raw_percent = %d, size = %d, ramp_up_percentage_size = %d ", raw_percent, size, ramp_up_percentage_size);

	//special case:raw_percent - size <= 0
	if(raw_percent - size <= 0){
		*cX = raw_percent;
		*cY = size - raw_percent;
		return;
	}

	//ramp_up_percentage_size can't equal or less than 0
	if(ramp_up_percentage_size <= 0){
		PWM_ERR("ramp_up_percentage_size <= 0");
		return;
	}

	if(raw_percent == size * ramp_up_percentage_size){
		//special case:raw_percent == size * ramp_up_percentage_size

		if(ramp_up_percentage_size - 1 == 0){
			//Here, ramp_up_percentage_size - 1 == 0 is imposible
			PWM_ERR("ramp_up_percentage_size - 1 == 0");
			return;
		}

		*cY = (raw_percent - size) / (ramp_up_percentage_size - 1);
	}
	else
		*cY = (raw_percent - size) / ramp_up_percentage_size;

	*cX = size - *cY;

	return;
}

static void set_lut_table(struct led_pmic_data* pmic_data, struct lut_params* lut_params, int selection)
{
	int ramp_up_percentage_size = 0;
	int raw_percent = 0;
	int index = 0;
	int index_cX = 0;
	int lut_table_length = 0;

	//cX * (n * cY)
	int cX = 0;
	int cY = 0;

	/*Set lut table index value*/
	set_lut_table_index(pmic_data);

	lut_table_length = pmic_data->lut_table_end - pmic_data->lut_table_start + 1;

	/*Calculate lut table values*/
	ramp_up_percentage_size = cal_each_ramp_size(lut_table_length - 2, pmic_data->fade_in_out_pwm);
	raw_percent = cal_brightness_percentage(pmic_data->fade_in_out_pwm);
	PWM_DBG("pmic_data->fade_in_out_pwm=%d, ramp_up_percentage_size = %d, raw_percent =%d", 
			pmic_data->fade_in_out_pwm, ramp_up_percentage_size, raw_percent);

	cal_component_val(lut_table_length - 2, ramp_up_percentage_size, raw_percent, &cX, &cY);
	PWM_DBG("cX = %d, cY =%d", cX, cY);


	local_pwm_lut_table_diff[selection][0] = 0;
	local_pwm_lut_table_diff[selection][lut_table_length - 1] = raw_percent;

	set_lut_params((lut_params+selection), pmic_data);

	if(ramp_up_percentage_size <= 0 && pmic_data->fade_in_out_pwm > 0){
		ramp_up_percentage_size = 0;
	}
	else
	{
		if(raw_percent != (lut_table_length - 2) * ramp_up_percentage_size)
			ramp_up_percentage_size+=1;
	}

	for (index = 1; index <= cY; index ++)
	{
		/* assign value to loop up table for fade in/out */
		local_pwm_lut_table_diff[selection][index] = index * ramp_up_percentage_size;

		/* if value is out of range, re-assign max value as our PWM control*/
		if (local_pwm_lut_table_diff[selection][index] > 100)
			local_pwm_lut_table_diff[selection][index]	= 100;

		PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

		/*Check if we need to invert*/
		if (pmic_data->invert)
		{
			local_pwm_lut_table_diff[selection][index] = 100 -	local_pwm_lut_table_diff[selection][index];
			PWM_DBG("invert local_pwm_lut_table_diff[selection][%d] = %d", index, local_pwm_lut_table_diff[selection][index]);
		}
	}

	for (index_cX = 1; index_cX <= cX && index <= lut_table_length - 1 ; index_cX ++, index++)
	{
		/* assign value to loop up table for fade in/out */
		local_pwm_lut_table_diff[selection][index] = cY * ramp_up_percentage_size + index_cX * ((pmic_data->fade_in_out_pwm == 0)? 0: 1);

		/* if value is out of range, re-assign max value as our PWM control*/
		if (local_pwm_lut_table_diff[selection][index] > 100)
			local_pwm_lut_table_diff[selection][index] = 100;

		PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

		/*Check if we need to invert*/
		if (pmic_data->invert)
		{
			local_pwm_lut_table_diff[selection][index] = 100 -	local_pwm_lut_table_diff[selection][index];
			PWM_DBG("invert local_pwm_lut_table_diff[selection][%d] = %d", index, local_pwm_lut_table_diff[selection][index]);
		}
	}

	PWM_DBG("local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

	//make sure the last lut table item would have the max value
	if(local_pwm_lut_table_diff[selection][index] < local_pwm_lut_table_diff[selection][index-1])
		local_pwm_lut_table_diff[selection][index] = local_pwm_lut_table_diff[selection][index-1];

	PWM_DBG("modified local_pwm_lut_table_diff[selection][%d], value = %d", index, local_pwm_lut_table_diff[selection][index]);

}

static int pmic_gpio_led_fade_on_diff(struct spmi_device* spmi_dev, struct led_diff_data* diff_data)
{
	int ret = 0;
	struct lut_params lut_params[3];
	struct pwm_period_config pwm_period;

	struct pwm_device *pwm_dev_r = diff_data->pwm_dev_r;
	struct pwm_device *pwm_dev_g = diff_data->pwm_dev_g;
	struct pwm_device *pwm_dev_b = diff_data->pwm_dev_b;

	struct led_pmic_data *pmic_data_r = diff_data->pmic_data_r;
	struct led_pmic_data *pmic_data_g = diff_data->pmic_data_g;
	struct led_pmic_data *pmic_data_b = diff_data->pmic_data_b;

	/*Setup PWM period*/
	pwm_period.pwm_size = PM_PWM_SIZE_9BIT;
	pwm_period.clk = pmic_data_r->pwm_clock;
	pwm_period.pre_div = pmic_data_r->pwm_div_value;
	pwm_period.pre_div_exp = pmic_data_r->pwm_div_exp;

	/*Setup PWM period*/
	ret = pwm_config_period(pwm_dev_r, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm r period error, ret = %d", ret);
		return -EIO;
	}

	ret = pwm_config_period(pwm_dev_g, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm g period error, ret = %d", ret);
		return -EIO;
	}

	ret = pwm_config_period(pwm_dev_b, &pwm_period);
	if (ret)
	{
		PWM_ERR("config pwm b period error, ret = %d", ret);
		return -EIO;
	}

	/*set lut table*/
	set_lut_table(pmic_data_r, lut_params, 0);
	set_lut_table(pmic_data_g, lut_params, 1);
	set_lut_table(pmic_data_b, lut_params, 2);

	/*Config lut*/	
	ret = pwm_lut_config(pwm_dev_r,
		LED_PWM_PERIOD, /* ignored by hardware */
		local_pwm_lut_table_diff[0],
		lut_params[0]);

	if (ret < 0) {
		//dev_err(&spmi_dev->dev, "Failed to configure pwm LUT\n");
		return ret;
	}

	ret = pwm_lut_config(pwm_dev_g,
		LED_PWM_PERIOD, /* ignored by hardware */
		local_pwm_lut_table_diff[1],
		lut_params[1]);

	if (ret < 0) {
		//dev_err(&spmi_dev->dev, "Failed to configure pwm LUT\n");
		return ret;
	}

	ret = pwm_lut_config(pwm_dev_b,
		LED_PWM_PERIOD, /* ignored by hardware */
		local_pwm_lut_table_diff[2],
		lut_params[2]);

	if (ret < 0) {
		//dev_err(&spmi_dev->dev, "Failed to configure pwm LUT\n");
		return ret;
	}

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev_r, 1, pmic_data_r);
	if (ret)
	{
		PWM_ERR("config r IO error, ret = %d", ret);
	}

	ret = configure_hw_pin(spmi_dev, pwm_dev_g, 1, pmic_data_g);
	if (ret)
	{
		PWM_ERR("config g IO error, ret = %d", ret);
	}

	ret = configure_hw_pin(spmi_dev, pwm_dev_b, 1, pmic_data_b);
	if (ret)
	{
		PWM_ERR("config b IO error, ret = %d", ret);
	}

	return ret;
}
//PERI-BJ-Fade_in_out_diff_time-00+}

static int pmic_gpio_led_fade_off(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;

	/*Configure hw related pin*/
	ret = configure_hw_pin(spmi_dev, pwm_dev, 0, pmic_data);
	if (ret)
	{
		PWM_ERR("config IO error, ret = %d", ret);
	}
	
	return 0;
}

/*leds control function collection*/
static led_functions led_functions_collection[TYPE_MAX] = {

#ifdef Config_LED_PM8226_FIH
[TYPE_PMIC_GPIO] = {
		.name = "LED_PMIC_GPIO",
		.led_actions = {NULL, NULL, NULL, NULL, NULL, NULL},
},//PMIC GPIO

[TYPE_PMIC_MPP] = {
		.name = "LED_PMIC_MPP",
		.led_actions = {pmic_gpio_led_on, pmic_gpio_led_off, 
						pmic_gpio_led_blinking_on, pmic_gpio_led_blinking_off, 
						pmic_gpio_led_fade_on, pmic_gpio_led_fade_off},
},//PMIC MPP
#endif

[TYPE_GPIO] = {
		.name = "LED_GPIO",
		.led_actions = {NULL, NULL, NULL, NULL, NULL, NULL},
}, //GPIO

};

//PERI-BJ-Do_MPP_INIT-00+{
static int mpp_init(struct spmi_device* spmi_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int val;
	
	if (pmic_data->hardware == LED_HW_PMIC_MPP)
	{
		/*mpp init */
		val = pmic_data->current_sink;
		
		if (val < 0)
			val = 0;

		PWM_DBG("mpp init val = %d", val);

		ret = qpnp_led_masked_write(spmi_dev, LED_MPP_VIN_CTRL(pmic_data->base), 
			LED_MPP_VIN_MASK, LED_MPP_VIN_CTRL_DEFAULT);
		if (ret) {
			dev_err(&spmi_dev->dev,
				"Failed to write led vin control reg\n");
			return ret;
		}

		ret = qpnp_led_masked_write(spmi_dev, LED_MPP_SINK_CTRL(pmic_data->base), LED_MPP_SINK_MASK, val);
		if (ret) {
			dev_err(&spmi_dev->dev,
				"Failed to write led enable reg\n");
			return ret;
		}

	}
	return ret;
}
//PERI-BJ-Do_MPP_INIT-00+}

/*Function for led driver use to control LEDs*/
int common_control_leds(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data)
{
	int ret = 0;
	int index = 0;

	led_functions this_led_functions;

	if (pwm_dev == NULL || pmic_data == NULL)
	{
		PWM_ERR("NULL data type");
		return -EIO;
	}

	PWM_DBG("pmic_data->hardware = %d", pmic_data->hardware);

	/*If hardware type is valid, determine led control funcionts*/
	if (LED_HW_PMIC_GPIO <= pmic_data->hardware && pmic_data->hardware <= LED_HW_PMIC_LPG)
	{
		this_led_functions = led_functions_collection[pmic_data->hardware];
		PWM_DBG("we got led function hw type: %s", this_led_functions.name);
	}
	else
	{
		PWM_ERR("invalid hardware type");
		return -EIO;
	}

	PWM_DBG("pmic_data->command = %d", pmic_data->command);

	/*Parse which action (fade/blinking/on)*/
	if (pmic_data->command >= SMEM_CMD_LED_ON_OFF && pmic_data->command <= SMEM_CMD_LED_FADE_IN_OUT)
	{
		if (pmic_data->control)
		{
			index = 2 * pmic_data->command;
		}
		else
		{
			index = 2 * pmic_data->command + 1;
		}
	}

	PWM_MSG("index = %d ", index);

//PERI-BJ-Do_MPP_INIT-00+{
#ifdef Config_LED_PM8226_FIH
	if(!(index%2)){
		ret = mpp_init(spmi_dev, pmic_data);
		if (ret) {
			dev_err(&spmi_dev->dev,
				"Failed to do mpp init\n");
			return ret;
		}
	}
#endif
//PERI-BJ-Do_MPP_INIT-00+}

	/*Check action index valid or not*/
	if (index >= LED_ACTION_MAX)
	{
		PWM_ERR("out of action range, index = %d", index);
		return -EIO;
	}
	else
	{
		if (this_led_functions.led_actions[index] != NULL)
		{
			//call mapping function to control led
			ret = (*this_led_functions.led_actions[index])(spmi_dev, pwm_dev, pmic_data);
		}
		else
		{
			PWM_ERR("the function is not support now!");
			ret = 0;
		}
	}

	return ret;	
}
EXPORT_SYMBOL_GPL(common_control_leds);

//PERI-BJ-Fade_in_out_diff_time-00+{
int control_fade_nonsync_leds(struct spmi_device* spmi_dev, struct led_diff_data* diff_data)
{

	struct pwm_device *pwm_dev_r = diff_data->pwm_dev_r;
	struct pwm_device *pwm_dev_g = diff_data->pwm_dev_g;
	struct pwm_device *pwm_dev_b = diff_data->pwm_dev_b;

	struct led_pmic_data *pmic_data_r = diff_data->pmic_data_r;
	struct led_pmic_data *pmic_data_g = diff_data->pmic_data_g;
	struct led_pmic_data *pmic_data_b = diff_data->pmic_data_b;

	int ret = 0;

	if (pwm_dev_r == NULL || pmic_data_r == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	if (pwm_dev_g == NULL || pmic_data_g == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	if (pwm_dev_b == NULL || pmic_data_b == NULL)
	{
		PWM_ERR("%s, NULL data type", __func__);
		return -EIO;
	}

	/*Parse which action (fade/blinking/on)*/
	if (pmic_data_r->control || pmic_data_g->control || pmic_data_b->control)
	{//on

		PWM_MSG("control_fade_nonsync_leds:ON");

//PERI-BJ-Do_MPP_INIT-00+{
#ifdef Config_LED_PM8226_FIH
		/*mpp init */
		ret = mpp_init(spmi_dev, pmic_data_r);
		if (ret) {
			dev_err(&spmi_dev->dev,
				"Failed to do mpp init\n");
			return ret;
		}

		ret = mpp_init(spmi_dev, pmic_data_g);
		if (ret) {
			dev_err(&spmi_dev->dev,
				"Failed to do mpp init\n");
			return ret;
		}

		ret = mpp_init(spmi_dev, pmic_data_b);
		if (ret) {
			dev_err(&spmi_dev->dev,
				"Failed to do mpp init\n");
			return ret;
		}
#endif
//PERI-BJ-Do_MPP_INIT-00+}

		ret = pmic_gpio_led_fade_on_diff(spmi_dev, diff_data);
		
		if (ret)
		{
			PWM_ERR("pmic_gpio_led_fade_on_diff error! ret = %d !", ret);
			return ret;
		}
	}
	else
	{//off
		PWM_MSG("control_fade_nonsync_leds:OFF");
		ret = pmic_gpio_led_fade_off(spmi_dev, pwm_dev_r, pmic_data_r);
		if (ret)
		{
			PWM_ERR("disable r lut channel error! ret = %d !", ret);
			return ret;
		}

		ret = pmic_gpio_led_fade_off(spmi_dev, pwm_dev_g, pmic_data_g);

		if (ret)
		{
			PWM_ERR("disable g lut channel error! ret = %d !", ret);
			return ret;
		}


		ret = pmic_gpio_led_fade_off(spmi_dev, pwm_dev_b, pmic_data_b);
		if (ret)
		{
			PWM_ERR("disable b lut channel error! ret = %d !", ret);
			return ret;
		}

	}

	return ret;	
}
EXPORT_SYMBOL_GPL(control_fade_nonsync_leds);
//PERI-BJ-Fade_in_out_diff_time-00+}

