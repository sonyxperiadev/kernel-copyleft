#include <linux/fih_pwm_lib.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

//PERI-BJ-LED_DEBUG_MSG-00*{
#define LED_ERR(fmt, args...) printk(KERN_ERR "[%s][ERR] Driver_LED : "fmt" \n", __func__, ##args)
#define LED_WAN(fmt, args...) printk(KERN_WARNING"[%s][WAN] Driver_LED : "fmt" \n", __func__, ##args)

#define DEBUG_LED
#ifdef DEBUG_LED
#define LED_MSG(fmt, args...) printk(KERN_INFO "[%s][MSG] Driver_LED : "fmt" \n", __func__, ##args)
#define LED_DBG_DIFF(fmt, args...) //printk(KERN_INFO "[%s][DBG_F] Driver_LED : "fmt" \n", __func__, ##args)//PERI-BJ-DISABLE_LOG_FADE_diff-00-
#define LED_DBG(fmt, args...) printk(KERN_INFO"[%s][DBG] Driver_LED : "fmt" \n", __func__, ##args)
#else
#define LED_MSG(fmt, args...)
#define LED_DBG_DIFF(fmt, args...)
#define LED_DBG(fmt, args...)
#endif
//PERI-BJ-LED_DEBUG_MSG-00*}

#ifdef DEBUG_LED
//currently used in log message
static const char			*select_func[]		= { "GPIO", "MPP", "SYS_GPIO", "LUT" };
#endif
static const char			*current_sink_table[]	= { "5mA", "10mA", "15mA", "20mA", "25mA", "30mA", "35mA", "40mA" };
static const char			*current_rgb_table[]	= { "0mA", "4mA", "8mA", "12mA"}; //PERI-BJ-SetCurrent-00+

static struct control_class_device	*file_node_class;
static struct led_data		*leds_data;
static struct led_command_data	*command_data;
static struct led_command_info	command_info;
static struct workqueue_struct	*work_queue;

//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*{
static struct workqueue_struct	*work_queue_diff;//for led fadin/out different timer.
static unsigned int		timeout_buf=0;//PERI-BJ-Fade_in_out_diff_time-00*
static unsigned int		fade_off_flag=0;

//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*}

//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
//If need to Use sw control not hw control feature, enable the feature
//#define LED_SW_CONTROL_SUPPORT
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}


//QPNP settings
#define LED_MPP_SOURCE_SEL_DEFAULT	LED_MPP_MODE_ENABLE
#define LED_MPP_MODE_SINK			(0x06 << 4)
#define LED_MPP_MODE_ENABLE			0x01
#define LED_DEFAULT_BASE_ADDR		0xa100

#define LED_DEFAULT_PWM_CLOCK 		PM_PWM_CLK_19P2MHZ
#define LED_DEFAULT_PWM_DIV_VALUE	PM_PWM_PDIV_6
#define LED_DEFAULT_PWM_DIV_EXP		3

enum led_brightness {
	LED_OFF		= 0,
	LED_HALF	= 255,//127,
	LED_FULL	= 510,//255,
};

enum led_mode {
	PWM_MODE = 0,
	LPG_MODE,
	MANUAL_MODE,
};

#if 0
/**
 *  mpp_config_data - mpp configuration data
 *  @pwm_cfg - device pwm configuration
 *  @current_setting - current setting, 5ma-40ma in 5ma increments
 *  @source_sel - source selection
 *  @mode_ctrl - mode control
 *  @pwm_mode - pwm mode in use
 */
struct mpp_config_data {
	struct pwm_config_data	*pwm_cfg;
	u8	current_setting;
	u8	source_sel;
	u8	mode_ctrl;
	u8 pwm_mode;
};

/**
 *  pwm_config_data - pwm configuration data
 *  @lut_params - lut parameters to be used by pwm driver
 *  @pwm_device - pwm device
 *  @pwm_channel - pwm channel to be configured for led
 *  @pwm_period_us - period for pwm, in us
 *  @mode - mode the led operates in
 */
struct pwm_config_data {
	struct lut_params	lut_params;
	struct pwm_device	*pwm_dev;
	int			pwm_channel;
	u32			pwm_period_us;
	struct pwm_duty_cycles	*duty_cycles;
	u8	mode;
	u8	enable;
};
#endif

//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*{
enum {
	DISABLE,
	ENABLE,
};

enum {
	FADE_STATE_IN,
	FADE_STATE_OUT,
};
//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*}

//extern bool is_power_off_charging(void);//PERI-BJ-Modify_Power_Off_Led_Init-00+

static void	led_on_off_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, parameter->para1 == LED_TURN_OFF ? msm_data->led_off_level : msm_data->led_on_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) %s", msm_data->msm_pin, data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= parameter->para1;

			common_control_leds(data->spmi_dev, data->pwm_dev, pmic_data);

			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s) %s, [Pin:LPG]=[%d:%d]", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_blinking_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support blinking", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_BLINKING;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= parameter->para1;

			common_control_leds(data->spmi_dev, data->pwm_dev, pmic_data);
			
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s) blink %s, [Pin:LPG]=[%d:%d]", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_fade_in_out_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support fade in/out", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_FADE_IN_OUT;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= parameter->para1;

			common_control_leds(data->spmi_dev, data->pwm_dev, pmic_data);
			
			mutex_unlock( &data->lock );
			LED_DBG_DIFF( "PMIC %s led(%s) fade in/out %s, [Pin:LPG]=[%d:%d]", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

//Use single command to lit R,G,B 
//PERI-BJ-Fade_in_out_once-00*{
static void	led_fade_in_out_once_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support fade in/out once", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );

			pmic_data->interval = (parameter->para2 > 500) ? parameter->para2 : 500;
			pmic_data->toggle_up_down = DISABLE;
			pmic_data->ramp_loop = DISABLE;
			pmic_data->ramp_up_down = parameter->para3;

			mutex_unlock( &data->lock );

			led_fade_in_out_set(data, parameter );

			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}


#if 0
	struct led_data 	*destination_r	= leds_data + 2;
	struct led_data 	*destination_g	= leds_data + 1;
	struct led_data 	*destination_b	= leds_data + 0;

	struct led_pmic_data	*pmic_data_r	= &destination_r->detail.pmic_data;
	struct led_pmic_data	*pmic_data_g	= &destination_g->detail.pmic_data;
	struct led_pmic_data	*pmic_data_b	= &destination_b->detail.pmic_data;

	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support fade in/out", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			mutex_lock( &destination_r->lock );
			//R
			pmic_data_r->control	= parameter->para1;
			pmic_data_r->interval = (parameter->para2 > 500) ? parameter->para2 : 500;
			pmic_data_r->toggle_up_down = DISABLE;
			pmic_data_r->ramp_loop = DISABLE;
			pmic_data_r->ramp_up_down = parameter->para3;
			pmic_data_r->command	= SMEM_CMD_LED_FADE_IN_OUT;
			pmic_data_r->hardware = data->use_hw;

			//G
			pmic_data_g->control	= parameter->para1;
			pmic_data_g->interval = (parameter->para2 > 500) ? parameter->para2 : 500;
			pmic_data_g->toggle_up_down = DISABLE;
			pmic_data_g->ramp_loop = DISABLE;
			pmic_data_g->ramp_up_down = parameter->para3;
			pmic_data_g->command	= SMEM_CMD_LED_FADE_IN_OUT;
			pmic_data_g->hardware = data->use_hw;

			//B
			pmic_data_b->control	= parameter->para1;
			pmic_data_b->interval = (parameter->para2 > 500) ? parameter->para2 : 500;
			pmic_data_b->toggle_up_down = DISABLE;
			pmic_data_b->ramp_loop = DISABLE;
			pmic_data_b->ramp_up_down = parameter->para3;
			pmic_data_b->command	= SMEM_CMD_LED_FADE_IN_OUT;
			pmic_data_b->hardware = data->use_hw;
			mutex_unlock( &destination_r->lock );

			//Currently not support
			//control_fade_nonsync_leds(destination_r->pwm_dev, pmic_data_r, destination_g->pwm_dev, pmic_data_g, destination_b->pwm_dev, pmic_data_b);
			LED_MSG("led_fade_in_out_once_set not support");

			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
#endif
}
//PERI-BJ-Fade_in_out_once-00*}

static void	led_on_off_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_on_off_set( data, parameter );
}

static void	led_blinking_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_blinking_set( data, parameter );
}

static void	led_fade_in_out_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_fade_in_out_set( data, parameter );
}

static void	led_sw_blinking_set( struct led_data *data, struct command_parameter *parameter )
{
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT

	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support blinking", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			hrtimer_cancel( &data->led_time );

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= LED_TURN_OFF;

			LED_MSG("led_sw_blinking_set not support");
			//common_control_leds(data->pwm_dev, pmic_data);

			if( parameter->para1 != LED_TURN_OFF )
			{
				data->led_state		= LED_OFF_STATE;
				data->unit_time_out	= 0;
				hrtimer_start( &data->led_time, ktime_set( LED_SEC( pmic_data->blinking_time1 ), LED_NS( pmic_data->blinking_time1 ) ), HRTIMER_MODE_REL );
			}

			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s) blink %s, [Pin:LPG]=[%d:%d].", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}

}

static void	led_sw_fade_in_out_set( struct led_data *data, struct command_parameter *parameter )
{
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT

	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support fade in/out", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			hrtimer_cancel( &data->led_time );

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= LED_TURN_OFF;

			LED_DBG("led_sw_fade_in_out_set not support");
			//common_control_leds(data->pwm_dev, pmic_data);

			LED_DBG("parameter->para1=%d", parameter->para1);


			if( parameter->para1 != LED_TURN_OFF )
			{
				data->led_state		= LED_OFF_STATE;
				data->sw_brightness	= 0;
				data->unit_brightness	= pmic_data->fade_in_out_pwm * MINI_UNIT / FADE_IN_OUT_LEVEL;
				data->unit_time_out	= pmic_data->interval;
				data->sw_led_count	= FADE_IN_OUT_LEVEL;
				LED_DBG("hrtimer_start");

				hrtimer_start( &data->led_time, ktime_set( LED_SEC( pmic_data->blinking_time2 ), LED_NS( pmic_data->blinking_time2 ) ), HRTIMER_MODE_REL );
			}

			mutex_unlock( &data->lock );
			LED_MSG( "%s led(%s) fade in/out %s, [Pin:LPG]=[%d:%d].", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}

}

static void	led_sw_blinking_check_mode( struct led_data *data, struct command_parameter *parameter )
{
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_sw_blinking_set( data, parameter );
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}
}

static void	led_sw_fade_in_out_check_mode( struct led_data *data, struct command_parameter *parameter )
{
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_sw_fade_in_out_set( data, parameter );
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}
}

static void	led_on_brightness_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{

			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set brightness", msm_data->msm_pin, data->name );

			break;
		}
#endif

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->on_off_pwm	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), set PWM(%d)", *( select_func + data->use_hw ), data->name, pmic_data->on_off_pwm );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_blinking_brightness_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set brightness", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->blinking_pwm1	= parameter->para1; //lo
			pmic_data->blinking_pwm2	= parameter->para2; //hi
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), set blinking PWM[%d:%d]", *( select_func + data->use_hw ), data->name, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2 );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_fade_in_out_interval_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set interval", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->interval	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), set intervale(%d)", *( select_func + data->use_hw ), data->name, pmic_data->interval );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_blinking_time_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set blinking time", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->blinking_time1	= parameter->para1;
			pmic_data->blinking_time2	= parameter->para2;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), set blinking time[%d:%d]", *( select_func + data->use_hw ), data->name, pmic_data->blinking_time1, pmic_data->blinking_time2 );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_mode_set( struct led_data *data, struct command_parameter *parameter )
{
	mutex_lock( &data->lock );
	data->special_mode	= parameter->para1;
	mutex_unlock( &data->lock );

	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s), %s mode", msm_data->msm_pin, data->name, data->special_mode ? "Special" : "Normal" );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			LED_MSG( "PMIC %s led(%s), %s mode", *( select_func + data->use_hw ), data->name, data->special_mode ? "Special" : "Normal" );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_clk_div_exp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set clk, div & exp", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_clock	= parameter->para1;
			pmic_data->pwm_div_value	= parameter->para2;
			pmic_data->pwm_div_exp	= parameter->para3;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), [CLK:DIV:EXP] = [%d:%d:%d]", *( select_func + data->use_hw ), data->name, pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_clk( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set clk", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_clock	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), CLK(%d)", *( select_func + data->use_hw ), data->name, pmic_data->pwm_clock );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_div( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set div", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_div_value	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), DIV(%d)", *( select_func + data->use_hw ), data->name, pmic_data->pwm_div_value );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_exp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set exp", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_div_exp	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), EXP(%d)", *( select_func + data->use_hw ), data->name, pmic_data->pwm_div_exp );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_toggle_loop_ramp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set toggle, loop & ramp", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->toggle_up_down	= parameter->para1;
			pmic_data->ramp_loop	= parameter->para2;
			pmic_data->ramp_up_down	= parameter->para3;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), [Toggle:Loop:Ramp]=[%s:%s:%s]", *( select_func + data->use_hw ), data->name, pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No" );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_toggle( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
			case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set toggle", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->toggle_up_down	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), Toggle(%s)", *( select_func + data->use_hw ), data->name, pmic_data->toggle_up_down ? "Yes" : "No" );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_loop( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set loop", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->ramp_loop	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), Loop(%s)", *( select_func + data->use_hw ), data->name, pmic_data->ramp_loop ? "Yes" : "No" );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_ramp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set ramp", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->ramp_up_down	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), Ramp(%s)", *( select_func + data->use_hw ), data->name, pmic_data->ramp_up_down ? "Yes" : "No" );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_fade_in_out_brightness_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set fade in/out brightness", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->fade_in_out_pwm	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), set fade in/out PWM[%d]", *( select_func + data->use_hw ), data->name, pmic_data->fade_in_out_pwm );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_invert( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set invert", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->invert	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), %s invert", *( select_func + data->use_hw ), data->name, pmic_data->invert ? "Enable" : "Disable" );
			break;
		}

		case	LED_HW_PMIC_MPP :
		{
			LED_MSG( "PMIC %s led(%s) can't set invert", *( select_func + data->use_hw ), data->name );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_current_sink( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set current sink", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		{
			LED_MSG( "PMIC %s led(%s) can't set current sink", *( select_func + data->use_hw ), data->name );
			break;
		}

		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			if( parameter->para1 >= sizeof( current_sink_table ) / sizeof( *current_sink_table ) )
			{
				LED_MSG( "PMIC %s led(%s) can't suppurt this value(%d)", *( select_func + data->use_hw ), data->name, parameter->para1 );
				break;
			}

			mutex_lock( &data->lock );
			pmic_data->current_sink	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), current sink(%s)", *( select_func + data->use_hw ), data->name, *( current_sink_table + pmic_data->current_sink ) );
			break;
		}

//PERI-BJ-SetCurrent-00+{
		case	LED_HW_PMIC_LPG :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			if( parameter->para1 >= sizeof( current_rgb_table ) / sizeof( *current_rgb_table ) )
			{
				LED_MSG( "LED : PMIC %s led(%s) can't suppurt this value(%d)", *( select_func + data->use_hw ), data->name, parameter->para1 );
				break;
			}

			mutex_lock( &data->lock );
			pmic_data->current_sink	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "LED : PMIC %s led(%s), current sink(%s)", *( select_func + data->use_hw ), data->name, *( current_rgb_table + pmic_data->current_sink ) );
			break;
		}
//PERI-BJ-SetCurrent-00+}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_pin( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			msm_data->msm_pin	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s), set pin(%d)", msm_data->msm_pin, data->name, msm_data->msm_pin );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pmic_pin	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), set pin(%d)", *( select_func + data->use_hw ), data->name, pmic_data->pmic_pin );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_on_off_level( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			msm_data->led_on_level	= parameter->para1;
			msm_data->led_off_level	= parameter->para2;
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s), level[on:off]=[%s:%s]", msm_data->msm_pin, data->name, msm_data->led_on_level ? "High" : "Low", msm_data->led_off_level ? "High" : "Low" );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			LED_MSG( "PMIC %s led(%s) can't set on/off level", *( select_func + data->use_hw ), data->name );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_lpg_out( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set LPG out", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->lpg_out	= parameter->para1;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), LPG(%d)", *( select_func + data->use_hw ), data->name, pmic_data->lpg_out );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_lut_table_range( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s) can't set range of LUT table", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->lut_table_start	= parameter->para1;
			pmic_data->lut_table_end	= parameter->para2;
			mutex_unlock( &data->lock );
			LED_MSG( "PMIC %s led(%s), LUT table[%d-%d]", *( select_func + data->use_hw ), data->name, pmic_data->lut_table_start, pmic_data->lut_table_end );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

static void	led_set_on_off_state( struct led_data *data, struct command_parameter *parameter )
{
	mutex_lock( &data->lock );
	data->on_off_state	= parameter->para1;
	mutex_unlock( &data->lock );

	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			LED_MSG( "MSM GPIO-%d(%s), the state is %s", msm_data->msm_pin, data->name, data->on_off_state ? "On" : "Off" );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			LED_MSG( "PMIC %s led(%s), the state is %s", *( select_func + data->use_hw ), data->name, data->on_off_state ? "On" : "Off" );
			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}
}

//PERI-BJ-Fade_in_out_diff_time-00*{
static void	led_set_fade_in_out_diff_data(struct led_data *data, struct command_parameter *parameter)
{

	struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

	//mutex_lock( &data->lock );

	//PERI-BJ-AddDefaultFadeValue-00*{
	//Avoid too small value for fade in/out. It cause error execution.
	data->fade_in_interval = (parameter->para2 > 500) ? parameter->para2 : 500;
	data->fade_out_interval = (parameter->para3 > 500) ? parameter->para3 : 500;
	//PERI-BJ-AddDefaultFadeValue-00*}

	pmic_data->control	= parameter->para1;
	pmic_data->command	= SMEM_CMD_LED_FADE_IN_OUT;
	pmic_data->hardware = data->use_hw;
	pmic_data->toggle_up_down = DISABLE;
	pmic_data->ramp_loop = DISABLE;


	if(data->in_out_state == FADE_STATE_IN){
		pmic_data->ramp_up_down = ENABLE; // Dark -> Bright
		pmic_data->interval = data->fade_in_interval;
	}
	else{
		pmic_data->ramp_up_down = DISABLE; // Bright -> Dark
		pmic_data->interval = data->fade_out_interval;
	}
	//pmic_data->blinking_time1	= 100;
	//pmic_data->blinking_time2	= 100;

	//data->on_off_state = 0;
	//data->in_out_state = FADE_STATE_IN;
	//mutex_unlock( &data->lock );
}

static void led_fade_in_out_time_diff(struct led_data *data, struct command_parameter *parameter )
{

	unsigned int 		timeout;
	struct led_data 	*destination_r	= leds_data + 0;
	struct led_data 	*destination_g	= leds_data + 1;
	struct led_data 	*destination_b	= leds_data + 2;

	struct led_diff_data* diff_data;

	struct led_pmic_data	*pmic_data_r	= &destination_r->detail.pmic_data;
	//struct led_pmic_data	*pmic_data_g	= &destination_g->detail.pmic_data;
	//struct led_pmic_data	*pmic_data_b	= &destination_b->detail.pmic_data;
	//struct command_parameter parameter_diff;
	timeout = pmic_data_r->interval+timeout_buf;

	if( ( diff_data = kzalloc(sizeof(struct led_diff_data), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for driver data" );
		return;
	}

	diff_data->pmic_data_r = &destination_r->detail.pmic_data;
	diff_data->pmic_data_g = &destination_g->detail.pmic_data;
	diff_data->pmic_data_b = &destination_b->detail.pmic_data;

	diff_data->pwm_dev_r = destination_r->pwm_dev;
	diff_data->pwm_dev_g = destination_g->pwm_dev;
	diff_data->pwm_dev_b = destination_b->pwm_dev;

	LED_DBG("time_diff start");

	switch( data->use_hw )
	{
#ifdef CONFIG_HW_MSM_GPIO
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			LED_MSG( "MSM GPIO-%d(%s) does not support fade in/out", msm_data->msm_pin, data->name );
			break;
		}
#endif
		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		case	LED_HW_PMIC_LPG:
		{
			if( parameter->para1 != LED_TURN_OFF )
			{
				hrtimer_cancel( &destination_r->led_time_diff );
				LED_DBG_DIFF("[LED_Driver/%d:%s() : FADE_STATE_IN-Turn OFF]",__LINE__,__FUNCTION__);
	//			"32 : LED fade in/out at different timing (on/off, in time, out time), 3 parameter\n"
	//			"33 : LED fade in/out once (on, in/out time, toggle up/down), 3 parameter\n"
				mutex_lock( &data->lock );

				LED_DBG_DIFF("fade in");
				LED_DBG_DIFF("fade_off_flag = 0");
				fade_off_flag = 0;

				destination_r->in_out_state = FADE_STATE_IN;
				destination_g->in_out_state = FADE_STATE_IN;
				destination_b->in_out_state = FADE_STATE_IN;

				led_set_fade_in_out_diff_data(destination_r, parameter);
				led_set_fade_in_out_diff_data(destination_g, parameter);
				led_set_fade_in_out_diff_data(destination_b, parameter);

				mutex_unlock( &data->lock );

				LED_DBG_DIFF("fade out, set rgb start");
				control_fade_nonsync_leds(destination_r->spmi_dev, diff_data);

				LED_DBG_DIFF("fade out, set rgb done");

				LED_DBG("[LED_Driver/%d:%s()] : hrtimer_start:LED_SEC(timeout)=%d,LED_NS(timeout)=%d",__LINE__,__FUNCTION__,LED_SEC(timeout),LED_NS(timeout));
				hrtimer_start( &destination_r->led_time_diff, ktime_set( LED_SEC(timeout), LED_NS(timeout) ), HRTIMER_MODE_REL );

			}
			else
			{
				LED_DBG_DIFF("fade_off_flag = 1");
				mutex_lock( &data->lock );//PERI-BJ-Add_Mutex-00+
				fade_off_flag = 1;
				mutex_unlock( &data->lock );
			}

			LED_MSG( "PMIC %s led(%s) fade in/out %s, [Pin:LPG]=[%d:%d]", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data_r->pmic_pin, pmic_data_r->lpg_out );

			break;
		}

		default :
			LED_ERR("Does not support this H/W type(%d)", data->use_hw );
			break;
	}

	kfree(diff_data); /* PERI-AH-Fixed coverity resource leak-00++ */
	
}
//PERI-BJ-Fade_in_out_diff_time-00*}

//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT
static void led_sched_blink( struct work_struct *work )
{
	struct led_data		*sw_led		= container_of( work, struct led_data, work_blink );
	struct led_pmic_data	*pmic_data	= &sw_led->detail.pmic_data;
	unsigned int		timeout;

	LED_DBG("[LED_Driver/%d:%s()]",__LINE__,__FUNCTION__);


	hrtimer_cancel( &sw_led->led_time );

	mutex_lock( &sw_led->lock );
	pmic_data->command	= SMEM_CMD_LED_ON_OFF;
	pmic_data->hardware	= sw_led->use_hw;
	pmic_data->control	= LED_TURN_ON;
	pmic_data->on_off_pwm	= sw_led->led_state == LED_OFF_STATE ? pmic_data->blinking_pwm2 : pmic_data->blinking_pwm1;

	LED_DBG("led_sched_blink not support");
	//common_control_leds(sw_led->pwm_dev, pmic_data);

	timeout	= sw_led->led_state == LED_OFF_STATE ? pmic_data->blinking_time2 : pmic_data->blinking_time1;
	sw_led->led_state	= sw_led->led_state == LED_OFF_STATE ? LED_ON_STATE : LED_OFF_STATE;
	hrtimer_start( &sw_led->led_time, ktime_set( LED_SEC( timeout ), LED_NS( timeout ) ), HRTIMER_MODE_REL );
	mutex_unlock( &sw_led->lock );
}

static void led_sched_fade_in_out( struct work_struct *work )
{
	struct led_data		*sw_led		= container_of( work, struct led_data, work_fade_in_out );
	struct led_pmic_data	*pmic_data	= &sw_led->detail.pmic_data;
	unsigned int		timeout;

	hrtimer_cancel( &sw_led->led_time );

	mutex_lock( &sw_led->lock );
	pmic_data->command	= SMEM_CMD_LED_ON_OFF;
	pmic_data->hardware	= sw_led->use_hw;
	pmic_data->control	= LED_TURN_ON;
	pmic_data->on_off_pwm	= sw_led->sw_brightness / MINI_UNIT;

	LED_DBG("led_sched_fade_in_out not support");
	//common_control_leds(sw_led->pwm_dev, pmic_data);

	if( sw_led->sw_led_count-- )
	{
		timeout	= sw_led->unit_time_out;

		if( sw_led->led_state == LED_OFF_STATE )
			sw_led->sw_brightness	= sw_led->sw_brightness + sw_led->unit_brightness > MAX_PWM * MINI_UNIT ? MAX_PWM * MINI_UNIT : sw_led->sw_brightness + sw_led->unit_brightness;
		else
			sw_led->sw_brightness	= sw_led->sw_brightness - sw_led->unit_brightness < MIN_PWM * MINI_UNIT ? MIN_PWM * MINI_UNIT : sw_led->sw_brightness - sw_led->unit_brightness;
	}
	else
	{
		sw_led->sw_led_count	= FADE_IN_OUT_LEVEL;
		timeout	= sw_led->led_state == LED_OFF_STATE ? pmic_data->blinking_time1 : pmic_data->blinking_time2;
		sw_led->led_state	= sw_led->led_state == LED_OFF_STATE ? LED_ON_STATE : LED_OFF_STATE;
	}

	hrtimer_start( &sw_led->led_time, ktime_set( LED_SEC( timeout ), LED_NS( timeout ) ), HRTIMER_MODE_REL );
	mutex_unlock( &sw_led->lock );
}

static enum hrtimer_restart led_timeout( struct hrtimer *timer )
{
	struct  led_data	*sw_led	= container_of( timer, struct led_data, led_time );

	if( sw_led->unit_time_out )
		queue_work( work_queue, &sw_led->work_fade_in_out );
	else
		queue_work( work_queue, &sw_led->work_blink );

	return	HRTIMER_NORESTART;
}
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}


//PERI-BJ-Fade_in_out_diff_time-00*{
static void led_sched_fade_in_out_diff( struct work_struct *work )
{
	unsigned int		timeout;

	struct led_data 	*destination_r	= leds_data + 0;
	struct led_data 	*destination_g	= leds_data + 1;	
	struct led_data 	*destination_b	= leds_data + 2;

	struct led_diff_data* diff_data;

	struct command_parameter parameter_diff1;
	struct command_parameter parameter_diff2;

	if( ( diff_data = kzalloc(sizeof(struct led_diff_data), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for driver data" );
		return;
	}

	diff_data->pmic_data_r = &destination_r->detail.pmic_data;
	diff_data->pmic_data_g = &destination_g->detail.pmic_data;
	diff_data->pmic_data_b = &destination_b->detail.pmic_data;

	diff_data->pwm_dev_r = destination_r->pwm_dev;
	diff_data->pwm_dev_g = destination_g->pwm_dev;
	diff_data->pwm_dev_b = destination_b->pwm_dev;

	hrtimer_cancel( &destination_r->led_time_diff);

	if(destination_r->in_out_state == FADE_STATE_IN){
		//use rgb lock, may have sequential false, so use r lock only here to try the feature
		//Do Fade out
		LED_DBG_DIFF("FADE OUT, destination_r->fade_out_interval=%d",destination_r->fade_out_interval);

		mutex_lock( &destination_r->lock );

		timeout = destination_r->fade_out_interval+timeout_buf;

		//set next state
		destination_r->in_out_state = FADE_STATE_OUT;
		destination_g->in_out_state = FADE_STATE_OUT;
		destination_b->in_out_state = FADE_STATE_OUT;

		//fade out
		parameter_diff1.para1 = ENABLE;//on
		parameter_diff1.para2 = destination_r->fade_in_interval;
		parameter_diff1.para3 = destination_r->fade_out_interval;

		//set data
		led_set_fade_in_out_diff_data(destination_r, &parameter_diff1);
		led_set_fade_in_out_diff_data(destination_g, &parameter_diff1);
		led_set_fade_in_out_diff_data(destination_b, &parameter_diff1);

		mutex_unlock( &destination_r->lock );

		LED_DBG_DIFF("fade out, set rgb start");
		control_fade_nonsync_leds(destination_r->spmi_dev, diff_data);

		LED_DBG_DIFF("fade_off_flag=%d",fade_off_flag);
		if(!fade_off_flag){//no timer start while turn off flag enable
			LED_DBG_DIFF("hrtimer_start:LED_SEC(timeout)=%d,LED_NS(timeout)=%d",LED_SEC(timeout),LED_NS(timeout));
			hrtimer_start( &destination_r->led_time_diff, ktime_set( LED_SEC( (timeout)), LED_NS( timeout ) ), HRTIMER_MODE_REL );
		}

	}
	else{
		//use rgb lock, may have sequential false, so use r lock only here to try the feature
		//Do Fade in
		LED_DBG_DIFF("FADE IN, destination_r->fade_in_interval=%d",destination_r->fade_in_interval);

		mutex_lock( &destination_r->lock );

		timeout = destination_r->fade_in_interval+timeout_buf;

		//set next state
		destination_r->in_out_state = FADE_STATE_IN;
		destination_g->in_out_state = FADE_STATE_IN;
		destination_b->in_out_state = FADE_STATE_IN;

		//fade in
		parameter_diff2.para1 = ENABLE;//on
		parameter_diff2.para2 = destination_r->fade_in_interval;
		parameter_diff2.para3 = destination_r->fade_out_interval;

		//set data
		led_set_fade_in_out_diff_data(destination_r, &parameter_diff2);
		led_set_fade_in_out_diff_data(destination_g, &parameter_diff2);
		led_set_fade_in_out_diff_data(destination_b, &parameter_diff2);

		mutex_unlock( &destination_r->lock );

		LED_DBG_DIFF("fade in, set rgb start");
		control_fade_nonsync_leds(destination_r->spmi_dev, diff_data);

		LED_DBG_DIFF("hrtimer_start:LED_SEC(timeout)=%d,LED_NS(timeout)=%d",LED_SEC(timeout),LED_NS(timeout));
		hrtimer_start( &destination_r->led_time_diff, ktime_set( LED_SEC(timeout), LED_NS( timeout ) ), HRTIMER_MODE_REL );
		
	}

	kfree(diff_data); /* PERI-AH-Fixed coverity resource leak-00++ */
}

static enum hrtimer_restart led_timeout_diff( struct hrtimer *timer )
{
	struct  led_data	*sw_led	= container_of( timer, struct led_data, led_time_diff );

	queue_work( work_queue_diff, &sw_led->work_fade_in_out_diff );

	return	HRTIMER_NORESTART;
}
//PERI-BJ-Fade_in_out_diff_time-00*}

//PERI-BJ-Add_Special_Mode-00+{
static void	led_fade_in_out_time_diff_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_fade_in_out_time_diff( data, parameter );
}

static void	led_fade_in_out_once_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		LED_MSG( "%s led, special mode", data->name );
	else
		led_fade_in_out_once_set( data, parameter );
}
//PERI-BJ-Add_Special_Mode-00+}

static int	prepare_commands( void )
{

	struct led_command_source	led_command_list[] =
	{
		{ LED_COMMAND_ON_OFF, led_on_off_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_BLINKING, led_blinking_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT, led_fade_in_out_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_ON_BRIGHTNESS, led_on_brightness_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_BLINKING_BRIGHTNESS, led_blinking_brightness_set, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_INTERVAL, led_fade_in_out_interval_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_BLINKING_TIME, led_blinking_time_set, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_MODE, led_mode_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_ON_OFF, led_on_off_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_BLINKING, led_blinking_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_FADE_IN_OUT, led_fade_in_out_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_CLK_DIV_EXP, led_set_clk_div_exp, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_SET_CLK, led_set_clk, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_DIV, led_set_div, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_EXP, led_set_exp, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_TOGGLE_LOOP_RAMP, led_set_toggle_loop_ramp, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_SET_TOGGLE, led_set_toggle, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_LOOP, led_set_loop, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_RAMP, led_set_ramp, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_INVERT, led_set_invert, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_CURRENT_SINK, led_set_current_sink, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_PIN, led_set_pin, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_ON_OFF_LEVEL, led_set_on_off_level, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_SET_LPG_OUT, led_set_lpg_out, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_BRIGHTNESS, led_fade_in_out_brightness_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SW_BLINKING, led_sw_blinking_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SW_FADE_IN_OUT, led_sw_fade_in_out_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_SW_BLINKING, led_sw_blinking_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_SW_FADE_IN_OUT, led_sw_fade_in_out_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_LUT_TABLE_RANGE, led_set_lut_table_range, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_SET_ON_OFF_STATE, led_set_on_off_state, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_TIME_DIFF, led_fade_in_out_time_diff_check_mode, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_ONCE , led_fade_in_out_once_check_mode, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_SPECIAL_FADE_IN_OUT_TIME_DIFF, led_fade_in_out_time_diff, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_SPECIAL_FADE_IN_OUT_ONCE , led_fade_in_out_once_set, COMMAND_3_PARAMENTER },

	};

	struct BST_data	*sort_buffer;
	struct BST_info	sort_info;
	int		loop, return_value;

	return_value	= 1;

	if( ( command_info.commmand_buffer = kzalloc(sizeof(struct BS_data) * sizeof( led_command_list ) / sizeof( *led_command_list ), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for command buffer" );
		return	0;
	}

	if( ( sort_buffer = kzalloc(sizeof(struct BST_data) * sizeof( led_command_list ) / sizeof( *led_command_list ), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for sort buffer of command" );
		kfree(command_info.commmand_buffer);
		return	0;
	}

	if( ( command_data = kzalloc(sizeof(struct led_command_data) * sizeof( led_command_list ) / sizeof( *led_command_list ), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for sort buffer of command" );
		kfree(command_info.commmand_buffer);
		kfree(sort_buffer);
		return	0;
	}

	BST_init( &sort_info );

	for( loop = 0 ; loop < sizeof( led_command_list ) / sizeof( *led_command_list ) ; ++loop )
	{
		struct led_command_source	*command		= led_command_list + loop;
		struct led_command_data	*commands_data	= command_data + loop;
		struct BST_data		*sort_data	= sort_buffer + loop;

		commands_data->function	= command->function;
		commands_data->parameter	= command->parameter;
		sort_data->index		= command->command;
		sort_data->data		= commands_data;
		BST_add( &sort_info, sort_data );
	}

	if( !( command_info.count = BST_sort( &sort_info, command_info.commmand_buffer, sizeof( led_command_list ) / sizeof( *led_command_list ) ) ) )
	{
		LED_ERR("Sort command failed" );
		return_value	= 0;
	}

	kfree(sort_buffer);
	return	return_value;
}

static ssize_t led_command( void *node_data, struct device_attribute *attr, char *buf)
{
	return	snprintf( buf, PAGE_SIZE, 
		         "CMD\n"
			 "01 : LED on/off, 1 parameter\n"
			 "02 : LED blinking, 1 parameter\n"
			 "03 : LED fade in/out, 1 parameter\n"
			 "04 : Brightness of LED on, 1 parameter\n"
			 "05 : Brightness of LED blinking, 2 parameter\n"
			 "06 : Interval for LED fade in/out, 1 parameter\n"
			 "07 : Blinking time, 2 parameter\n"
			 "08 : Set mode, 1 parameter\n"
			 "09 : LED on/off in special mode, 1 parameter\n"
			 "10 : LED blinking in special mode, 1 parameter\n"
			 "11 : LED fade in/out in special mode, 1 parameter\n"
			 "12 : Set PMIC GPIO/MPP clock, DIV & EXP, 3 parameter\n"
			 "13 : Set PMIC GPIO/MPP clock, 1 parameter\n"
			 "14 : Set PMIC GPIO/MPP DIV, 1 parameter\n"
			 "15 : Set PMIC GPIO/MPP EXP, 1 parameter\n"
			 "16 : Set PMIC GPIO/MPP toggle, loop & ramp, 3 parameter\n"
			 "17 : Set PMIC GPIO/MPP toggle, 1 parameter\n"
			 "18 : Set PMIC GPIO/MPP loop, 1 parameter\n"
			 "19 : Set PMIC GPIO/MPP ramp, 1 parameter\n"
			 "20 : Set PMIC GPIO invert, 1 parameter\n"
			 "21 : Set PMIC MPP/LPG current sink, 1 parameter\n"
			 "22 : Set LED pin, 1 parameter\n"
			 "23 : Set MSM GPIO on/off level, 2 parameter\n"
			 "24 : Set PMIC GPIO/MPP/LPG, 1 parameter\n"
			 "25 : Brightness of LED fade in/out, 1 parameter\n"
			 "26 : LED SW blinking, 1 parameter\n"
			 "27 : LED SW fade in/out, 1 parameter\n"
			 "28 : LED SW blinking in special mode, 1 parameter\n"
			 "29 : LED SW fade in/out in special mode, 1 parameter\n"
			 "30 : Set range of LUT table, 2 parameter\n"
			 "31 : Set on/off state of LED, 1 parameter\n"
			 "32 : LED fade in/out at different timing (on/off, in time, out time), 3 parameter\n"
			 "33 : LED fade in/out once (on, in/out time, toggle up/down), 3 parameter\n"
			 "34 : LED fade in/out at different timing in special mode, 3 parameter\n"
			 "35 : LED fade in/out once in special mode, 3 parameter\n"

			 );
}

static ssize_t led_control( void *node_data, struct device_attribute *attr, const char *buf, size_t size )
{

	struct leds_driver_data	*data		= node_data;
	struct wake_lock		*normal_wakelock	= &data->normal_wakelock;
	unsigned int		pointer		= 0;
	int			rem_size		= size;

	LED_DBG("wake_lock(normal_wakelock)");
	wake_lock(normal_wakelock);

	while( pointer < rem_size )
	{
		struct BS_data		*search_data;
		struct led_data		*detail_data;
		struct led_command_data	*command;
		unsigned int		char_count;
		int			user_command, led_id;

		if( !get_para_from_buffer( buf + pointer, "%d", &user_command ) )
		{
			LED_MSG( "Can't get command from buffer" );
			break;
		}

		if( !( search_data = binary_search( command_info.commmand_buffer, command_info.count, user_command ) ) )
		{
			LED_MSG( "Can't support command(%d)", user_command );
			break;
		}

		command		= ( struct led_command_data* )search_data->data;

		if( !( char_count = get_para_char_count( buf + pointer, buf + size, 1 ) ) )
		{
			LED_MSG( "Can't get charactor count." );
			break;
		}

		pointer += char_count;

		if( !get_para_from_buffer( buf + pointer, "%d", &led_id ) )
		{
			LED_MSG( "Can't get led id" );
			break;
		}

		if( !( search_data = binary_search( data->index_buffer, data->count, led_id ) ) )
		{
			LED_MSG( "Can't support led id(%d)", led_id );
			break;
		}

		detail_data	= ( struct led_data* )search_data->data;

		if( !( char_count = get_para_char_count( buf + pointer, buf + size, 1 ) ) )
		{
			LED_MSG( "Can't get charactor count.." );
			break;
		}

		pointer += char_count;
		LED_MSG( "[CMD:ID:Para]=[%d:%d:%d]", user_command, led_id, command->parameter );

		{
			struct command_parameter	parameter;
			int		*parameter_pointer[] = { &parameter.para1, &parameter.para2, &parameter.para3, &parameter.para4, &parameter.para5, &parameter.para6, &parameter.para7, &parameter.para8 };
			unsigned int	loop;

			if( command->parameter > sizeof( parameter_pointer ) / sizeof( *parameter_pointer ) )
			{
				LED_MSG( "Out of parameter array" );
				break;
			}

			for( loop = 0 ; loop < command->parameter ; ++loop )
			{
				if( !get_para_from_buffer( buf + pointer, "%d", *( parameter_pointer + loop ) ) )
					break;

				if( !( char_count = get_para_char_count( buf + pointer, buf + size, ONE_PARAMETER ) ) )
				{
					LED_MSG( "Can't get charactor count..." );
					break;
				}

				pointer += char_count;
			}

			if( loop == command->parameter )
				command->function( detail_data, &parameter );
			else
			{
				LED_MSG( "Can't get parameter" );
				break;
			}
		}
	}

	wake_unlock(normal_wakelock);
	LED_DBG("wake_unlock(normal_wakelock)");

	return size;
}

static ssize_t led_info( void *node_data, struct device_attribute *attr, char *buf)
{
	struct leds_driver_data	*data	= node_data;
	int	loop, info_size, rem_size;

	rem_size		= PAGE_SIZE;
	info_size	= 0;

	for( loop = 0 ; loop < data->count ; ++loop )
	{
		struct led_data	*led_info	= ( data->index_buffer + loop )->data;

		switch( led_info->use_hw )
		{
#ifdef CONFIG_HW_MSM_GPIO
			case	LED_HW_MSM_GPIO :
			{
				struct led_gpio_data	*msm_data	= &led_info->detail.gpio_data;

				info_size += snprintf( buf + info_size, rem_size,
					             "\n%s LED(%d), MSM GPIO-%d\n"
						     "Level[On:Off]=[%d:%d]\n"
						     "%s mode\n"
						     "%s state\n",
						     led_info->name, ( data->index_buffer + loop )->index, msm_data->msm_pin,
						     msm_data->led_on_level, msm_data->led_off_level,
						     led_info->special_mode ? "Special" : "Normal",
						     led_info->on_off_state ? "On" : "Off"
						     );
				break;
			}
#endif
//PERI-BJ-SetCurrent-00+{
			case	LED_HW_PMIC_GPIO :
			{
				struct led_pmic_data	*pmic_data	= &led_info->detail.pmic_data;
			
				info_size += snprintf( buf + info_size, rem_size,
								 "\n%s LED(%d), PMIC GPIO-%d, LPG-%d\n"
							 "On/Off PWM(%d), Blinking PWM[%d:%d], Fade in/out PWM(%d)\n"
							 "Blinking Time[%dms:%dms], Interval(%dms)\n"
							 "Frequency[CLK:DIV:EXP]=[%d:%d:%d]\n"
							 "Ramp Generator[Toggle:Loop:Up]=[%s:%s:%s]\n"
							 "LUT table[%d-%d]\n"
							 "%s invert\n"
							 "%s mode\n"
							 "%s state\n",
							 led_info->name, ( data->index_buffer + loop )->index, pmic_data->pmic_pin + 1, pmic_data->lpg_out + 1,
							 pmic_data->on_off_pwm, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2, pmic_data->fade_in_out_pwm,
							 pmic_data->blinking_time1, pmic_data->blinking_time2, pmic_data->interval,
							 pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp,
							 pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No",
							 pmic_data->lut_table_start, pmic_data->lut_table_end,
							 pmic_data->invert ? "Enable" : "Disable",
							 led_info->special_mode ? "Special" : "Normal",
							 led_info->on_off_state ? "On" : "Off"
							 );
				break;
			}

			case	LED_HW_PMIC_LPG:
			{
				struct led_pmic_data	*pmic_data	= &led_info->detail.pmic_data;

				info_size += snprintf( buf + info_size, rem_size,
					             "\n%s LED(%d), PMIC GPIO-%d, LPG-%d\n"
						     "On/Off PWM(%d), Blinking PWM[%d:%d], Fade in/out PWM(%d)\n"
						     "Blinking Time[%dms:%dms], Interval(%dms)\n"
						     "Frequency[CLK:DIV:EXP]=[%d:%d:%d]\n"
						     "Ramp Generator[Toggle:Loop:Up]=[%s:%s:%s]\n"
						     "LUT table[%d-%d]\n"
						     "%s invert\n"
							 "Current sink(%s)\n"
						     "%s mode\n"
						     "%s state\n",
						     led_info->name, ( data->index_buffer + loop )->index, pmic_data->pmic_pin + 1, pmic_data->lpg_out + 1,
						     pmic_data->on_off_pwm, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2, pmic_data->fade_in_out_pwm,
						     pmic_data->blinking_time1, pmic_data->blinking_time2, pmic_data->interval,
						     pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp,
						     pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No",
						     pmic_data->lut_table_start, pmic_data->lut_table_end,
						     pmic_data->invert ? "Enable" : "Disable",
							 *( current_rgb_table + pmic_data->current_sink ),
						     led_info->special_mode ? "Special" : "Normal",
						     led_info->on_off_state ? "On" : "Off"
						     );
				break;
			}
//PERI-BJ-SetCurrent-00+}
			case	LED_HW_PMIC_MPP :
			{
				struct led_pmic_data	*pmic_data	= &led_info->detail.pmic_data;

				info_size += snprintf( buf + info_size, rem_size,
					             "\n%s LED(%d), MPP Source Select-%d, LPG-%d\n"
						     "On/Off PWM(%d), Blinking PWM[%d:%d], Fade in/out PWM(%d)\n"
						     "Blinking Time[%dms:%dms], Interval(%dms)\n"
						     "Frequency[CLK:DIV:EXP]=[%d:%d:%d]\n"
						     "Ramp Generator[Toggle:Loop:Up]=[%s:%s:%s]\n"
						     "LUT table[%d-%d]\n"
						     "Current sink(%s)\n"
						     "%s mode\n"
						     "%s state\n",
						     led_info->name, ( data->index_buffer + loop )->index, pmic_data->source_sel, pmic_data->lpg_out + 1,
						     pmic_data->on_off_pwm, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2, pmic_data->fade_in_out_pwm,
						     pmic_data->blinking_time1, pmic_data->blinking_time2, pmic_data->interval,
						     pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp,
						     pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No",
						     pmic_data->lut_table_start, pmic_data->lut_table_end,
						     *( current_sink_table + pmic_data->current_sink ),
						     led_info->special_mode ? "Special" : "Normal",
						     led_info->on_off_state ? "On" : "Off"
						     );
				break;
			}
		}
		rem_size	= PAGE_SIZE - info_size;

		if( rem_size <= 0 )
		{
			LED_MSG( "Out of buffer size" );
			break;
		}
	}

	return	info_size;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void	led_early_suspend_function(struct early_suspend *handler)
{
	struct wake_lock		*timeout_wakelock	= &container_of(handler, struct leds_driver_data, led_early_suspend)->timeout_wakelock;
	struct BS_data		*led_datas	= container_of(handler, struct leds_driver_data, led_early_suspend)->index_buffer;
	unsigned int		count		= container_of(handler, struct leds_driver_data, led_early_suspend)->count;
	struct command_parameter	parameter;
	unsigned int		loop;

	wake_lock_timeout( timeout_wakelock, WAIT_LOCK_TIME * HZ / 1000 );
	LED_MSG( "Wake lock(%dms)", WAIT_LOCK_TIME );
	parameter.para1	= LED_TURN_OFF;

	for( loop = 0 ; loop < count ; ++loop )
		if( !( ( struct led_data* )( led_datas + loop )->data )->on_off_state )
			led_on_off_check_mode( ( struct led_data* )( led_datas + loop )->data, &parameter );
}

static void	led_late_resume_function(struct early_suspend *handler)
{
}
#endif

static int __devinit qpnp_mpp_init(struct device_node *node, struct leds_driver_data *driver_data)
{
	int	loop;
	struct device_node *temp;
	struct led_pmic_data	*pmic_data;
	int rc;
	const char *led_label;
	const char *temp_string;
	u32 val;
	struct led_data *destination;

	rc = of_property_read_string(node, "label", &led_label);
	if (rc < 0) {
		dev_err(&driver_data->spmi_dev->dev,
			"Failure reading label, rc = %d\n", rc);
		return	-EINVAL;
	}
	//LED_DBG("led_label = %s", led_label);

	loop = -1;
	temp = NULL;
	for_each_child_of_node(node, temp) {
		loop++;
		destination = leds_data + loop;
		//struct led_device_data	*source 	= device_data->device_data + loop;

		mutex_init(&destination->lock);

		destination->spmi_dev = driver_data->spmi_dev;

		rc = of_property_read_string(temp, "linux,name", &destination->name);
		if (rc < 0) {
			dev_err(&driver_data->spmi_dev->dev,
				"Failure reading led name, rc = %d\n", rc);
			return -EINVAL;
		}

		//LED_DBG("destination->name= %s", destination->name);

		rc = of_property_read_u32(temp, "fih,use_hw",
			&destination->use_hw);
		if (rc < 0) {
			dev_err(&driver_data->spmi_dev->dev,
				"Failure reading use_hw, rc =	%d\n", rc);
			return	-EINVAL;		
		}

		//LED_DBG("destination->use_hw = %d", destination->use_hw);

/*
		rc = of_property_read_u32(temp, "qcom,max-current",
			&destination->max_current);
		if (rc < 0) {
			dev_err(&driver_data->spmi_dev->dev,
				"Failure reading max_current, rc =	%d\n", rc);
			return	-EINVAL;		
		}

		LED_DBG("destination->max_current= %d", destination->max_current);
*/

		destination->special_mode	= SPECIAL_CONTROL_OFF;
			
		rc = of_property_read_u32(temp, "qcom,id", &destination->id);
		if (rc < 0) {
			dev_err(&driver_data->spmi_dev->dev,
				"Failure reading led id, rc =  %d\n", rc);
			return -EINVAL;
		}

		destination->default_on = false;
		rc = of_property_read_string(temp, "qcom,default-state",
			&temp_string);
		if (!rc) {
			if (strncmp(temp_string, "on", sizeof("on")) == 0)
				destination->default_on = true;
		} else if (rc != -EINVAL)
			return rc;
		
		destination->turn_off_delay_ms = 0;
		rc = of_property_read_u32(temp, "qcom,turn-off-delay-ms", &val);
		if (!rc)
			destination->turn_off_delay_ms = val;
		else if (rc != -EINVAL)
			return rc;

		pmic_data	= &destination->detail.pmic_data;

		pmic_data->base = LED_DEFAULT_BASE_ADDR;
		rc = of_property_read_u32(temp, "fih,base", &val);
		if (!rc)
			pmic_data->base = (u16) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->base: 0x%x\n", pmic_data->base);

		pmic_data->source_sel = LED_MPP_SOURCE_SEL_DEFAULT;
		rc = of_property_read_u32(temp, "qcom,source-sel", &val);
		if (!rc)
			pmic_data->source_sel = (u8) val;
		else if (rc != -EINVAL)
			return rc;
		//LED_DBG("pmic_data->source_sel: %d", pmic_data->source_sel);


		//Set color indicator;
		if (strncmp(destination->name, "red", sizeof("red")) == 0)
			pmic_data->color = COLOR_RED;
		else if(strncmp(destination->name, "green", sizeof("green")) == 0)
			pmic_data->color = COLOR_GREEN;
		else if(strncmp(destination->name, "blue", sizeof("blue")) == 0)
			pmic_data->color = COLOR_BLUE;


		pmic_data->mode_ctrl = LED_MPP_MODE_SINK;
		rc = of_property_read_u32(temp, "qcom,mode-ctrl", &val);
		if (!rc)
			pmic_data->mode_ctrl = (u8) val;
		else if (rc != -EINVAL)
			return rc;


		pmic_data->current_sink = 0;
		rc = of_property_read_u32(temp, "fih,current_sink", &val);
		if (!rc)
			pmic_data->current_sink = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		pmic_data->on_off_pwm = LED_OFF;
		rc = of_property_read_u32(temp, "fih,on_off_pwm", &val);
		if (!rc)
			pmic_data->on_off_pwm = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->on_off_pwm: %d", pmic_data->on_off_pwm);


		pmic_data->fade_in_out_pwm = LED_OFF;
		rc = of_property_read_u32(temp, "fih,fade_in_out_pwm", &val);
		if (!rc)
			pmic_data->fade_in_out_pwm = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->fade_in_out_pwm: %d", pmic_data->fade_in_out_pwm);


		pmic_data->interval = 0;
		rc = of_property_read_u32(temp, "fih,interval", &val);
		if (!rc)
			pmic_data->interval = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->interval: %d", pmic_data->interval);


		pmic_data->ramp_loop = 0;
		rc = of_property_read_u32(temp, "fih,ramp_loop", &val);
		if (!rc)
			pmic_data->ramp_loop = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->ramp_loop: %d", pmic_data->ramp_loop);


		pmic_data->ramp_up_down = 0;
		rc = of_property_read_u32(temp, "fih,ramp_up_down", &val);
		if (!rc)
			pmic_data->ramp_up_down = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->ramp_up_down: %d", pmic_data->ramp_up_down);

		pmic_data->toggle_up_down = 0;
		rc = of_property_read_u32(temp, "fih,toggle_up_down", &val);
		if (!rc)
			pmic_data->toggle_up_down = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->toggle_up_down: %d", pmic_data->toggle_up_down);


		pmic_data->blinking_pwm1 = 0;
		rc = of_property_read_u32(temp, "fih,blinking_pwm1", &val);
		if (!rc)
			pmic_data->blinking_pwm1 = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->blinking_pwm1: %d", pmic_data->blinking_pwm1);

		pmic_data->blinking_pwm2 = 0;
		rc = of_property_read_u32(temp, "fih,blinking_pwm2", &val);
		if (!rc)
			pmic_data->blinking_pwm2 = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->blinking_pwm2: %d", pmic_data->blinking_pwm2);

		pmic_data->blinking_time1 = 0;
		rc = of_property_read_u32(temp, "fih,blinking_time1", &val);
		if (!rc)
			pmic_data->blinking_time1 = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->blinking_time1: %d", pmic_data->blinking_time1);

		pmic_data->blinking_time2 = 0;
		rc = of_property_read_u32(temp, "fih,blinking_time2", &val);
		if (!rc)
			pmic_data->blinking_time2 = (unsigned int) val;
		else if (rc != -EINVAL)
			return rc;

		//LED_DBG("pmic_data->blinking_time2: %d", pmic_data->blinking_time2);

		//default without read from dts
		pmic_data->pwm_clock = LED_DEFAULT_PWM_CLOCK;
		pmic_data->pwm_div_value = LED_DEFAULT_PWM_DIV_VALUE;
		pmic_data->pwm_div_exp = LED_DEFAULT_PWM_DIV_EXP;

		//set in fih_pwm_lib.c
		pmic_data->lut_table_start = 0;
		pmic_data->lut_table_end = 0;

		//currently not used for pm8226 MPP
		pmic_data->pmic_pin = -1; //non-pin
		pmic_data->invert = 0;

		if (destination->use_hw != LED_HW_MSM_GPIO)
		{
			rc = of_property_read_u32(temp, "qcom,pwm-channel", &val);
			if (!rc)
			{
				pmic_data->lpg_out= (unsigned int) val;
				destination->pwm_dev =
					pwm_request(pmic_data->lpg_out, destination->name);

				if (IS_ERR_OR_NULL(destination->pwm_dev)) {
					dev_err(&driver_data->spmi_dev->dev, "could not acquire PWM Channel %d, error %ld\n", val,PTR_ERR(destination->pwm_dev));
					destination->pwm_dev = NULL;
					return -ENODEV;
				}
			}
			else
				return rc;
		}
	}

	return 0;
}

static int msm_pmic_led_probe(struct spmi_device *spmi)
{
	struct BST_data		*sort_buffer;
	struct control_node_load	file_node;
	struct BST_info		sort_info;
	struct resource *led_resource;
	struct led_pmic_data	*pmic_data;

	struct led_data 	*destination;
	struct BST_data 	*sort_data;

	struct command_parameter	parameter;
	struct leds_driver_data *driver_data;
	struct device_node *node;
	struct device_node *temp;

	int rc;
	int	loop;
	int num_leds;

	node = spmi->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	num_leds = 0;
	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds)
		return -ECHILD;

	//LED_DBG("num_leds=%d",num_leds);

	if( ( driver_data = kzalloc(sizeof(struct leds_driver_data), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for driver" );
		return	-ENOMEM;
	}

	driver_data->num_leds = num_leds;

	if( ( leds_data = kzalloc(sizeof(struct led_data) * driver_data->num_leds, GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for driver data" );

		kfree(driver_data);
		return	-ENOMEM;
	}

	if( ( destination = kzalloc(sizeof(struct led_data), GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for driver data destination" );

		kfree(driver_data);
		kfree(leds_data);
		return	-ENOMEM;
	}

	if( ( driver_data->index_buffer = kzalloc(sizeof(struct BS_data) * driver_data->num_leds, GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for index buffer" );

		kfree(driver_data);
		kfree(leds_data);
		kfree(destination);
		return	-ENOMEM;
	}

	if( ( sort_buffer = kzalloc(sizeof(struct BST_data) * driver_data->num_leds, GFP_KERNEL) ) == NULL )
	{
		LED_ERR("no memory for binary search tree" );

		kfree(driver_data->index_buffer);
		kfree(driver_data);
		kfree(leds_data);
		kfree(destination);
		return	-ENOMEM;
	}

	BST_init( &sort_info );

	work_queue	= create_workqueue( "led_wq" );

	work_queue_diff	= create_workqueue( "led_wq_diff" );//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*


	led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
	if (!led_resource) {
		dev_err(&spmi->dev, "Unable to get LED base address\n");

		/* PERI-AH-Fixed coverity resource leak-00++[ */
		kfree(sort_buffer);
		kfree(driver_data->index_buffer);
		kfree(destination);
		kfree(leds_data);		
		kfree(driver_data);
		/* PERI-AH-Fixed coverity resource leak-00++] */
		
		rc = -ENXIO;
		return rc;
	}
	
	driver_data->spmi_dev = spmi;

	//set config via dts file
	qpnp_mpp_init(node, driver_data);

	for(loop = 0;loop < driver_data->num_leds;loop++)
	{
		destination = leds_data + loop;
		sort_data = sort_buffer + loop;
		//BJ:need to check the value
		sort_data->index = loop;//source->id;
		
		sort_data->data = destination;

//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT

		INIT_WORK( &destination->work_blink, led_sched_blink );
		INIT_WORK( &destination->work_fade_in_out, led_sched_fade_in_out );
		hrtimer_init( &destination->led_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		destination->led_time.function	= led_timeout;
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}


//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*{
		//For led fad in/out diff timer
		INIT_WORK( &destination->work_fade_in_out_diff, led_sched_fade_in_out_diff );
		hrtimer_init( &destination->led_time_diff, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		destination->led_time_diff.function = led_timeout_diff;
//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*}

		BST_add( &sort_info, sort_data );

		switch( destination->use_hw )
		{
#ifdef CONFIG_HW_MSM_GPIO
			case	LED_HW_MSM_GPIO :
				if( gpio_request(destination->detail.gpio_data.msm_pin, destination->name) )
					LED_ERR("Failed to request GPIO-%d(%s)", destination->detail.gpio_data.msm_pin, destination->name );
				break;
#endif
			case	LED_HW_PMIC_GPIO :
			case	LED_HW_PMIC_MPP :

				/* configure default state */
				if (destination->default_on) {
					pmic_data	= &destination->detail.pmic_data;
					parameter.para1 = pmic_data->on_off_pwm;
					led_on_brightness_set( destination, &parameter );
					
					parameter.para1 = LED_TURN_ON;
					led_on_off_set( destination, &parameter );
				
				} else{
					//parameter.para1 = LED_OFF;
					//led_on_brightness_set( destination, &parameter );
				
					parameter.para1 = LED_TURN_OFF;
					led_on_off_set( destination, &parameter );
				}

				break;

			case	LED_HW_PMIC_LPG:
//PERI-BJ-Modify_Power_Off_Led_Init-00*{
#if 0
				if(!is_power_off_charging()){
					LED_MSG("Not in power off charging. To set_default_pwm_register");

					//PERI-BJ-Fix_Coverity_Explicit_null_dereferenced-00*{
					if (destination->pwm_dev == NULL)
					{
						LED_ERR("[PWM_ERR] NULL pwm device, fail to call set_default_pwm_register()");
					}
					else{
						//If device entering S1 boot and used led before entering OS , it will affect LED behavior(because some led register value be changed).
						//we need to set these register as QCT default value to make sure the behavior is consistent.

						//BJBJBJ
						LED_MSG("set_default_pwm_register(destination->pwm_dev)");
						//set_default_pwm_register(destination->pwm_dev);//PERI-BJ-Set_PWM_register_default-00+
					}
					//PERI-BJ-Fix_Coverity_Explicit_null_dereferenced-00*}
				}
#endif
//PERI-BJ-Modify_Power_Off_Led_Init-00*}
				break;
		}
#if 0

		{
			parameter.para1 = LED_TURN_OFF;
			led_on_off_set( destination, &parameter );

//PERI-BJ-Modify_Power_Off_Led_Init-00+{

			if(is_power_off_charging()){
				LED_MSG("In power off charging state. To turn on red color");

				if(loop == 2)//red
				{
					parameter.para1 = 128;
					led_on_brightness_set( destination, &parameter );

					parameter.para1 = LED_TURN_ON;
					led_on_off_set( destination, &parameter );
				}
			}

//PERI-BJ-Modify_Power_Off_Led_Init-00+}
		}
#endif


	}

	if( !( driver_data->count = BST_sort( &sort_info, driver_data->index_buffer, driver_data->num_leds ) ) )
	{
		LED_ERR("Sort failed" );
		
		/* PERI-AH-Fixed coverity resource leak-00++[ */
		kfree(sort_buffer);
		kfree(driver_data->index_buffer);
		kfree(destination);
		kfree(leds_data);
		kfree(driver_data);
		/* PERI-AH-Fixed coverity resource leak-00++] */	
		
		return	-ENOMEM;
	}

	// For wake lock
	wake_lock_init( &driver_data->timeout_wakelock, WAKE_LOCK_SUSPEND, "fih_led_timeout" );
	wake_lock_init( &driver_data->normal_wakelock, WAKE_LOCK_SUSPEND, "fih_led_normal" );

#ifdef CONFIG_HAS_EARLYSUSPEND
	driver_data->led_early_suspend.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB;
	driver_data->led_early_suspend.suspend	= led_early_suspend_function;
	driver_data->led_early_suspend.resume	= led_late_resume_function;
	register_early_suspend(&driver_data->led_early_suspend);
#endif
	if( prepare_commands() )
	{
		// Create file node in sys/class/led/fih_led
		file_node.class_name	= "led";
		file_node.device_name	= "fih_led";
		file_node.file_node_data	= driver_data;
		file_node.control_read	= led_command;
		file_node.control_write	= led_control;
		file_node.info_read	= led_info;
		file_node_class		= control_file_node_register( &file_node );
	}

	dev_set_drvdata(&spmi->dev, driver_data);
	kfree(sort_buffer);
	return	0;
}

static int __devexit msm_pmic_led_remove(struct spmi_device *spmi)
{

	//struct leds_device_data	*device_data	= spmi->dev.platform_data;
	struct leds_driver_data	*driver_data;
	int	loop;

	driver_data = dev_get_drvdata( &spmi->dev);

	for( loop = 0 ; loop < driver_data->num_leds ; ++loop )
		mutex_destroy( &( ( struct led_data* )driver_data->index_buffer->data + loop )->lock );

	control_file_node_unregister( file_node_class );
	wake_lock_destroy(&driver_data->timeout_wakelock);
	wake_lock_destroy(&driver_data->normal_wakelock);

	//kfree(device_data->device_data);
	//kfree(device_data);
	kfree(driver_data->index_buffer);
	kfree(driver_data);
	kfree(leds_data);
	kfree(command_info.commmand_buffer);
	kfree(command_data);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "fih,pmic-leds",
	}
};

static struct spmi_driver msm_pmic_led_driver = {
	.driver		= {
		.name	= "fih,pmic-leds",
		.of_match_table = spmi_match_table,
	},
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
};

static int __init msm_pmic_led_init(void)
{
	return spmi_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	spmi_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);
MODULE_AUTHOR("Y.S Chang <yschang@fih-foxconn.com>");

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("fih:pmic-leds");
