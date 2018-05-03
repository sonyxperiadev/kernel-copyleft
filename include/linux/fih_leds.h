#ifndef	FIH_LEDS_MSM_PMIC_H
#define	FIH_LEDS_MSM_PMIC_H

#include <mach/pmic.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include <linux/development-tool/development_tool.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/spmi.h>

// Don't modify anything!!!
#define	LED_HW_PMIC_GPIO	0
#define	LED_HW_PMIC_MPP		1
#define	LED_HW_MSM_GPIO		2
#define LED_HW_PMIC_LPG     3

// Don't modify anything!!!
#define	SMEM_CMD_LED_ON_OFF		0
#define	SMEM_CMD_LED_BLINKING		1
#define	SMEM_CMD_LED_FADE_IN_OUT		2

#define	PM_LPG_CLK_SELECT_OFF	0
#define	PM_LPG_CLK_SELECT_1K	1
#define	PM_LPG_CLK_SELECT_32K	2
#define	PM_LPG_CLK_SELECT_19P2M	3

#define	PM_GPIO_1		0
#define	PM_GPIO_2		1
#define	PM_GPIO_3		2
#define	PM_GPIO_4		3
#define	PM_GPIO_5		4
#define	PM_GPIO_6		5
#define	PM_GPIO_7		6
#define	PM_GPIO_8		7
#define	PM_GPIO_9		8
#define	PM_GPIO_10		9
#define	PM_GPIO_11		10

#define	PM_LPG_OUT1		0
#define	PM_LPG_OUT2		1
#define	PM_LPG_OUT3		2
#define	PM_LPG_OUT4		3
#define	PM_LPG_OUT5		4
#define	PM_LPG_OUT6		5
#define	PM_LPG_OUT7		6
#define	PM_LPG_OUT8		7

#define	SPECIAL_CONTROL_OFF	0
#define	SPECIAL_CONTROL_ON	1

#define	LED_TURN_OFF		0
#define	LED_TURN_ON		1

#define	SMEM_BUFFER_SIZE		32

#define	ONE_PARAMETER		1

#define	LED_OFF_STATE		0
#define	LED_ON_STATE		1

#define	FADE_IN_OUT_LEVEL		32

#define	MINI_UNIT		1000

#define	MAX_PWM			511
#define	MIN_PWM			0

#define	LED_SEC( time )	( time / 1000 )
#define	LED_NS( time )	( ( time % 1000 ) * 1000000 )

#define	WAIT_LOCK_TIME		50

// LED driver Command
#define	LED_COMMAND_ON_OFF		1
#define	LED_COMMAND_BLINKING		2
#define	LED_COMMAND_FADE_IN_OUT		3
#define	LED_COMMAND_ON_BRIGHTNESS		4
#define	LED_COMMAND_BLINKING_BRIGHTNESS	5
#define	LED_COMMAND_FADE_IN_OUT_INTERVAL	6
#define	LED_COMMAND_BLINKING_TIME		7
#define	LED_COMMAND_MODE			8
#define	LED_COMMAND_SPECIAL_ON_OFF		9
#define	LED_COMMAND_SPECIAL_BLINKING	10
#define	LED_COMMAND_SPECIAL_FADE_IN_OUT	11
#define	LED_COMMAND_SET_CLK_DIV_EXP	12
#define	LED_COMMAND_SET_CLK		13
#define	LED_COMMAND_SET_DIV		14
#define	LED_COMMAND_SET_EXP		15
#define	LED_COMMAND_SET_TOGGLE_LOOP_RAMP	16
#define	LED_COMMAND_SET_TOGGLE		17
#define	LED_COMMAND_SET_LOOP		18
#define	LED_COMMAND_SET_RAMP		19
#define	LED_COMMAND_SET_INVERT		20
#define	LED_COMMAND_SET_CURRENT_SINK	21
#define	LED_COMMAND_SET_PIN		22
#define	LED_COMMAND_SET_ON_OFF_LEVEL	23
#define	LED_COMMAND_SET_LPG_OUT		24
#define	LED_COMMAND_FADE_IN_OUT_BRIGHTNESS	25
#define	LED_COMMAND_SW_BLINKING		26
#define	LED_COMMAND_SW_FADE_IN_OUT		27
#define	LED_COMMAND_SPECIAL_SW_BLINKING	28
#define	LED_COMMAND_SPECIAL_SW_FADE_IN_OUT	29
#define	LED_COMMAND_SET_LUT_TABLE_RANGE	30
#define	LED_COMMAND_SET_ON_OFF_STATE	31
#define LED_COMMAND_FADE_IN_OUT_TIME_DIFF 32 //PERI-BJ-LED_FADE_IN_OUT_DIFF-00*
#define LED_COMMAND_FADE_IN_OUT_ONCE 33 //PERI-BJ-Fade_in_out_once-01+
#define LED_COMMAND_SPECIAL_FADE_IN_OUT_TIME_DIFF 34 //PERI-BJ-Add_Special_Mode-00+
#define LED_COMMAND_SPECIAL_FADE_IN_OUT_ONCE 35 //PERI-BJ-Add_Special_Mode-00+

#define	COMMAND_0_PARAMENTER		0
#define	COMMAND_1_PARAMENTER		1
#define	COMMAND_2_PARAMENTER		2
#define	COMMAND_3_PARAMENTER		3
#define	COMMAND_4_PARAMENTER		4
#define	COMMAND_5_PARAMENTER		5
#define	COMMAND_6_PARAMENTER		6
#define	COMMAND_7_PARAMENTER		7
#define	COMMAND_8_PARAMENTER		8

// Data structure
struct led_gpio_data
{
	unsigned int	msm_pin;
	unsigned int	led_on_level;
	unsigned int	led_off_level;
};

struct led_pmic_data
{
	unsigned int	command, hardware, control;
	unsigned int	pmic_pin;
	unsigned int	lpg_out;
	unsigned int	on_off_pwm;
	unsigned int	blinking_pwm1, blinking_pwm2;
	unsigned int	blinking_time1, blinking_time2;
	unsigned int	pwm_clock, pwm_div_value, pwm_div_exp;
	unsigned int	interval, toggle_up_down, ramp_loop, ramp_up_down;
	unsigned int	current_sink;
	unsigned int	invert;
	//unsigned int	use_table;
	unsigned int	fade_in_out_pwm;
	unsigned int	lut_table_start, lut_table_end;


	u16				base;
	u8				source_sel;
	u8				color;
	u8				mode_ctrl;


};

union led_detail
{
	struct led_gpio_data	gpio_data;
	struct led_pmic_data	pmic_data;
};

//PERI-BJ-Fade_in_out_diff_time-00+{
struct led_diff_data
{
	struct pwm_device 	*pwm_dev_r;
	struct pwm_device 	*pwm_dev_g;
	struct pwm_device 	*pwm_dev_b;

	struct led_pmic_data *pmic_data_r;
	struct led_pmic_data *pmic_data_g;
	struct led_pmic_data *pmic_data_b;
};
//PERI-BJ-Fade_in_out_diff_time-00+}

struct	led_data
{
	const char			*name;
	struct pwm_device 	*pwm_dev;
	struct spmi_device	*spmi_dev;
	struct mutex		lock;
	unsigned int		use_hw;
	unsigned int		special_mode : 1;
	unsigned int		on_off_state : 1;
	u16					base;
	int					max_current;
	int					id;
	bool				default_on;
	int					turn_off_delay_ms;
	
	union led_detail	detail;

	// For SW LED
	struct hrtimer		led_time;
	struct work_struct	work_blink;
	struct work_struct	work_fade_in_out;

//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*{
#ifdef LED_SW_CONTROL_SUPPORT
	// SW LED parameter
	unsigned int		sw_led_count;
	unsigned int		led_state;
	unsigned int		unit_time_out;
	int			sw_brightness;
	int			unit_brightness;
#endif
//PERI-BJ-ENABLE_DISABLE_SW_CONTROL-00*}

//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*{
	// For LED Fade in/out diff timer
	unsigned int		fade_in_interval;
	unsigned int		fade_out_interval;
	unsigned int 		in_out_state;
	struct hrtimer		led_time_diff;
	struct work_struct	work_fade_in_out_diff;
//PERI-BJ-LED_FADE_IN_OUT_DIFF-00*}

};

struct	leds_driver_data
{
	struct BS_data		*index_buffer;
	unsigned int		count;

	struct spmi_device	*spmi_dev;

	int			id;

	u8			reg;
	u8			num_leds;
	int			max_current;
	bool		default_on;
	int			turn_off_delay_ms;


	// For wake lock
	struct wake_lock		timeout_wakelock;
	struct wake_lock		normal_wakelock;
	struct early_suspend	led_early_suspend;
};

struct	led_device_data
{
	const char	*name;
	unsigned	int	use_hw;
	unsigned	int	id;
	union led_detail	detail;
};

struct	leds_device_data
{
	struct led_device_data	*device_data;
	unsigned int		count;
};

struct	command_parameter
{
	int	para1;
	int	para2;
	int	para3;
	int	para4;
	int	para5;
	int	para6;
	int	para7;
	int	para8;
};

struct	led_command_source
{
	int	command;
	void	( *function )( struct led_data*, struct command_parameter* );
	int	parameter;
};

struct	led_command_data
{
	void	( *function )( struct led_data*, struct command_parameter* );
	int	parameter;
};

struct	led_command_info
{
	struct BS_data	*commmand_buffer;
	unsigned int	count;
};

#endif
