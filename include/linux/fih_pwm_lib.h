/********************************************************************
@
@ Foxconn, Inc
@
@ 20120901, OwenHuang, Initial Version
@ 20121019, BennetJau  , Modify for PM8038 support
@ 20130722, BennetJau  , Modify fro PM8226 support and referenc from QCT's QPNP_LED solution
********************************************************************/
#include <linux/fih_leds.h>
#include <linux/qpnp/pwm.h>

#define	PM_PWM_LUT_SIZE			64 //PERI-BJ-Fade_in_out_diff_time-00+

/*LED action type*/
enum {
LED_ACTION_ON = 0,
LED_ACTION_OFF,
LED_ACTION_BLINK_ON,
LED_ACTION_BLINK_OFF,
LED_ACTION_FADE_ON,
LED_ACTION_FADE_OFF,
LED_ACTION_MAX,
};

/*Hardware PIN type*/
enum {
TYPE_PMIC_GPIO = 0,
TYPE_PMIC_MPP,
TYPE_GPIO,
TYPE_PMIC_LPG,
TYPE_MAX,
};

//PERI-BJ-SetCurrent-00+{
/*Dedicated RGB PIN current*/
enum{
RGB_0mA = 0,
RGB_4mA,
RGB_8mA,
RGB_12mA,
};
//PERI-BJ-SetCurrent-00+}

enum{
COLOR_RED = 0,
COLOR_GREEN,
COLOR_BLUE,
};


/* SPMI LPG registers */
enum qpnp_lpg_registers_list {
	QPNP_LPG_PATTERN_CONFIG,
	QPNP_LPG_PWM_SIZE_CLK,
	QPNP_LPG_PWM_FREQ_PREDIV_CLK,
	QPNP_LPG_PWM_TYPE_CONFIG,
	QPNP_PWM_VALUE_LSB,
	QPNP_PWM_VALUE_MSB,
	QPNP_ENABLE_CONTROL,
	QPNP_RAMP_CONTROL,
	QPNP_RAMP_STEP_DURATION_LSB = QPNP_RAMP_CONTROL + 9,
	QPNP_RAMP_STEP_DURATION_MSB,
	QPNP_PAUSE_HI_MULTIPLIER_LSB,
	QPNP_PAUSE_HI_MULTIPLIER_MSB,
	QPNP_PAUSE_LO_MULTIPLIER_LSB,
	QPNP_PAUSE_LO_MULTIPLIER_MSB,
	QPNP_HI_INDEX,
	QPNP_LO_INDEX,
	QPNP_TOTAL_LPG_SPMI_REGISTERS
};

/* LPG revisions */
enum qpnp_lpg_revision {
	QPNP_LPG_REVISION_0 = 0x0,
	QPNP_LPG_REVISION_1 = 0x1,
};


struct qpnp_lut_config {
	u8	*duty_pct_list;
	int	list_len;
	int	lo_index;
	int	hi_index;
	int	lut_pause_hi_cnt;
	int	lut_pause_lo_cnt;
	int	ramp_step_ms;
	bool	ramp_direction;
	bool	pattern_repeat;
	bool	ramp_toggle;
	bool	enable_pause_hi;
	bool	enable_pause_lo;
};

struct qpnp_lpg_config {
	struct qpnp_lut_config	lut_config;
	u16			base_addr;
	u16			lut_base_addr;
	u16			lut_size;
};

struct qpnp_pwm_config {
	int				channel_id;
	bool				in_use;
	const char			*lable;
	int				pwm_value;
	int				pwm_period;
	int				pwm_duty;
	struct pwm_period_config	period;
};

/* Public facing structure */
struct pwm_device {
	struct qpnp_lpg_chip	*chip;
	struct qpnp_pwm_config	pwm_config;
};

struct qpnp_lpg_chip {
	struct	spmi_device	*spmi_dev;
	struct	pwm_device	pwm_dev;
	spinlock_t		lpg_lock;
	struct	qpnp_lpg_config	lpg_config;
	u8	qpnp_lpg_registers[QPNP_TOTAL_LPG_SPMI_REGISTERS];
	enum qpnp_lpg_revision	revision;
};



/*For leds driver use, it could control PMIC GPIO/PMIC GPIO/MPP/Pure LPG function*/
int common_control_leds(struct spmi_device* spmi_dev, struct pwm_device *pwm_dev, struct led_pmic_data *pmic_data);
int control_fade_nonsync_leds(struct spmi_device* spmi_dev, struct led_diff_data* diff_data);//PERI-BJ-Fade_in_out_diff_time-00+

