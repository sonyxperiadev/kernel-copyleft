/*
 * LP5562 LED driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * Author: Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_data/leds-lp55xx-fxn.h>
#include <linux/slab.h>

#include <linux/wakelock.h>
#include <linux/development-tool/development_tool.h> 
#include <linux/fih_hw_info.h>
#include <linux/pm.h>
#include <linux/pinctrl/consumer.h> 

#include "leds-lp55xx-common-fxn.h"


#define LP5562_PROGRAM_LENGTH		32
#define LP5562_MAX_LEDS			4

/* ENABLE Register 00h */
#define LP5562_REG_ENABLE		0x00
#define LP5562_EXEC_ENG1_M		0x30
#define LP5562_EXEC_ENG2_M		0x0C
#define LP5562_EXEC_ENG3_M		0x03
#define LP5562_EXEC_M			0x3F
#define LP5562_MASTER_ENABLE		0x40	/* Chip master enable */
#define LP5562_LOGARITHMIC_PWM		0x80	/* Logarithmic PWM adjustment */
#define LP5562_EXEC_RUN			0x2A
#define LP5562_ENABLE_DEFAULT	\
	(LP5562_MASTER_ENABLE | LP5562_LOGARITHMIC_PWM)
#define LP5562_ENABLE_RUN_PROGRAM	\
	(LP5562_ENABLE_DEFAULT | LP5562_EXEC_RUN)

/* OPMODE Register 01h */
#define LP5562_REG_OP_MODE		0x01
#define LP5562_MODE_ENG1_M		0x30
#define LP5562_MODE_ENG2_M		0x0C
#define LP5562_MODE_ENG3_M		0x03
#define LP5562_LOAD_ENG1		0x10
#define LP5562_LOAD_ENG2		0x04
#define LP5562_LOAD_ENG3		0x01
#define LP5562_RUN_ENG1			0x20
#define LP5562_RUN_ENG2			0x08
#define LP5562_RUN_ENG3			0x02
#define LP5562_ENG1_IS_LOADING(mode)	\
	((mode & LP5562_MODE_ENG1_M) == LP5562_LOAD_ENG1)
#define LP5562_ENG2_IS_LOADING(mode)	\
	((mode & LP5562_MODE_ENG2_M) == LP5562_LOAD_ENG2)
#define LP5562_ENG3_IS_LOADING(mode)	\
	((mode & LP5562_MODE_ENG3_M) == LP5562_LOAD_ENG3)

/* BRIGHTNESS Registers */
#define LP5562_REG_R_PWM		0x04
#define LP5562_REG_G_PWM		0x03
#define LP5562_REG_B_PWM		0x02
#define LP5562_REG_W_PWM		0x0E

/* CURRENT Registers */
#define LP5562_REG_R_CURRENT		0x07
#define LP5562_REG_G_CURRENT		0x06
#define LP5562_REG_B_CURRENT		0x05
#define LP5562_REG_W_CURRENT		0x0F

/* CONFIG Register 08h */
#define LP5562_REG_CONFIG		0x08
#define LP5562_PWM_HF			0x40
#define LP5562_PWRSAVE_EN		0x20
#define LP5562_CLK_INT			0x01	/* Internal clock */
#define LP5562_DEFAULT_CFG		(LP5562_PWM_HF | LP5562_PWRSAVE_EN)

/* RESET Register 0Dh */
#define LP5562_REG_RESET		0x0D
#define LP5562_RESET			0xFF

/* PROGRAM ENGINE Registers */
#define LP5562_REG_PROG_MEM_ENG1	0x10
#define LP5562_REG_PROG_MEM_ENG2	0x30
#define LP5562_REG_PROG_MEM_ENG3	0x50

/* LEDMAP Register 70h */
#define LP5562_REG_ENG_SEL		0x70
#define LP5562_ENG_SEL_PWM		0
#define LP5562_ENG_FOR_RGB_M		0x3F
#define LP5562_ENG_SEL_RGB		0x1B	/* R:ENG1, G:ENG2, B:ENG3 */
#define LP5562_ENG_FOR_W_M		0xC0
#define LP5562_ENG1_FOR_W		0x40	/* W:ENG1 */
#define LP5562_ENG2_FOR_W		0x80	/* W:ENG2 */
#define LP5562_ENG3_FOR_W		0xC0	/* W:ENG3 */

/* Program Commands */
#define LP5562_CMD_DISABLE		0x00
#define LP5562_CMD_LOAD			0x15
#define LP5562_CMD_RUN			0x2A
#define LP5562_CMD_DIRECT		0x3F
#define LP5562_PATTERN_OFF		0

/* PERI-AH-LED-00+[ */
#define LED_MSG(fmt, args...) printk(KERN_INFO "[%s][MSG] Driver_LED : "fmt" \n", __func__, ##args)
#define LED_ERR(fmt, args...) printk(KERN_ERR "[%s][ERR] Driver_LED : "fmt" \n", __func__, ##args)
#define LED_WAN(fmt, args...) printk(KERN_WARNING"[%s][WAN] Driver_LED : "fmt" \n", __func__, ##args)

#define ONE_PARAMETER		1

// LED driver Command
#define	LED_COMMAND_ON_OFF 			1
#define	LED_COMMAND_BLINKING 		2
#define	LED_COMMAND_FADE_IN_OUT		3
#define	LED_COMMAND_ON_BRIGHTNESS			4
#define	LED_COMMAND_BLINKING_BRIGHTNESS		5
#define	LED_COMMAND_FADE_IN_OUT_INTERVAL	6
#define	LED_COMMAND_BLINKING_TIME	7
#define	LED_COMMAND_MODE			8
#define	LED_COMMAND_SPECIAL_ON_OFF		9
#define	LED_COMMAND_SPECIAL_BLINKING	10
#define	LED_COMMAND_SPECIAL_FADE_IN_OUT	11
#define	LED_COMMAND_SET_CLK_DIV_EXP		12
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
#define	LED_COMMAND_FADE_IN_OUT_TIME_DIFF 32 
#define	LED_COMMAND_FADE_IN_OUT_ONCE 33 
#define	LED_COMMAND_SPECIAL_FADE_IN_OUT_TIME_DIFF 34 
#define	LED_COMMAND_SPECIAL_FADE_IN_OUT_ONCE 35 

#define	COMMAND_0_PARAMENTER		0
#define	COMMAND_1_PARAMENTER		1
#define	COMMAND_2_PARAMENTER		2
#define	COMMAND_3_PARAMENTER		3
#define	COMMAND_4_PARAMENTER		4
#define	COMMAND_5_PARAMENTER		5
#define	COMMAND_6_PARAMENTER		6
#define	COMMAND_7_PARAMENTER		7
#define	COMMAND_8_PARAMENTER		8

#define	LED_TURN_OFF	0
#define	LED_TURN_ON		1

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
	void	( *function )( struct lp55xx_chip*, struct command_parameter* );
	int	parameter;
};

struct	led_command_data
{
	void	( *function )( struct lp55xx_chip*, struct command_parameter* );
	int	parameter;
};

struct	led_command_info
{
	struct BS_data	*commmand_buffer;
	unsigned int	count;
};

static struct control_class_device	*file_node_class;
static struct led_command_data	*command_data;
static struct led_command_info	command_info;

struct wake_lock leds_lp5562_wakelock; 
/* PERI-AH-LED-00+] */

static inline void lp5562_wait_opmode_done(void)
{
	/* operation mode change needs to be longer than 153 us */
	usleep_range(200, 300);
}

static inline void lp5562_wait_enable_done(void)
{
	/* it takes more 488 us to update ENABLE register */
	usleep_range(500, 600);
}

static void lp5562_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	u8 addr[] = {
		LP5562_REG_R_CURRENT,
		LP5562_REG_G_CURRENT,
		LP5562_REG_B_CURRENT,
		LP5562_REG_W_CURRENT,
	};

	led->led_current = led_current;
	lp55xx_write(led->chip, addr[led->chan_nr], led_current);
}

static void lp5562_load_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5562_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5562_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5562_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5562_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5562_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5562_LOAD_ENG3,
	};

	lp55xx_update_bits(chip, LP5562_REG_OP_MODE, mask[idx], val[idx]);

	lp5562_wait_opmode_done();
}

static void lp5562_stop_engine(struct lp55xx_chip *chip)
{
	lp55xx_write(chip, LP5562_REG_OP_MODE, LP5562_CMD_DISABLE);
	lp5562_wait_opmode_done();
}

static void lp5562_run_engine(struct lp55xx_chip *chip, bool start)
{
	int ret;
	u8 mode;
	u8 exec;

	/* stop engine */
	if (!start) {
		lp55xx_write(chip, LP5562_REG_ENABLE, LP5562_ENABLE_DEFAULT);
		lp5562_wait_enable_done();
		lp5562_stop_engine(chip);
		lp55xx_write(chip, LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);
		lp55xx_write(chip, LP5562_REG_OP_MODE, LP5562_CMD_DIRECT);
		lp5562_wait_opmode_done();
		return;
	}

	/*
	 * To run the engine,
	 * operation mode and enable register should updated at the same time
	 */

	ret = lp55xx_read(chip, LP5562_REG_OP_MODE, &mode);
	if (ret)
		return;

	ret = lp55xx_read(chip, LP5562_REG_ENABLE, &exec);
	if (ret)
		return;

	/* change operation mode to RUN only when each engine is loading */
	if (LP5562_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5562_MODE_ENG1_M) | LP5562_RUN_ENG1;
		exec = (exec & ~LP5562_EXEC_ENG1_M) | LP5562_RUN_ENG1;
	}

	if (LP5562_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5562_MODE_ENG2_M) | LP5562_RUN_ENG2;
		exec = (exec & ~LP5562_EXEC_ENG2_M) | LP5562_RUN_ENG2;
	}

	if (LP5562_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5562_MODE_ENG3_M) | LP5562_RUN_ENG3;
		exec = (exec & ~LP5562_EXEC_ENG3_M) | LP5562_RUN_ENG3;
	}

	lp55xx_write(chip, LP5562_REG_OP_MODE, mode);
	lp5562_wait_opmode_done();

	lp55xx_update_bits(chip, LP5562_REG_ENABLE, LP5562_EXEC_M, exec);
	lp5562_wait_enable_done();
}

static int lp5562_update_firmware(struct lp55xx_chip *chip,
					const u8 *data, size_t size)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 pattern[LP5562_PROGRAM_LENGTH] = {0};
	u8 addr[] = {
		[LP55XX_ENGINE_1] = LP5562_REG_PROG_MEM_ENG1,
		[LP55XX_ENGINE_2] = LP5562_REG_PROG_MEM_ENG2,
		[LP55XX_ENGINE_3] = LP5562_REG_PROG_MEM_ENG3,
	};
	unsigned cmd;
	char c[3];
	int program_size;
	int nrchars;
	int offset = 0;
	int ret;
	int i;

	/* clear program memory before updating */
	for (i = 0; i < LP5562_PROGRAM_LENGTH; i++)
		lp55xx_write(chip, addr[idx] + i, 0);

	i = 0;
	while ((offset < size - 1) && (i < LP5562_PROGRAM_LENGTH)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(data + offset, "%2s%n ", c, &nrchars);
		if (ret != 1)
			goto err;

		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto err;

		pattern[i] = (u8)cmd;
		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if (i % 2)
		goto err;

	program_size = i;
	for (i = 0; i < program_size; i++)
		lp55xx_write(chip, addr[idx] + i, pattern[i]);

	return 0;

err:
	dev_err(&chip->cl->dev, "wrong pattern format\n");
	return -EINVAL;
}

static void lp5562_firmware_loaded(struct lp55xx_chip *chip)
{
	const struct firmware *fw = chip->fw;

	if (fw->size > LP5562_PROGRAM_LENGTH) {
		dev_err(&chip->cl->dev, "firmware data size overflow: %zu\n",
			fw->size);
		return;
	}

	/*
	 * Program momery sequence
	 *  1) set engine mode to "LOAD"
	 *  2) write firmware data into program memory
	 */

	lp5562_load_engine(chip);
	lp5562_update_firmware(chip, fw->data, fw->size);
}

static int lp5562_post_init_device(struct lp55xx_chip *chip)
{
	int ret;
	u8 cfg = LP5562_DEFAULT_CFG;
	unsigned int hw_id = 0; /* PERI-AH-change to use extclk-00+ */

	/* Set all PWMs to direct control mode */
	ret = lp55xx_write(chip, LP5562_REG_OP_MODE, LP5562_CMD_DIRECT);
	if (ret)
		return ret;

	lp5562_wait_opmode_done();

	/* PERI-AH-change to use extclk-00+[ */
	hw_id = fih_get_product_phase();
    
    if (hw_id > PHASE_Pre_SP) {
		LED_MSG(" external clock used\n");
	}
    else {
        /* Update configuration for the clock setting 
          in tulip devm_clk_get always failed so skip this.*/
        //if (!lp55xx_is_extclk_used(chip)) 
		cfg |= LP5562_CLK_INT;
        LED_MSG(" internal clock used\n");
	}
	/* PERI-AH-change to use extclk-00+] */

	ret = lp55xx_write(chip, LP5562_REG_CONFIG, cfg);
	if (ret)
		return ret;

	/* Initialize all channels PWM to zero -> leds off */
	lp55xx_write(chip, LP5562_REG_R_PWM, 0);
	lp55xx_write(chip, LP5562_REG_G_PWM, 0);
	lp55xx_write(chip, LP5562_REG_B_PWM, 0);
	lp55xx_write(chip, LP5562_REG_W_PWM, 0);

	/* Set LED map as register PWM by default */
	lp55xx_write(chip, LP5562_REG_ENG_SEL, LP5562_ENG_SEL_PWM);

	return 0;
}

static void lp5562_led_brightness_work(struct work_struct *work)
{
	struct lp55xx_led *led = container_of(work, struct lp55xx_led,
					      brightness_work);
	struct lp55xx_chip *chip = led->chip;
	u8 addr[] = {
		LP5562_REG_R_PWM,
		LP5562_REG_G_PWM,
		LP5562_REG_B_PWM,
		LP5562_REG_W_PWM,
	};

	mutex_lock(&chip->lock);
	lp55xx_write(chip, addr[led->chan_nr], led->brightness);
	mutex_unlock(&chip->lock);
}

static void lp5562_write_program_memory(struct lp55xx_chip *chip,
					u8 base, const u8 *rgb, int size)
{
	int i;

	if (!rgb || size <= 0)
		return;

	for (i = 0; i < size; i++)
		lp55xx_write(chip, base + i, *(rgb + i));

	lp55xx_write(chip, base + i, 0);
	lp55xx_write(chip, base + i + 1, 0);
}

/* check the size of program count */
static inline bool _is_pc_overflow(struct lp55xx_predef_pattern *ptn)
{
	return (ptn->size_r >= LP5562_PROGRAM_LENGTH ||
		ptn->size_g >= LP5562_PROGRAM_LENGTH ||
		ptn->size_b >= LP5562_PROGRAM_LENGTH);
}

static int lp5562_run_predef_led_pattern(struct lp55xx_chip *chip, int mode)
{
	struct lp55xx_predef_pattern *ptn;
	int i;

	if (mode == LP5562_PATTERN_OFF) {
		lp5562_run_engine(chip, false);
		return 0;
	}

	ptn = chip->pdata->patterns + (mode - 1);
	if (!ptn || _is_pc_overflow(ptn)) {
		dev_err(&chip->cl->dev, "invalid pattern data\n");
		return -EINVAL;
	}

	lp5562_stop_engine(chip);

	/* Set LED map as RGB */
	lp55xx_write(chip, LP5562_REG_ENG_SEL, LP5562_ENG_SEL_RGB);

	/* Load engines */
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		chip->engine_idx = i;
		lp5562_load_engine(chip);
	}

	/* Clear program registers */
	lp55xx_write(chip, LP5562_REG_PROG_MEM_ENG1, 0);
	lp55xx_write(chip, LP5562_REG_PROG_MEM_ENG1 + 1, 0);
	lp55xx_write(chip, LP5562_REG_PROG_MEM_ENG2, 0);
	lp55xx_write(chip, LP5562_REG_PROG_MEM_ENG2 + 1, 0);
	lp55xx_write(chip, LP5562_REG_PROG_MEM_ENG3, 0);
	lp55xx_write(chip, LP5562_REG_PROG_MEM_ENG3 + 1, 0);

	/* Program engines */
	lp5562_write_program_memory(chip, LP5562_REG_PROG_MEM_ENG1,
				ptn->r, ptn->size_r);
	lp5562_write_program_memory(chip, LP5562_REG_PROG_MEM_ENG2,
				ptn->g, ptn->size_g);
	lp5562_write_program_memory(chip, LP5562_REG_PROG_MEM_ENG3,
				ptn->b, ptn->size_b);

	/* Run engines */
	lp5562_run_engine(chip, true);

	return 0;
}

static ssize_t lp5562_store_pattern(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_predef_pattern *ptn = chip->pdata->patterns;
	int num_patterns = chip->pdata->num_patterns;
	unsigned long mode;
	int ret;

	pr_info("lp5562_store_pattern num_patterns=%d", num_patterns);
        
    ret = kstrtoul(buf, 0, &mode);
	if (ret)
		return ret;

	if (mode > num_patterns || !ptn)
		return -EINVAL;

	mutex_lock(&chip->lock);
	ret = lp5562_run_predef_led_pattern(chip, mode);
	mutex_unlock(&chip->lock);

	if (ret)
		return ret;

	return len;
}

static ssize_t lp5562_store_engine_mux(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	u8 mask;
	u8 val;

	/* LED map
	 * R ... Engine 1 (fixed)
	 * G ... Engine 2 (fixed)
	 * B ... Engine 3 (fixed)
	 * W ... Engine 1 or 2 or 3
	 */

	if (sysfs_streq(buf, "RGB")) {
		mask = LP5562_ENG_FOR_RGB_M;
		val = LP5562_ENG_SEL_RGB;
	} else if (sysfs_streq(buf, "W")) {
		enum lp55xx_engine_index idx = chip->engine_idx;

		mask = LP5562_ENG_FOR_W_M;
		switch (idx) {
		case LP55XX_ENGINE_1:
			val = LP5562_ENG1_FOR_W;
			break;
		case LP55XX_ENGINE_2:
			val = LP5562_ENG2_FOR_W;
			break;
		case LP55XX_ENGINE_3:
			val = LP5562_ENG3_FOR_W;
			break;
		default:
			return -EINVAL;
		}

	} else {
		dev_err(dev, "choose RGB or W\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	lp55xx_update_bits(chip, LP5562_REG_ENG_SEL, mask, val);
	mutex_unlock(&chip->lock);

	return len;
}

static LP55XX_DEV_ATTR_WO(led_pattern, lp5562_store_pattern);
static LP55XX_DEV_ATTR_WO(engine_mux, lp5562_store_engine_mux);

static struct attribute *lp5562_attributes[] = {
	&dev_attr_led_pattern.attr,
	&dev_attr_engine_mux.attr,
	NULL,
};

static const struct attribute_group lp5562_group = {
	.attrs = lp5562_attributes,
};

/* Chip specific configurations */
static struct lp55xx_device_config lp5562_cfg = {
	.max_channel  = LP5562_MAX_LEDS,
	.reset = {
		.addr = LP5562_REG_RESET,
		.val  = LP5562_RESET,
	},
	.enable = {
		.addr = LP5562_REG_ENABLE,
		.val  = LP5562_ENABLE_DEFAULT,
	},
	.post_init_device   = lp5562_post_init_device,
	.set_led_current    = lp5562_set_led_current,
	.brightness_work_fn = lp5562_led_brightness_work,
	.run_engine         = lp5562_run_engine,
	.firmware_cb        = lp5562_firmware_loaded,
	.dev_attr_group     = &lp5562_group,
};

static ssize_t lp5562_write_pattern(struct file *filp,
				    struct kobject *kobj,
				    struct bin_attribute *attr,
				    char *buf, loff_t offset,
				    size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	size_t len = LP5562_PROGRAM_LENGTH * 2 + 1;

	if (size > len) {
		dev_err(&chip->cl->dev, "pattern data size overflow: %zu\n",
			size);
		return -EINVAL;
	}

	/*
	 * Program momery sequence
	 *  1) set engine mode to "LOAD"
	 *  2) write firmware data into program memory
	 */

	lp5562_load_engine(chip);
	lp5562_update_firmware(chip, buf, size);

	return size;
}

static struct bin_attribute lp5562_pattern_attr = {
	.attr  = {
		.name = "pattern_data",
		.mode = S_IWUSR,
	},
	.size  = 0,
	.write = lp5562_write_pattern,
};

static int lp5562_add_bin_attr(struct lp55xx_chip *chip)
{
	return device_create_bin_file(&chip->cl->dev, &lp5562_pattern_attr);
}

static void lp5562_remove_bin_attr(struct lp55xx_chip *chip)
{
	device_remove_bin_file(&chip->cl->dev, &lp5562_pattern_attr);
}

/* PERI-AH-LED-00+[ */
static void	led_on_off_check_mode( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    char buf[LP5562_PROGRAM_LENGTH];
    int size;
    int led_idx= chip->engine_idx-1;  

	LED_MSG( "led: %d, para1: %d, brightness: %d ", chip->engine_idx, parameter->para1, chip->sw_brightness[led_idx]); 

	mutex_lock(&chip->lock);

	if (parameter->para1 == LED_TURN_ON) {

        size= snprintf( buf, PAGE_SIZE, "40%2x", chip->sw_brightness[led_idx]);
        LED_MSG("[%s] is a string %d chars long\n", buf, size); 
        
        /* Set LED map as RGB */
    	lp55xx_update_bits(chip, LP5562_REG_ENG_SEL, LP5562_ENG_FOR_RGB_M, LP5562_ENG_SEL_RGB);

        /* Load engines */
		lp5562_load_engine(chip);

		/* load pattern */
		lp5562_update_firmware(chip, buf, size); 

	}

	mutex_unlock(&chip->lock);
 
}

static void	led_on_brightness_set( struct lp55xx_chip *chip, struct command_parameter *parameter )
{

	LED_MSG( "led: %d, para1: %d ", chip->engine_idx, parameter->para1);
    chip->sw_brightness[chip->engine_idx-1]= parameter->para1;
}

static void	led_set_on_off_state( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    LED_MSG( "ignore.");
}

static void	led_fade_in_out_interval_set( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    LED_MSG( "ignore.");
}

static void	led_blinking_brightness_set( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    LED_MSG( "led: %d, para1: %d, para2: %d", chip->engine_idx, parameter->para1, parameter->para2);
    chip->sw_brightness[chip->engine_idx-1]= parameter->para1;
}

static void	led_blinking_time_set( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    LED_MSG( "led: %d, para1: %d, para2: %d", chip->engine_idx, parameter->para1, parameter->para2);
	chip->led_on_interval[chip->engine_idx-1]= parameter->para1;
    chip->led_off_interval[chip->engine_idx-1]= parameter->para2;
}

static void	led_blinking_check_mode( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    char buf[LP5562_PROGRAM_LENGTH];
    int size;
    int led_idx= chip->engine_idx-1; 

	LED_MSG( "led: %d, para1: %d ", chip->engine_idx, parameter->para1);
    LED_MSG( "brightness: %d on-interval: %d, off-interval: %d", chip->sw_brightness[led_idx], chip->led_on_interval[led_idx], chip->led_off_interval[led_idx]);

	mutex_lock(&chip->lock);

	if (parameter->para1 == LED_TURN_ON) {

        size= snprintf( buf, PAGE_SIZE, "460040%2x460040007F00A2A4", chip->sw_brightness[led_idx]);
        LED_MSG("[%s] is a string %d chars long\n", buf, size); 
        
        /* Set LED map as RGB */
    	lp55xx_update_bits(chip, LP5562_REG_ENG_SEL, LP5562_ENG_FOR_RGB_M, LP5562_ENG_SEL_RGB);

        /* Load engines */
		lp5562_load_engine(chip);

		/* load pattern */
		lp5562_update_firmware(chip, buf, size); 

	}

	mutex_unlock(&chip->lock);
 }

static void	led_fade_in_out_brightness_set( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    LED_MSG( "led: %d, para1: %d", chip->engine_idx, parameter->para1);
    chip->sw_brightness[chip->engine_idx-1]= parameter->para1;
}

static void	led_set_toggle_loop_ramp( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    LED_MSG( "ignore.");
}

static void	led_fade_in_out_check_mode( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    char buf[LP5562_PROGRAM_LENGTH];
    int size;
    int led_idx= chip->engine_idx-1;

	LED_MSG( "led: %d, para1: %d ", chip->engine_idx, parameter->para1);
    LED_MSG( "brightness: %d on-interval: %d, off-interval: %d", chip->sw_brightness[led_idx], chip->led_on_interval[led_idx], chip->led_off_interval[led_idx]);

	mutex_lock(&chip->lock);

	if (parameter->para1 == LED_TURN_ON) {

        size= snprintf( buf, PAGE_SIZE, "047E047E08FE08FE");
        LED_MSG("[%s] is a string %d chars long\n", buf, size); 
        
        /* Set LED map as RGB */
    	lp55xx_update_bits(chip, LP5562_REG_ENG_SEL, LP5562_ENG_FOR_RGB_M, LP5562_ENG_SEL_RGB);

        /* Load engines */
		lp5562_load_engine(chip);

		/* load pattern */
		lp5562_update_firmware(chip, buf, size); 

	}

	mutex_unlock(&chip->lock);
}

static void	led_fade_in_out_once_check_mode( struct lp55xx_chip *chip, struct command_parameter *parameter )
{
    char buf[LP5562_PROGRAM_LENGTH*2];
    int size;
    int led_idx= chip->engine_idx-1;    

	LED_MSG( "led: %d, para1: %d ", chip->engine_idx, parameter->para1);
    LED_MSG( "brightness: %d on-interval: %d, off-interval: %d", chip->sw_brightness[led_idx], chip->led_on_interval[led_idx], chip->led_off_interval[led_idx]);

	mutex_lock(&chip->lock);

	if (parameter->para1 == LED_TURN_ON) {

        size= snprintf( buf, PAGE_SIZE, "047F047F08FF08FFB380C000");
        
        LED_MSG("[%s] is a string %d chars long\n", buf, size); 
        
        /* Set LED map as RGB */
    	lp55xx_update_bits(chip, LP5562_REG_ENG_SEL, LP5562_ENG_FOR_RGB_M, LP5562_ENG_SEL_RGB);

        /* Load engines */
		lp5562_load_engine(chip);

		/* load pattern */
		lp5562_update_firmware(chip, buf, size); 

	}

	mutex_unlock(&chip->lock);
}

static int prepare_commands( void )
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
		{ LED_COMMAND_SPECIAL_ON_OFF, led_on_off_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_TOGGLE_LOOP_RAMP, led_set_toggle_loop_ramp, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_BRIGHTNESS, led_fade_in_out_brightness_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_ON_OFF_STATE, led_set_on_off_state, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_ONCE , led_fade_in_out_once_check_mode, COMMAND_1_PARAMENTER },
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

	struct lp55xx_chip	*data		= node_data;
	unsigned int		pointer		= 0;
	int					rem_size	= size;
	int 				i;

    wake_lock(&leds_lp5562_wakelock);
    LED_MSG( "leds lp5562 wakelock");
    
    //stop all of engine
    lp5562_run_engine(data, false);

    //initial data
    for (i=0; i<LP55XX_ENGINE_MAX; i++) {
	    data->sw_brightness[i] = 255;
	    data->led_on_interval[i] = 100;
	    data->led_off_interval[i] = 6000;
	}
    
	while( pointer < rem_size )
	{
		struct BS_data		*search_data;
		struct led_command_data	*command;
		unsigned int		char_count;
		int			user_command, led_id;

		//user_command
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

        //led_id 
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

		led_id= led_id+1; 
        if( !( led_id > LP55XX_ENGINE_INVALID && led_id <= LP55XX_ENGINE_MAX) )
		{
			LED_MSG( "Can't support led id(%d)", led_id );
			break;
		}

        data->engine_idx = led_id;         
        
		//parameter
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
				command->function( data, &parameter );
			else
			{
				LED_MSG( "Can't get parameter" );
				break;
			}
		}
	}

    /* Run engines */
    lp5562_run_engine(data, true);

	wake_unlock(&leds_lp5562_wakelock);

	return size;
}

static int led_pinctrl_configure(struct lp55xx_chip *chip, bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state =
			pinctrl_lookup_state(chip->led_pinctrl,
						"tlmm_lp5562_active");
		if (IS_ERR(set_state)) {
			LED_ERR("cannot get lp5562 pinctrl active state\n");
			return PTR_ERR(set_state);
		}
	} else {
		set_state =
			pinctrl_lookup_state(chip->led_pinctrl,
						"tlmm_lp5562_suspend");
		if (IS_ERR(set_state)) {
			LED_ERR("cannot get lp5562 pinctrl sleep state\n");
			return PTR_ERR(set_state);
		}
	}
    
	retval = pinctrl_select_state(chip->led_pinctrl, set_state);
	if (retval) {
		LED_ERR("cannot set lp5562 pinctrl active state\n");
		return retval;
	}

	return 0;
}
/* PERI-AH-LED-00+] */

static int lp5562_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct lp55xx_chip *chip;
	struct lp55xx_led *led;
	struct lp55xx_platform_data *pdata;
	struct device_node *np = client->dev.of_node;

    struct control_node_load	file_node; /* PERI-AH-LED-00+ */

	if (!dev_get_platdata(&client->dev)) {
		if (np) {
			ret = lp55xx_of_populate_pdata(&client->dev, np);
			if (ret < 0)
				return ret;
		} else {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}
	}
	pdata = dev_get_platdata(&client->dev);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	led = devm_kzalloc(&client->dev,
			sizeof(*led) * pdata->num_channels, GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	chip->cl = client;
	chip->pdata = pdata;
	chip->cfg = &lp5562_cfg;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, led);

	ret = lp55xx_init_device(chip);
	if (ret)
		goto err_init;

	ret = lp55xx_register_leds(led, chip);
	if (ret)
		goto err_register_leds;

	ret = lp55xx_register_sysfs(chip);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto err_register_sysfs;
	}

	ret = lp5562_add_bin_attr(chip);
	if (ret) {
		dev_err(&client->dev, "binary attributes err: %d\n", ret);
		goto err_bin_attr;
	}

    /* PERI-AH-LED-00+[ */
	/* Get pinctrl if target uses pinctrl */
	chip->led_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(chip->led_pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
	}

	if (chip->led_pinctrl) {
		ret = led_pinctrl_configure(chip, true);
		if (ret) {
			dev_err(&client->dev, "cannot set lp5562 pinctrl active state\n");
		}
	}

	if( prepare_commands() )
	{
		// Create file node in sys/class/led/fih_led
		file_node.class_name	= "led";
		file_node.device_name	= "fih_led";
		file_node.file_node_data  = chip;
		file_node.control_read	= led_command;
		file_node.control_write	= led_control;
		file_node_class			= control_file_node_register( &file_node );
	} 
    
    wake_lock_init( &leds_lp5562_wakelock, WAKE_LOCK_SUSPEND, "leds-lp55xx-fxn" ); 
	/* PERI-AH-LED-00+] */

	return 0;
    
err_bin_attr:
    lp55xx_unregister_sysfs(chip);
err_register_sysfs:
	lp55xx_unregister_leds(led, chip);
err_register_leds:
	lp55xx_deinit_device(chip);
err_init:
	return ret;
}

static int lp5562_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp5562_stop_engine(chip);

    lp5562_remove_bin_attr(chip);
	lp55xx_unregister_sysfs(chip);
	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

    /* PERI-AH-LED-00+[ */
	control_file_node_unregister( file_node_class );
    wake_lock_destroy(&leds_lp5562_wakelock); 
    /* PERI-AH-LED-00+] */    

	return 0;
}

/* PERI-AH-LED-00+[ */
#ifdef CONFIG_PM_SLEEP
static int lp5562_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);  
	struct lp55xx_led *led = i2c_get_clientdata(client);
	int ret;

	if (led->chip->led_pinctrl) {
		ret = led_pinctrl_configure(led->chip, false);
		if (ret) {
			LED_ERR("failed to put the pin in suspend state\n");
			return ret;
		}
	}
    LED_MSG("success to put the pin in suspend state\n");
    
	return 0;
}

static int lp5562_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);  
	struct lp55xx_led *led = i2c_get_clientdata(client);
	int ret;

	if (led->chip->led_pinctrl) {
		ret = led_pinctrl_configure(led->chip, true);
		if (ret) {
			LED_ERR("failed to put the pin in resume state\n");
			return ret;
		}
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lp5562_pm, lp5562_suspend,          lp5562_resume); 

/* PERI-AH-LED-00+] */

static const struct i2c_device_id lp5562_id[] = {
	{ "lp5562", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp5562_id);

#ifdef CONFIG_OF
static const struct of_device_id of_lp5562_leds_match[] = {
	{ .compatible = "ti,lp5562", },
	{},
};

MODULE_DEVICE_TABLE(of, of_lp5562_leds_match);
#endif

static struct i2c_driver lp5562_driver = {
	.driver = {
		.name	= "lp5562",
		.pm = &lp5562_pm,            
		.of_match_table = of_match_ptr(of_lp5562_leds_match),
	},
	.probe		= lp5562_probe,
	.remove		= lp5562_remove,
	.id_table	= lp5562_id,
};

module_i2c_driver(lp5562_driver);

MODULE_DESCRIPTION("Texas Instruments LP5562 LED Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
