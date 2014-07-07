/*
 * leds-as3665.c - AS3665 LED driver
 *
 * Copyright (C) 2012 ams AG
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Byron Shi <byron.shi@ams.com>
 * Author: Yu Hongxing <Hongxing2.Yu@SonyMobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/leds-as3665.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define AS3665_REG_EXEC_ENABLE			0x00
#define AS3665_REG_EXEC_MODE			0x01
#define AS3665_REG_LED_CTRL1			0x02
#define AS3665_REG_LED_CTRL2			0x03
#define AS3665_REG_GPO_CTRL			0x04
#define AS3665_REG_CP_CTRL			0x05
#define AS3665_REG_CP_MODE_SWITCH		0x06
#define AS3665_REG_SUPERVISION			0x08
#define AS3665_REG_ADC_CTRL			0x09
#define AS3665_REG_ADC_MSB			0x0A
#define AS3665_REG_ADC_LSB			0x0B
#define AS3665_REG_INT_STATUS			0x0C
#define AS3665_REG_INT_MASK			0x0D
#define AS3665_REG_TEMP_SENSE_CTRL		0x0E
#define AS3665_REG_VARIABLE_D			0x0F
#define AS3665_REG_LED_CURRENT_BASE		0x10
#define AS3665_REG_LED_MAXCURR_BASE		0x19
#define AS3665_REG_AUDIO_CTRL			0x1C
#define AS3665_REG_AUDIO_AGC			0x1D
#define AS3665_REG_LED_TEMP			0x1F
#define AS3665_REG_RESET_CTRL			0x3C
#define AS3665_REG_CHIP_ID1			0x3D
#define AS3665_REG_CHIP_ID2			0x3E
#define AS3665_REG_PAGE_SEL			0x5F
#define AS3665_REG_PROG_MEM_BASE		0x60
#define AS3665_REG_LED_PWM_BASE			0x80
#define AS3665_REG_LED_PWM_GPO			0x8F
#define AS3665_REG_FADER1			0x9B
#define AS3665_REG_FADER2			0x9C
#define AS3665_REG_FADER3			0x9D
#define AS3665_REG_LED_SETUP_BASE		0xA0
#define AS3665_REG_SEQ1_PROG_START		0xB0
#define AS3665_REG_SEQ2_PROG_START		0xB1
#define AS3665_REG_SEQ3_PROG_START		0xB2
#define AS3665_REG_SEQ1_PC			0xB4
#define AS3665_REG_SEQ2_PC			0xB5
#define AS3665_REG_SEQ3_PC			0xB6
#define AS3665_REG_VARIABLE_A1			0xB8
#define AS3665_REG_VARIABLE_A2			0xB9
#define AS3665_REG_VARIABLE_A3			0xBA
#define AS3665_REG_VARIABLE_C			0xBB
#define AS3665_REG_VARIABLE_B1			0xBC
#define AS3665_REG_VARIABLE_B2			0xBD
#define AS3665_REG_VARIABLE_B3			0xBE
#define AS3665_REG_SRAM_BASE			0XD0
#define AS3665_REG_PROG_DIRECT_ACCESS		0xFE

#define AS3665_ENABLE				0x40
#define AS3665_RAM_INIT				0x80

#define AS3665_CMD_DISABLED			0x00 /* 00000000 */
#define AS3665_CMD_RELOAD			0x15 /* 00010101 */
#define AS3665_CMD_RUN				0x2A /* 00101010 */

#define AS3665_CMD_HOLD				0x00 /* 00000000 */
#define AS3665_CMD_STEP				0x15 /* 00010101 */
#define AS3665_CMD_STEP_IN_PLACE		0x3F /* 00111111 */

#define AS3665_CP_1				0x00
#define AS3665_CP_1_5				0x01
#define AS3665_CP_2				0x02
#define AS3665_CP_1_15_UPDOWN			0x00
#define AS3665_CP_1_15_UP			0x04
#define AS3665_CP_1_15_2_UP			0x08
#define AS3665_CP_MANUAL			0x0C
#define AS3665_CP_AUTO_ON			0x10
#define AS3665_CP_ON				0x20

#define AS3665_LED7_ON_CP			0x01
#define AS3665_LED8_ON_CP			0x02
#define AS3665_LED9_ON_CP			0x04
#define AS3665_CP_MAX_5V4			0x08
#define AS3665_CP_SKIP_ON			0x10
#define AS3665_CP_AUTO_RESET			0x20

#define AS3665_AUTO_SHUTDOWN			0x80

#define AS3665_AUDIO_ON				0x01
#define AS3665_AUDIO_CMDSET			0x02
#define AS3665_AUDIO_BUF_ON			0x04
#define AS3665_AUDIO_BUF_GAIN_MASK		0x38
#define AS3665_AUDIO_AGC_CTRL_MASK		0x07
#define AS3665_AUDIO_AGC_TIME_MASK		0x18

#define AS3665_INT_CLK				0x00
#define AS3665_EXT_CLK				0x40

#define AS3665_FORCE_RESET			0x01

#define AS3665_PROGRAM_PAGE_LENGTH		32 /* in bytes */
#define AS3665_PROGRAM_PAGES			6
#define AS3665_PROGRAM_LENGTH			192 /* in bytes */

#define AS3665_LEDS				9
#define AS3665_SEQS				3

#define AS3665_SEQ_MASK_BASE			0x03 /* 00000011 */

#define AS3665_SEQ_STATUS_MASK			0x07 /* 00000111 */
#define SHIFT_MASK(id)				(((id) - 1) * 2)

struct as3665_sequencer {
	int		id;
	u8		mode;
	u8		run_mode;
	u8		sequencer_mask;
};

struct as3665_audio {
	u8		enable;
	u8		buf_gain;
	u8		agc_ctrl;
	u8		agc_time;
};

struct as3665_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	u8			startup_current;
	struct led_classdev     cdev;
	u8			brightness;
};

struct as3665_chip {
	struct mutex		lock; /* Serialize control */
	struct i2c_client	*client;
	struct as3665_sequencer	sequencers[AS3665_SEQS];
	struct as3665_audio audio;
	struct as3665_led	leds[AS3665_LEDS];
	struct as3665_platform_data *pdata;
	u8			num_channels;
	u8			num_leds;
	u8			enable;
};

static inline struct as3665_led *cdev_to_led(struct led_classdev *cdev)
{
	return container_of(cdev, struct as3665_led, cdev);
}

static inline struct as3665_chip *
sequencer_to_as3665(struct as3665_sequencer *sequencer)
{
	return container_of(sequencer, struct as3665_chip,
			    sequencers[sequencer->id - 1]);
}

static inline struct as3665_chip *led_to_as3665(struct as3665_led *led)
{
	return container_of(led, struct as3665_chip, leds[led->id]);
}

static int as3665_write(struct i2c_client *client, u8 reg, u8 value)
{
	s32 ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, " err %d writing reg %d, value %d\n",
				ret, reg, value);

	return ret;
}

static int as3665_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, " err %d reading reg %d\n", ret, reg);
		return ret;
	}

	*buf = ret;
	return 0;
}

static int as3665_configure(struct i2c_client *client)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	/* disable sequencers */
	ret = as3665_write(client, AS3665_REG_EXEC_ENABLE, AS3665_CMD_DISABLED);
	if (ret < 0)
		goto fail;

	chip->enable = 0;

	/* disable audio input */
	ret = as3665_write(client, AS3665_REG_AUDIO_CTRL, 0x00);
	if (ret < 0)
		goto fail;

	/* turn on all leds */
	ret = as3665_write(client, AS3665_REG_LED_CTRL1, 0xFF);
	if (ret < 0)
		goto fail;
	ret = as3665_write(client, AS3665_REG_LED_CTRL2, 0x01);

fail:
	return ret;
}

static int as3665_chipen_check(struct i2c_client *client)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	u8 chipstate, cpstate, ledson = 0;
	int ret = 0, i;

	ret = as3665_read(client, AS3665_REG_EXEC_ENABLE, &chipstate);
	if (ret < 0)
		goto fail;
	ret = as3665_read(client, AS3665_REG_CP_CTRL, &cpstate);
	if (ret < 0)
		goto fail;

	/* check sequencer mode */
	for (i = 0; i < 3; i++)
		ledson |= chip->sequencers[i].mode;

	/* check all leds' brightness */
	for (i = 0; i < chip->num_leds; i++)
		ledson |= chip->leds[i].brightness;

	if (ledson && !chip->enable) {
		ret = as3665_write(client, AS3665_REG_EXEC_ENABLE,
					chipstate | AS3665_ENABLE);
		if (ret < 0)
			goto fail;
		ret = as3665_write(client, AS3665_REG_CP_CTRL,
					cpstate | AS3665_CP_AUTO_ON);
		if (ret < 0)
			goto fail;

		chip->enable = 1;
	} else if (!ledson && chip->enable) {
		ret = as3665_write(client, AS3665_REG_CP_CTRL,
					cpstate & ~AS3665_CP_AUTO_ON);
		if (ret < 0)
			goto fail;
		ret = as3665_write(client, AS3665_REG_EXEC_ENABLE,
					chipstate & ~AS3665_ENABLE);
		if (ret < 0)
			goto fail;

		chip->enable = 0;

		for (i = 0; i < chip->num_leds; i++) {
			ret = as3665_write(client,
				AS3665_REG_LED_PWM_BASE + chip->leds[i].chan_nr,
				0x00);
			if (ret < 0)
				goto fail;
		}
	}

fail:
	return ret;
}

static int as3665_set_sequencer_mode(struct as3665_sequencer *sequencer,
				u8 mode)
{
	struct as3665_chip *chip = sequencer_to_as3665(sequencer);
	struct i2c_client *client = chip->client;
	int ret = 0;
	u8 sequencer_state, buf, temp_mode;

	/* if in that mode already do nothing, except for run & reload*/
	if (mode == sequencer->mode && mode != AS3665_CMD_RUN &&
		mode != AS3665_CMD_RELOAD)
		return 0;

	temp_mode = sequencer->mode;
	sequencer->mode = mode;
	if (mode) {
		ret = as3665_chipen_check(client);
		if (ret < 0)
			goto fail;
	}

	buf = mode;
	ret = as3665_read(client, AS3665_REG_EXEC_ENABLE, &sequencer_state);
	if (ret < 0)
		goto fail;

	sequencer_state &= ~sequencer->sequencer_mask;
	/* set mode only for this sequencer */
	buf &= sequencer->sequencer_mask;
	sequencer_state |= buf;

	ret = as3665_write(client, AS3665_REG_EXEC_ENABLE, sequencer_state);
	if (ret < 0)
		goto fail;

	if (!mode)
		ret = as3665_chipen_check(client);

fail:
	if (ret < 0)
		sequencer->mode = temp_mode;

	return ret;
}

static int as3665_set_sequencer_run_mode(struct as3665_sequencer *sequencer,
				u8 run_mode)
{
	struct as3665_chip *chip = sequencer_to_as3665(sequencer);
	struct i2c_client *client = chip->client;
	int ret = 0;
	u8 sequencer_runmode, buf;

	/* if in that run mode already do nothing, except for step */
	if (run_mode == sequencer->run_mode &&
		(run_mode != AS3665_CMD_STEP ||
		run_mode != AS3665_CMD_STEP_IN_PLACE))
		return 0;

	buf = run_mode;
	ret = as3665_read(client, AS3665_REG_EXEC_MODE, &sequencer_runmode);
	if (ret < 0)
		goto fail;

	sequencer_runmode &= ~sequencer->sequencer_mask;
	/* set run mode only for this sequencer */
	buf &= sequencer->sequencer_mask;
	sequencer_runmode |= buf;

	ret = as3665_write(client, AS3665_REG_EXEC_MODE, sequencer_runmode);
	if (ret < 0)
		goto fail;

	sequencer->run_mode = run_mode;

fail:
	return ret;
}

static int as3665_load_program(struct i2c_client *client, u8 *pattern, int len)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	int ret = 0, i;
	u8 page_num, bytes_remain;
	u8 chipstate, buf;

	ret = as3665_read(client, AS3665_REG_EXEC_ENABLE, &chipstate);
	if (ret < 0)
		goto fail;

	/* do ram init & disable sequencers before loading */
	ret = as3665_write(client, AS3665_REG_EXEC_ENABLE,
				AS3665_ENABLE | AS3665_CMD_DISABLED);
	if (ret < 0)
		goto fail;

	ret = as3665_write(client, AS3665_REG_EXEC_ENABLE,
				AS3665_RAM_INIT | AS3665_ENABLE);
	if (ret < 0)
		goto fail;
	usleep_range(12000, 13000);

	buf = 0x80;
	while (buf & AS3665_RAM_INIT) {
		ret = as3665_read(client, AS3665_REG_EXEC_ENABLE, &buf);
		if (ret < 0)
			goto fail;
	}

	chipstate &= AS3665_ENABLE;
	ret = as3665_write(client, AS3665_REG_EXEC_ENABLE,
				chipstate | AS3665_CMD_DISABLED);
	if (ret < 0)
		goto fail;

	for (i = 0; i < ARRAY_SIZE(chip->sequencers); i++)
		chip->sequencers[i].mode = AS3665_CMD_DISABLED;

	page_num = (len-3) >> 5;
	bytes_remain = (len-3) & 0x1F;
	for (i = 0; i < page_num; i++) {
		ret = as3665_write(client, AS3665_REG_PAGE_SEL, i);
		if (ret < 0)
			goto fail;
		ret = i2c_smbus_write_i2c_block_data(client,
				AS3665_REG_PROG_MEM_BASE,
				AS3665_PROGRAM_PAGE_LENGTH,
				pattern + 3 + i * AS3665_PROGRAM_PAGE_LENGTH);
		if (ret < 0)
			goto fail;
	}

	ret = as3665_write(client, AS3665_REG_PAGE_SEL, i);
	if (ret < 0)
		goto fail;
	ret = i2c_smbus_write_i2c_block_data(client,
				AS3665_REG_PROG_MEM_BASE,
				bytes_remain,
				pattern + 3 + i * AS3665_PROGRAM_PAGE_LENGTH);
	if (ret < 0)
		goto fail;

	ret = as3665_write(client, AS3665_REG_SEQ1_PROG_START, *pattern);
	if (ret < 0)
		goto fail;
	ret = as3665_write(client, AS3665_REG_SEQ2_PROG_START, *(pattern + 1));
	if (ret < 0)
		goto fail;
	ret = as3665_write(client, AS3665_REG_SEQ3_PROG_START, *(pattern + 2));

fail:
	return ret;
}

/*--------------------------------------------------------------*/
/*			Sysfs interface			*/
/*--------------------------------------------------------------*/

static void as3665_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct as3665_led *led = cdev_to_led(cdev);
	struct as3665_chip *chip = led_to_as3665(led);
	struct i2c_client *client = chip->client;
	int ret = 0;

	mutex_lock(&chip->lock);

	led->brightness = (u8)brightness;
	if (brightness)
		as3665_chipen_check(client);

	ret = as3665_write(client, AS3665_REG_LED_PWM_BASE + led->chan_nr,
		     led->brightness);
	if (ret < 0)
		goto fail;

	if (!brightness)
		as3665_chipen_check(client);

fail:
	mutex_unlock(&chip->lock);
}

static int as3665_do_store_load(struct i2c_client *client,
				const char *buf, size_t len)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	int  ret = 0, nrchars, offset = 0, i = 0;
	char c[3];
	unsigned int cmd;
	u8 pattern[(AS3665_PROGRAM_LENGTH + 3)] = {0};
	while ((offset < len - 1) && (i < AS3665_PROGRAM_LENGTH + 3)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(buf + offset, "%2s%n ", c, &nrchars);
		if (ret != 1)
			goto fail;
		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto fail;
		pattern[i] = cmd;

		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if ((i - 3) % 2)
		goto fail;

	mutex_lock(&chip->lock);
	ret = as3665_load_program(client, pattern, i);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		return ret;

	return len;
fail:
	dev_err(&client->dev, "wrong pattern format\n");
	return -EINVAL;
}

static ssize_t store_sequencer_load(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	return as3665_do_store_load(client, buf, len);
}

static ssize_t show_sequencer_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);

	switch (chip->sequencers[nr - 1].mode) {
	case AS3665_CMD_RUN:
		return scnprintf(buf, PAGE_SIZE, "run\n");
	case AS3665_CMD_RELOAD:
		return scnprintf(buf, PAGE_SIZE, "reload\n");
	case AS3665_CMD_DISABLED:
		return scnprintf(buf, PAGE_SIZE, "disabled\n");
	default:
		return scnprintf(buf, PAGE_SIZE, "disabled\n");
	}
}

#define show_mode(nr)							\
static ssize_t show_sequencer##nr##_mode(struct device *dev,		\
				    struct device_attribute *attr,	\
				    char *buf)				\
{									\
	return show_sequencer_mode(dev, attr, buf, nr);			\
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_sequencer_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_sequencer *sequencer = &chip->sequencers[nr - 1];

	mutex_lock(&chip->lock);

	if (!strncmp(buf, "run", 3))
		as3665_set_sequencer_mode(sequencer, AS3665_CMD_RUN);
	else if (!strncmp(buf, "reload", 6))
		as3665_set_sequencer_mode(sequencer, AS3665_CMD_RELOAD);
	else if (!strncmp(buf, "disabled", 8))
		as3665_set_sequencer_mode(sequencer, AS3665_CMD_DISABLED);

	mutex_unlock(&chip->lock);

	return len;
}

#define store_mode(nr)							\
static ssize_t store_sequencer##nr##_mode(struct device *dev,		\
				     struct device_attribute *attr,	\
				     const char *buf, size_t len)	\
{									\
	return store_sequencer_mode(dev, attr, buf, len, nr);		\
}
store_mode(1)
store_mode(2)
store_mode(3)

static ssize_t show_sequencer_run_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);

	switch (chip->sequencers[nr - 1].run_mode) {
	case AS3665_CMD_HOLD:
		return scnprintf(buf, PAGE_SIZE, "hold\n");
	case AS3665_CMD_STEP:
		return scnprintf(buf, PAGE_SIZE, "step\n");
	case AS3665_CMD_RUN:
		return scnprintf(buf, PAGE_SIZE, "run\n");
	case AS3665_CMD_STEP_IN_PLACE:
		return scnprintf(buf, PAGE_SIZE, "step_in_place\n");
	default:
		return scnprintf(buf, PAGE_SIZE, "disabled\n");
	}
}

#define show_run_mode(nr)						\
static ssize_t show_sequencer##nr##_run_mode(struct device *dev,	\
				    struct device_attribute *attr,	\
				    char *buf)				\
{									\
	return show_sequencer_run_mode(dev, attr, buf, nr);		\
}
show_run_mode(1)
show_run_mode(2)
show_run_mode(3)

static ssize_t store_sequencer_run_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_sequencer *sequencer = &chip->sequencers[nr - 1];

	mutex_lock(&chip->lock);

	if (!strncmp(buf, "hold", 4))
		as3665_set_sequencer_run_mode(sequencer, AS3665_CMD_HOLD);
	else if (!strncmp(buf, "step", 4))
		as3665_set_sequencer_run_mode(sequencer, AS3665_CMD_STEP);
	else if (!strncmp(buf, "run", 3))
		as3665_set_sequencer_run_mode(sequencer, AS3665_CMD_RUN);
	else if (!strncmp(buf, "step_in_place", 13))
		as3665_set_sequencer_run_mode(sequencer,
						AS3665_CMD_STEP_IN_PLACE);

	mutex_unlock(&chip->lock);

	return len;
}

#define store_run_mode(nr)						\
static ssize_t store_sequencer##nr##_run_mode(struct device *dev,	\
				     struct device_attribute *attr,	\
				     const char *buf, size_t len)	\
{									\
	return store_sequencer_run_mode(dev, attr, buf, len, nr);	\
}
store_run_mode(1)
store_run_mode(2)
store_run_mode(3)

static ssize_t show_audio_en(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;

	return scnprintf(buf, PAGE_SIZE, "%x\n", audio->enable);
}

static ssize_t store_audio_en(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;
	ssize_t ret = 0;
	unsigned long enable;
	u8 audiostate;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		enable = 1;

	mutex_lock(&chip->lock);

	ret = as3665_read(client, AS3665_REG_AUDIO_CTRL, &audiostate);
	if (ret < 0)
		goto fail;

	if (enable) {
		ret = as3665_write(client, AS3665_REG_AUDIO_CTRL,
				audiostate | AS3665_AUDIO_ON |
				AS3665_AUDIO_CMDSET | AS3665_AUDIO_BUF_ON);
		if (ret < 0)
			goto fail;
	} else {
		ret = as3665_write(client, AS3665_REG_AUDIO_CTRL,
				audiostate & ~(AS3665_AUDIO_ON |
				AS3665_AUDIO_CMDSET | AS3665_AUDIO_BUF_ON));
		if (ret < 0)
			goto fail;
	}

	audio->enable = enable;

fail:
	mutex_unlock(&chip->lock);
	return len;
}

static ssize_t show_audio_buf_gain(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;

	return scnprintf(buf, PAGE_SIZE, "%x\n", audio->buf_gain);
}

static ssize_t store_audio_buf_gain(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;
	ssize_t ret = 0;
	unsigned long buf_gain;
	u8 audiostate;

	if (kstrtoul(buf, 10, &buf_gain))
		return -EINVAL;

	if (buf_gain > 7)
		return -EINVAL;

	mutex_lock(&chip->lock);

	ret = as3665_read(client, AS3665_REG_AUDIO_CTRL, &audiostate);
	if (ret < 0)
		goto fail;
	audiostate &= ~AS3665_AUDIO_BUF_GAIN_MASK;
	audiostate |= buf_gain << 3;
	ret = as3665_write(client, AS3665_REG_AUDIO_CTRL, audiostate);
	if (ret < 0)
		goto fail;

	audio->buf_gain = buf_gain;

fail:
	mutex_unlock(&chip->lock);
	return len;
}

static ssize_t show_audio_agc_ctrl(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;

	return scnprintf(buf, PAGE_SIZE, "%x\n", audio->agc_ctrl);
}

static ssize_t store_audio_agc_ctrl(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;
	ssize_t ret = 0;
	unsigned long agc_ctrl;
	u8 agcstate;

	if (kstrtoul(buf, 10, &agc_ctrl))
		return -EINVAL;

	if (agc_ctrl > 7)
		return -EINVAL;

	mutex_lock(&chip->lock);

	ret = as3665_read(client, AS3665_REG_AUDIO_AGC, &agcstate);
	if (ret < 0)
		goto fail;
	agcstate &= ~AS3665_AUDIO_AGC_CTRL_MASK;
	agcstate |= agc_ctrl;
	ret = as3665_write(client, AS3665_REG_AUDIO_AGC, agcstate);
	if (ret < 0)
		goto fail;

	audio->agc_ctrl = agc_ctrl;

fail:
	mutex_unlock(&chip->lock);
	return len;
}

static ssize_t show_audio_agc_time(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;

	return scnprintf(buf, PAGE_SIZE, "%x\n", audio->agc_time);
}

static ssize_t store_audio_agc_time(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct as3665_audio *audio = &chip->audio;
	ssize_t ret = 0;
	unsigned long agc_time;
	u8 agcstate;

	if (kstrtoul(buf, 10, &agc_time))
		return -EINVAL;

	if (agc_time > 3)
		return -EINVAL;

	mutex_lock(&chip->lock);

	ret = as3665_read(client, AS3665_REG_AUDIO_AGC, &agcstate);
	if (ret < 0)
		goto fail;
	agcstate &= ~AS3665_AUDIO_AGC_TIME_MASK;
	agcstate |= agc_time << 3;
	ret = as3665_write(client, AS3665_REG_AUDIO_AGC, agcstate);
	if (ret < 0)
		goto fail;

	audio->agc_time = agc_time;

fail:
	mutex_unlock(&chip->lock);
	return len;
}

static ssize_t show_max_current(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct as3665_led *led = cdev_to_led(led_cdev);

	return scnprintf(buf, PAGE_SIZE, "%x\n", led->max_current);
}

static ssize_t store_max_current(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct as3665_led *led = cdev_to_led(led_cdev);
	struct as3665_chip *chip = led_to_as3665(led);
	ssize_t ret = 0;
	unsigned long curr;
	u8 maxcurr;

	if (kstrtoul(buf, 10, &curr))
		return -EINVAL;

	if (curr > 0x03)
		return -EINVAL;

	mutex_lock(&chip->lock);

	ret = as3665_read(chip->client,
			AS3665_REG_LED_MAXCURR_BASE + (led->chan_nr >> 2),
			&maxcurr);
	if (ret < 0)
		return ret;
	maxcurr &= ~(3 << (led->chan_nr % 4 * 2));
	maxcurr |= curr << (led->chan_nr % 4 * 2);
	ret = as3665_write(chip->client,
			AS3665_REG_LED_MAXCURR_BASE + (led->chan_nr >> 2),
			maxcurr);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		return ret;

	led->max_current = (u8)curr;

	return len;
}

static ssize_t show_current(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct as3665_led *led = cdev_to_led(led_cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", led->led_current);
}

static ssize_t set_current_internal(struct as3665_led *led,
				struct as3665_chip *chip, u8 curr)
{
	ssize_t ret = 0;
	ret = as3665_write(chip->client,
			AS3665_REG_LED_CURRENT_BASE + led->chan_nr, curr);

	return ret;


}

static ssize_t store_current(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct as3665_led *led = cdev_to_led(led_cdev);
	struct as3665_chip *chip = led_to_as3665(led);
	ssize_t ret = 0;
	unsigned long curr;

	if (kstrtoul(buf, 10, &curr))
		return -EINVAL;

	if (curr > 0xFF)
		return -EINVAL;

	ret = set_current_internal(led, chip, curr);

	if (ret < 0)
		return ret;

	led->led_current = (u8)curr;

	return len;
}

/* led class device attributes */
static DEVICE_ATTR(led_current, S_IRUGO
			| S_IWUSR, show_current, store_current);
static DEVICE_ATTR(max_current, S_IRUGO
			| S_IWUSR, show_max_current, store_max_current);

static struct attribute *as3665_led_attributes[] = {
	&dev_attr_led_current.attr,
	&dev_attr_max_current.attr,
	NULL,
};

static struct attribute_group as3665_led_attribute_group = {
	.attrs = as3665_led_attributes
};

/* device attributes */
static DEVICE_ATTR(sequencer1_mode, S_IRUGO | S_IWUSR,
		   show_sequencer1_mode, store_sequencer1_mode);
static DEVICE_ATTR(sequencer2_mode, S_IRUGO | S_IWUSR,
		   show_sequencer2_mode, store_sequencer2_mode);
static DEVICE_ATTR(sequencer3_mode, S_IRUGO | S_IWUSR,
		   show_sequencer3_mode, store_sequencer3_mode);
static DEVICE_ATTR(sequencer1_run_mode, S_IRUGO | S_IWUSR,
		   show_sequencer1_run_mode, store_sequencer1_run_mode);
static DEVICE_ATTR(sequencer2_run_mode, S_IRUGO | S_IWUSR,
		   show_sequencer2_run_mode, store_sequencer2_run_mode);
static DEVICE_ATTR(sequencer3_run_mode, S_IRUGO | S_IWUSR,
		   show_sequencer3_run_mode, store_sequencer3_run_mode);
static DEVICE_ATTR(sequencer_load, S_IWUSR, NULL, store_sequencer_load);
static DEVICE_ATTR(audio_en, S_IRUGO | S_IWUSR,
		   show_audio_en, store_audio_en);
static DEVICE_ATTR(audio_buf_gain, S_IRUGO | S_IWUSR,
		   show_audio_buf_gain, store_audio_buf_gain);
static DEVICE_ATTR(audio_agc_ctrl, S_IRUGO | S_IWUSR,
		   show_audio_agc_ctrl, store_audio_agc_ctrl);
static DEVICE_ATTR(audio_agc_time, S_IRUGO | S_IWUSR,
		   show_audio_agc_time, store_audio_agc_time);

static struct attribute *as3665_attributes[] = {
	&dev_attr_sequencer1_mode.attr,
	&dev_attr_sequencer2_mode.attr,
	&dev_attr_sequencer3_mode.attr,
	&dev_attr_sequencer1_run_mode.attr,
	&dev_attr_sequencer2_run_mode.attr,
	&dev_attr_sequencer3_run_mode.attr,
	NULL
};

static struct attribute *as3665_sequencer_attributes[] = {
	&dev_attr_sequencer_load.attr,
	NULL
};

static struct attribute *as3665_audio_attributes[] = {
	&dev_attr_audio_en.attr,
	&dev_attr_audio_buf_gain.attr,
	&dev_attr_audio_agc_ctrl.attr,
	&dev_attr_audio_agc_time.attr,
	NULL
};

static const struct attribute_group as3665_group = {
	.attrs = as3665_attributes,
};

static const struct attribute_group as3665_sequencer_group = {
	.attrs = as3665_sequencer_attributes,
};

static const struct attribute_group as3665_audio_group = {
	.attrs = as3665_audio_attributes,
};

static int as3665_register_sysfs(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int ret = 0;

	ret = sysfs_create_group(&dev->kobj, &as3665_group);
	if (ret < 0)
		return ret;

	ret = sysfs_create_group(&dev->kobj, &as3665_sequencer_group);
	if (ret < 0)
		goto fail1;

	ret = sysfs_create_group(&dev->kobj, &as3665_audio_group);
	if (ret < 0)
		goto fail2;

	return 0;

fail2:
	sysfs_remove_group(&dev->kobj, &as3665_sequencer_group);
fail1:
	sysfs_remove_group(&dev->kobj, &as3665_group);
	return ret;
}

static void as3665_unregister_sysfs(struct i2c_client *client)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int i;

	sysfs_remove_group(&dev->kobj, &as3665_group);

	sysfs_remove_group(&dev->kobj, &as3665_sequencer_group);

	sysfs_remove_group(&dev->kobj, &as3665_audio_group);

	for (i = 0; i < chip->num_leds; i++)
		sysfs_remove_group(&chip->leds[i].cdev.dev->kobj,
				&as3665_led_attribute_group);
}

/*--------------------------------------------------------------*/
/*			Probe, Attach, Remove			*/
/*--------------------------------------------------------------*/
static int __init as3665_init_sequencer(struct as3665_sequencer
					*sequencer, int id)
{
	if (id < 1 || id > AS3665_SEQS)
		return -EINVAL;
	sequencer->id = id;
	sequencer->sequencer_mask = AS3665_SEQ_MASK_BASE << SHIFT_MASK(id);

	return 0;
}

static int __init as3665_init_led(struct as3665_led *led, struct device *dev,
			   int chan, struct as3665_platform_data *pdata)
{
	int ret = 0;

	if (chan >= AS3665_LEDS)
		return -EINVAL;

	led->led_current = pdata->led_config[chan].led_current;
	led->max_current = pdata->led_config[chan].max_current;
	led->startup_current = pdata->led_config[chan].startup_current;
	led->chan_nr = pdata->led_config[chan].chan_nr;

	if (led->chan_nr >= AS3665_LEDS) {
		dev_err(dev, "Use channel numbers between 0 and %d\n",
			AS3665_LEDS - 1);
		return -EINVAL;
	}

	led->cdev.name = pdata->led_config[chan].name;
	led->cdev.brightness_set = as3665_set_brightness;
	ret = led_classdev_register(dev, &led->cdev);
	if (ret < 0) {
		dev_err(dev, "couldn't register led on %s\n", led->cdev.name);
		return ret;
	}

	ret = sysfs_create_group(&led->cdev.dev->kobj,
			&as3665_led_attribute_group);
	if (ret < 0) {
		dev_err(dev, "couldn't register current attribute\n");
		led_classdev_unregister(&led->cdev);
		return ret;
	}

	return 0;
}

static int as3665_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct as3665_chip	*chip;
	struct as3665_platform_data	*pdata;
	int ret, i, led;
	u8 buf;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client = client;

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		ret = -EINVAL;
		goto fail1;
	}

	mutex_init(&chip->lock);

	chip->pdata = pdata;

	if (pdata->setup_resources) {
		ret = pdata->setup_resources(&client->dev);
		if (ret < 0)
			goto fail1;
	}

	if (pdata->enable) {
		ret = pdata->enable(&client->dev, 0);
		if (ret < 0)
			goto fail2;
		usleep_range(1000, 2000); /* Keep enable down at least 1ms */
		ret = pdata->enable(&client->dev, 1);
		if (ret < 0)
			goto fail2;
		usleep_range(12000, 13000);
	}

	/* enable chip before reset the chip */
	ret = as3665_write(client, AS3665_REG_EXEC_ENABLE, AS3665_ENABLE);
	if (ret < 0)
		goto fail3;

	ret = as3665_write(client, AS3665_REG_RESET_CTRL, AS3665_FORCE_RESET);
	if (ret < 0)
		goto fail3;
	usleep_range(12000, 13000); /* Keep 12ms for reg reset */

	ret = as3665_read(client, AS3665_REG_CHIP_ID1, &buf);
	if (ret < 0)
		goto fail3;
	if (buf != 0xC9) {
		dev_err(&client->dev, "Device is not AS3665, ID is %x\n", buf);
		goto fail3;
	}

	dev_info(&client->dev, "AS3665 Programmable led chip found\n");

	/* Initialize sequencers */
	for (i = 0; i < ARRAY_SIZE(chip->sequencers); i++) {
		ret = as3665_init_sequencer(&chip->sequencers[i], i + 1);
		if (ret < 0)
			goto fail3;
	}

	ret = as3665_configure(client);
	if (ret < 0) {
		dev_err(&client->dev, "error configuring chip\n");
		goto fail3;
	}

	/* Initialize leds */
	chip->num_channels = pdata->num_channels;
	chip->num_leds = 0;
	led = 0;
	for (i = 0; i < pdata->num_channels; i++) {
		/* Do not initialize channels that are not connected */
		if (pdata->led_config[i].name == NULL)
			continue;

		ret = as3665_init_led(&chip->leds[led], &client->dev, i, pdata);
		if (ret < 0) {
			dev_err(&client->dev, "error initializing leds\n");
			goto fail4;
		}
		chip->num_leds++;

		chip->leds[led].id = led;
		/* Set led current, disable fader and color correction */
		ret = as3665_read(client,
			AS3665_REG_LED_MAXCURR_BASE +
			(chip->leds[led].chan_nr >> 2),
			&buf);
		if (ret < 0)
			goto fail4;
		buf &= ~(3 << (chip->leds[led].chan_nr % 4 * 2));
		buf |= chip->leds[led].max_current <<
			(chip->leds[led].chan_nr % 4 * 2);
		ret = as3665_write(client,
			AS3665_REG_LED_MAXCURR_BASE +
			(chip->leds[led].chan_nr >> 2),
			buf);
		if (ret < 0)
			goto fail4;
		ret = as3665_write(client,
			AS3665_REG_LED_CURRENT_BASE + chip->leds[led].chan_nr,
			chip->leds[led].led_current);
		if (ret < 0)
			goto fail4;
		ret = as3665_write(client,
			AS3665_REG_LED_SETUP_BASE + chip->leds[led].chan_nr,
			0x20);
		if (ret < 0)
			goto fail4;
		ret = as3665_write(client,
			AS3665_REG_LED_PWM_BASE + chip->leds[led].chan_nr,
			0x00);
		if (ret < 0)
			goto fail4;

		led++;
	}

	ret = as3665_register_sysfs(client);
	if (ret < 0) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto fail4;
	}

	for (i = 0; i < chip->num_leds; i++) {
		struct as3665_led *led = &chip->leds[i];
		if (led->startup_current) {
			set_current_internal(led, chip, led->startup_current);
			as3665_set_brightness(&chip->leds[i].cdev, 255);
		}
	}

	return ret;
fail4:
	for (i = 0; i < chip->num_leds; i++) {
		led_classdev_unregister(&chip->leds[i].cdev);
		sysfs_remove_group(&chip->leds[i].cdev.dev->kobj,
				&as3665_led_attribute_group);
	}
fail3:
	if (pdata->enable)
		pdata->enable(&client->dev, 0);
fail2:
	if (pdata->release_resources)
		pdata->release_resources(&client->dev);
fail1:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return ret;
}

static int as3665_remove(struct i2c_client *client)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	int i;

	as3665_unregister_sysfs(client);

	for (i = 0; i < chip->num_leds; i++)
		led_classdev_unregister(&chip->leds[i].cdev);

	if (chip->pdata->enable)
		chip->pdata->enable(&client->dev, 0);
	if (chip->pdata->release_resources)
		chip->pdata->release_resources(&client->dev);

	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

static void as3665_shutdown(struct i2c_client *client)
{
	struct as3665_chip *chip = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < chip->num_leds; i++) {
		struct as3665_led *led = &chip->leds[i];
		set_current_internal(led, chip, 0);
		as3665_set_brightness(&chip->leds[i].cdev, 0);
	}

	as3665_write(client, AS3665_REG_EXEC_ENABLE, 0x00);
}

static const struct i2c_device_id as3665_id[] = {
	{ "as3665", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, as3665_id);

static struct i2c_driver as3665_driver = {
	.driver = {
		.name	= "as3665",
	},
	.probe		= as3665_probe,
	.remove		= as3665_remove,
	.shutdown	= as3665_shutdown,
	.id_table	= as3665_id,
};

static int __init as3665_init(void)
{
	int ret;

	ret = i2c_add_driver(&as3665_driver);

	if (ret < 0)
		printk(KERN_ERR "Adding as3665 driver failed\n");

	return ret;
}

static void __exit as3665_exit(void)
{
	i2c_del_driver(&as3665_driver);
}

module_init(as3665_init);
module_exit(as3665_exit);

MODULE_AUTHOR("Byron Shi <byron.shi@ams.com>");
MODULE_AUTHOR("Yu Hongxing <Hongxing2.Yu@SonyMobile.com>");
MODULE_DESCRIPTION("AS3665 LED driver");
MODULE_LICENSE("GPL");
