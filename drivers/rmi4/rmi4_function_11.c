/*
 * RMI4 bus driver.
 * driver/rmi4/rmi4_function_34.c
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2112 Sony Mobile Communications AB
 *
 * Author: joachim Holst <joachim.holst@sonymobile.com>
 *
 * Based on rmi_bus by Synaptics and Unixphere.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/rmi4/rmi4.h>
#include <linux/rmi4/rmi4_function_11.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/list.h>

#define F11_MAX_NUM_OF_FINGERS		10
#define F11_REL_POS_MIN		-128
#define F11_REL_POS_MAX		127
#define F11_FINGER_STATE_MASK_N(i) \
		(F11_FINGER_STATE_MASK << (i%4 * F11_FINGER_STATE_SIZE))

#define F11_FINGER_STATE_VAL_N(f_state, i) \
		(f_state >> (i%4 * F11_FINGER_STATE_SIZE))
#define F11_CTRL_SENSOR_MAX_X_POS_OFFSET	6
#define F11_CTRL_SENSOR_MAX_Y_POS_OFFSET	8
#define F11_CEIL(x, y) (((x) + ((y)-1)) / (y))
#define F11_FINGER_STATE_MASK	0x03
#define F11_FINGER_STATE_SIZE	0x02

#define DEFAULT_MAX_ABS_MT_PRESSURE 255
#define DEFAULT_MAX_ABS_MT_TOUCH 15
#define DEFAULT_MAX_ABS_MT_ORIENTATION 1
#define DEFAULT_MIN_ABS_MT_TRACKING_ID 1
#define DEFAULT_MAX_ABS_MT_TRACKING_ID 10

/**
 * RMI F11 - function control register parameters
 * Each register that has a specific bit-field setup has an accompanied
 * register definition so that the setting can be chosen as a one-word
 * register setting or per-bit setting.
 */
union rmi_f11_2d_ctrl0 {
	struct {
		u8 reporting_mode:3;
		u8 abs_pos_filt:1;
		u8 rel_pos_filt:1;
		u8 rel_ballistics:1;
		u8 dribble:1;
		u8 report_beyond_clip:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl1 {
	struct {
		u8 palm_detect_thres:4;
		u8 motion_sensitivity:2;
		u8 man_track_en:1;
		u8 man_tracked_finger:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl2__3 {
	struct {
		u8 delta_x_threshold:8;
		u8 delta_y_threshold:8;
	};
	u8 regs[2];
};

union rmi_f11_2d_ctrl4 {
	struct {
		u8 velocity:8;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl5 {
	struct {
		u8 acceleration:8;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl6__7 {
	struct {
		u16 sensor_max_x_pos:12;
	};
	u8 regs[2];
};

union rmi_f11_2d_ctrl8__9 {
	struct {
		u16 sensor_max_y_pos:12;
	};
	u8 regs[2];
};

union rmi_f11_2d_ctrl10 {
	struct {
		u8 single_tap_int_enable:1;
		u8 tap_n_hold_int_enable:1;
		u8 double_tap_int_enable:1;
		u8 early_tap_int_enable:1;
		u8 flick_int_enable:1;
		u8 press_int_enable:1;
		u8 pinch_int_enable:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl11 {
	struct {
		u8 palm_detect_int_enable:1;
		u8 rotate_int_enable:1;
		u8 touch_shape_int_enable:1;
		u8 scroll_zone_int_enable:1;
		u8 multi_finger_scroll_int_enable:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl12 {
	struct {
		u8 sensor_map:7;
		u8 xy_sel:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl14 {
	struct {
		u8 sens_adjustment:5;
		u8 hyst_adjustment:3;
	};
	u8 reg;
};

struct f11_2d_device_query {
	union {
		struct {
			u8 nbr_of_sensors:3;
			u8 has_query9:1;
			u8 has_query11:1;
		};
		u8 f11_2d_query0;
	};

	u8 f11_2d_query9;

	union {
		struct {
			u8 has_z_tuning:1;
			u8 has_pos_interpolation_tuning:1;
			u8 has_w_tuning:1;
			u8 has_pitch_info:1;
			u8 has_default_finger_width:1;
			u8 has_segmentation_aggressiveness:1;
			u8 has_tx_rw_clip:1;
			u8 has_drumming_correction:1;
		};
		u8 f11_2d_query11;
	};
};

struct f11_2d_sensor_query {
	union {
		struct {
			/* query1 */
			u8 number_of_fingers:3;
			u8 has_rel:1;
			u8 has_abs:1;
			u8 has_gestures:1;
			u8 has_sensitivity_adjust:1;
			u8 configurable:1;
			/* query2 */
			u8 num_of_x_electrodes:7;
			/* query3 */
			u8 num_of_y_electrodes:7;
			/* query4 */
			u8 max_electrodes:7;
		};
		u8 f11_2d_query1__4[4];
	};

	union {
		struct {
			u8 abs_data_size:3;
			u8 has_anchored_finger:1;
			u8 has_adj_hyst:1;
			u8 has_dribble:1;
		};
		u8 f11_2d_query5;
	};

	u8 f11_2d_query6;

	union {
		struct {
			u8 has_single_tap:1;
			u8 has_tap_n_hold:1;
			u8 has_double_tap:1;
			u8 has_early_tap:1;
			u8 has_flick:1;
			u8 has_press:1;
			u8 has_pinch:1;
			u8 padding:1;

			u8 has_palm_det:1;
			u8 has_rotate:1;
			u8 has_touch_shapes:1;
			u8 has_scroll_zones:1;
			u8 has_individual_scroll_zones:1;
			u8 has_multi_finger_scroll:1;
		};
		u8 f11_2d_query7__8[2];
	};

	/* Empty */
	u8 f11_2d_query9;

	union {
		struct {
			u8 nbr_touch_shapes:5;
		};
		u8 f11_2d_query10;
	};
};

struct f11_2d_data_0 {
	u8 finger_n;
};

struct f11_2d_data_1_5 {
	u8 x_msb;
	u8 y_msb;
	u8 x_lsb:4;
	u8 y_lsb:4;
	u8 w_y:4;
	u8 w_x:4;
	u8 z;
};

struct f11_2d_data_6_7 {
	s8 delta_x;
	s8 delta_y;
};

struct f11_2d_data_8 {
	u8 single_tap:1;
	u8 tap_and_hold:1;
	u8 double_tap:1;
	u8 early_tap:1;
	u8 flick:1;
	u8 press:1;
	u8 pinch:1;
};

struct f11_2d_data_9 {
	u8 palm_detect:1;
	u8 rotate:1;
	u8 shape:1;
	u8 scrollzone:1;
	u8 finger_count:3;
};

struct f11_2d_data_10 {
	u8 pinch_motion;
};

struct f11_2d_data_10_12 {
	u8 x_flick_dist;
	u8 y_flick_dist;
	u8 flick_time;
};

struct f11_2d_data_11_12 {
	u8 motion;
	u8 finger_separation;
};

struct f11_2d_data_13 {
	u8 shape_n;
};

struct f11_2d_data_14_15 {
	u8 horizontal;
	u8 vertical;
};

struct f11_2d_data_14_17 {
	u8 x_low;
	u8 y_right;
	u8 x_upper;
	u8 y_left;
};

/* This is a tad evil. It seems that the queries are not static
 * buth change based on what the chip supports. Due to this, we
 * can't quite use a static struct, but will need to separately
 * allocate a "packet buffer" and typecast these pointers to
 * point to the correct location of this "packet buffer".
 * This looks really strange */
struct f11_2d_data {
	const struct f11_2d_data_0	*f_state;
	const struct f11_2d_data_1_5	*abs_pos;
	const struct f11_2d_data_6_7	*rel_pos;
	const struct f11_2d_data_8	*gest_1;
	const struct f11_2d_data_9	*gest_2;
	const struct f11_2d_data_10	*pinch;
	const struct f11_2d_data_10_12	*flick;
	const struct f11_2d_data_11_12	*rotate;
	const struct f11_2d_data_13	*shapes;
	const struct f11_2d_data_14_15	*multi_scroll;
	const struct f11_2d_data_14_17	*scroll_zones;
};

struct rmi4_f11_2d_sensor {
	struct list_head entry;

	struct rmi_f11_2d_axis_alignment axis_align;
	struct f11_2d_sensor_query sens_query;
	struct f11_2d_data data;
	u16 max_x;
	u16 max_y;
	u8 nbr_fingers;
	u8 finger_tracker[F11_MAX_NUM_OF_FINGERS];
	u8 *data_pkt;
	int pkt_size;
	u8 sensor_index;

	bool input_dev_registered;
	struct input_dev *input;
	bool mouse_dev_registered;
	struct input_dev *mouse_input;
};
#define to_rmi4_f11_2d_sensor(l)				\
	container_of(l, struct rmi4_f11_2d_sensor, entry)

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_RESERVED	= 0x03
};

/* The configuation is controlled as per register which means that if a register
 * is allocated for ctrl configuration one must make sure that all the bits are
 * set accordingly for that particular register.
 * It is not certain that all controls are available on all chips. So, we will
 * need to assigne these dynamically, like it's done in f11_2d_data, based
 * on what the chip supports.
 * One thing that needs to be addressed here, is the register gap that exists
 * in some chips. For TM1886-001, there is a gap between ctrl12.25 of 21
 * register indexes. This means that ctrl14 is not placed directly after 12.
 */
struct  rmi_f11_2d_ctrl {
	union rmi_f11_2d_ctrl0		ctrl0;
	union rmi_f11_2d_ctrl1		ctrl1;
	union rmi_f11_2d_ctrl2__3	ctrl2__3;
	union rmi_f11_2d_ctrl4		ctrl4;
	union rmi_f11_2d_ctrl5		ctrl5;
	union rmi_f11_2d_ctrl6__7	ctrl6__7;
	union rmi_f11_2d_ctrl8__9	ctrl8__9;
	union rmi_f11_2d_ctrl10	ctrl10;
	union rmi_f11_2d_ctrl11	ctrl11;
	union rmi_f11_2d_ctrl12	ctrl12;
	u8				ctrl12_size;
	union rmi_f11_2d_ctrl14	ctrl14;
	u8				ctrl15;
	u8				ctrl16;
	u8				ctrl17;
	u8				ctrl18;
	u8				ctrl19;
};

struct rmi4_f11_data {
	struct mutex f11_lock;
	struct list_head sensor_list;
	struct f11_2d_device_query dev_query;
	struct rmi_f11_2d_ctrl dev_controls;
};

static void rmi4_f11_irq_handler(int irq, void *data);

static void rmi4_f11_destroy_sensor(struct rmi4_f11_2d_sensor *sensor) {
	if (sensor) {
		if (sensor->input) {
			kfree(sensor->input->name);
			kfree(sensor->input->phys);
			if (sensor->input_dev_registered)
				input_unregister_device(sensor->input);
			else
				input_free_device(sensor->input);
		}
		if (sensor->mouse_input) {
			kfree(sensor->mouse_input->name);
			kfree(sensor->mouse_input->phys);
			if (sensor->mouse_dev_registered)
				input_unregister_device(sensor->mouse_input);
			else
				input_free_device(sensor->mouse_input);
		}
		kfree(sensor);
	}
}

static void rmi4_f11_unregister_input_devs(struct rmi4_function_device *fdev)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_f11_2d_sensor *sensor;
	struct rmi4_f11_data *data = dev_get_drvdata(&fdev->dev);

	if (list_empty(&data->sensor_list))
		return;

	list_for_each_safe(list, n, &data->sensor_list) {
		sensor = to_rmi4_f11_2d_sensor(list);
		list_del(list);
		rmi4_f11_destroy_sensor(sensor);
	}
}

static void rmi4_f11_dump_query(struct rmi4_function_device *fdev,
				struct f11_2d_sensor_query *squery)
{
	struct rmi4_f11_data *data = dev_get_drvdata(&fdev->dev);
	struct f11_2d_device_query *dquery = &data->dev_query;

	dev_info(&fdev->dev, "Device supports:\n");
	dev_info(&fdev->dev, "Num sensors: %d\n", dquery->nbr_of_sensors);
	dev_info(&fdev->dev, "has query 9: %s\n",
		 dquery->has_query9 ? "Yes" : "No");
	dev_info(&fdev->dev, "has query 11: %s\n",
		 dquery->has_query11 ? "Yes" : "No");

	dev_info(&fdev->dev, "Sensor supports:\n");
	dev_info(&fdev->dev, "Number of fingers: %d\n",
		 squery->number_of_fingers);
	dev_info(&fdev->dev, "Has rel: %s\n", squery->has_rel ? "Yes" : "No");
	dev_info(&fdev->dev, "Has abs: %s\n", squery->has_abs ? "Yes" : "No");
	dev_info(&fdev->dev, "Has gestures: %s\n",
		 squery->has_gestures ? "Yes" : "No");
	dev_info(&fdev->dev, "Sensitivity adjust: %s\n",
		 squery->has_sensitivity_adjust ? "Yes" : "No");
	dev_info(&fdev->dev, "Configurable: %s\n",
		 squery->configurable ? "Yes" : "No");
	dev_info(&fdev->dev, "Num X electrodes: %d\n",
		 squery->num_of_x_electrodes);
	dev_info(&fdev->dev, "Num Y electrodes: %d\n",
		 squery->num_of_y_electrodes);
	dev_info(&fdev->dev, "Max electrodes: %d\n",
		 squery->max_electrodes);
}

static void rmi_f11_rel_pos_report(struct rmi4_function_device *fdev,
				   struct rmi4_f11_2d_sensor *sensor,
				   u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger].delta_x;
	y = data->rel_pos[n_finger].delta_y;

	x = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)x));
	y = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)y));

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
	}
	/*
	** question: if either x or y (or both) are flipped,
	** then should delta x and/or delta y be flipped as well?
	*/

	if (x || y) {
		input_report_rel(sensor->input, REL_X, x);
		input_report_rel(sensor->input, REL_Y, y);
		input_report_rel(sensor->mouse_input, REL_X, x);
		input_report_rel(sensor->mouse_input, REL_Y, y);
	}
	input_sync(sensor->mouse_input);
}

static void rmi_f11_abs_pos_report(struct rmi4_function_device *fdev,
				   struct rmi4_f11_2d_sensor *sensor,
				   u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	int x, y, z;
	int w_x, w_y, w_max, w_min, orient;
	int temp;

	x = ((data->abs_pos[n_finger].x_msb << 4) |
	     data->abs_pos[n_finger].x_lsb);
	y = ((data->abs_pos[n_finger].y_msb << 4) |
	     data->abs_pos[n_finger].y_lsb);
	z = data->abs_pos[n_finger].z;
	w_x = data->abs_pos[n_finger].w_x;
	w_y = data->abs_pos[n_finger].w_y;
	w_max = max(w_x, w_y);
	w_min = min(w_x, w_y);

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
		temp = w_x;
		w_x = w_y;
		w_y = temp;
		}

	orient = w_x > w_y ? 1 : 0;

	if (axis_align->flip_x)
		x = max(sensor->max_x - x, 0);

	if (axis_align->flip_y)
		y = max(sensor->max_y - y, 0);

	/*
	** here checking if X offset or y offset are specified is
	**  redundant.  We just add the offsets or, clip the values
	**
	** note: offsets need to be done before clipping occurs,
	** or we could get funny values that are outside
	** clipping boundaries.
	*/
	x += axis_align->offset_X;
	y += axis_align->offset_Y;
	x =  max(axis_align->clip_X_low, x);
	y =  max(axis_align->clip_Y_low, y);
	if (axis_align->clip_X_high)
		x = min(axis_align->clip_X_high, x);
	if (axis_align->clip_Y_high)
		y =  min(axis_align->clip_Y_high, y);

	dev_dbg(&fdev->dev,
		"%s: f_state[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
		__func__, n_finger, finger_state, x, y, z, w_max, w_min);


	input_report_abs(sensor->input, ABS_MT_PRESSURE, z);
	input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, w_max);
	input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, w_min);
	input_report_abs(sensor->input, ABS_MT_ORIENTATION, orient);
	input_report_abs(sensor->input, ABS_MT_POSITION_X, x);
	input_report_abs(sensor->input, ABS_MT_POSITION_Y, y);
	input_report_abs(sensor->input, ABS_MT_TRACKING_ID, n_finger);
	input_report_key(sensor->input, BTN_TOUCH, !!finger_state);
	/* MT sync between fingers */
	input_mt_sync(sensor->input);
}

static void rmi_f11_finger_handler(struct rmi4_function_device *fdev,
				   struct rmi4_f11_2d_sensor *sensor)
{
	const struct f11_2d_data_0 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = (f_state[i >> 2].finger_n &
				F11_FINGER_STATE_MASK_N(i));
		finger_state = F11_FINGER_STATE_VAL_N(finger_state, i);

		if (finger_state == F11_RESERVED) {
			dev_err(&fdev->dev,
				"%s: Invalid finger state[%d]:0x%02x.",
				__func__, i, finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
			   (finger_state == F11_INACCURATE)) {
			finger_pressed_count++;

			dev_dbg(&fdev->dev,
				 "Finger %d is present. Reporting\n", i);

			if (sensor->data.abs_pos)
				rmi_f11_abs_pos_report(fdev, sensor,
						       finger_state, i);

			if (sensor->data.rel_pos)
				rmi_f11_rel_pos_report(fdev, sensor, i);
		}
	}

	if (!finger_pressed_count) {
		dev_dbg(&fdev->dev, "%s - All fingers released\n", __func__);
		input_mt_sync(sensor->input);
	} else {
		dev_dbg(&fdev->dev, "%d fingers used\n", finger_pressed_count);
	}

	input_sync(sensor->input);
}

static int rmi4_f11_2d_construct_data(struct rmi4_function_device *fdev,
				      struct rmi4_f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_query *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->number_of_fingers == 5 ? 10 :
				query->number_of_fingers + 1);

	sensor->pkt_size = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs)
		sensor->pkt_size += (sensor->nbr_fingers * 5);

	if (query->has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (query->f11_2d_query7__8[0])
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1])
		sensor->pkt_size += sizeof(u8);

	if (query->has_pinch || query->has_flick || query->has_rotate) {
		sensor->pkt_size += 3;
		if (!query->has_flick)
			sensor->pkt_size--;
		if (!query->has_rotate)
			sensor->pkt_size--;
	}

	if (query->has_touch_shapes)
		sensor->pkt_size += F11_CEIL(query->nbr_touch_shapes + 1, 8);

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = (struct f11_2d_data_0 *)sensor->data_pkt;
	i = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		data->abs_pos = (struct f11_2d_data_1_5 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 5);
	}

	if (query->has_rel) {
		data->rel_pos = (struct f11_2d_data_6_7 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 2);
	}

	if (query->f11_2d_query7__8[0]) {
		data->gest_1 = (struct f11_2d_data_8 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1]) {
		data->gest_2 = (struct f11_2d_data_9 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_pinch) {
		data->pinch = (struct f11_2d_data_10 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_flick) {
		if (query->has_pinch) {
			data->flick = (struct f11_2d_data_10_12 *)data->pinch;
			i += 2;
		} else {
			data->flick = (struct f11_2d_data_10_12 *)
					&sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->has_rotate) {
		if (query->has_flick) {
			data->rotate = (struct f11_2d_data_11_12 *)
					(data->flick + 1);
		} else {
			data->rotate = (struct f11_2d_data_11_12 *)
					&sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->has_touch_shapes)
		data->shapes = (struct f11_2d_data_13 *)&sensor->data_pkt[i];

	return 0;
}

static int rmi4_f11_get_query_parameters(struct rmi4_function_device *fdev,
					 struct f11_2d_sensor_query *query,
					 u16 query_offset)
{
	int rc;

	dev_dbg(&fdev->dev, "%s - Query index = 0x%02X\n", __func__,
		 query_offset);

	rc = rmi4_bus_read(fdev, QUERY, query_offset,
			   (u8 *)&query->f11_2d_query1__4,
			   sizeof(query->f11_2d_query1__4));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read query1__4\n",
			__func__);
		return rc;
	}
	query_offset += rc;

	dev_dbg(&fdev->dev, "%s q1-4 read\n", __func__);
	dev_dbg(&fdev->dev, "%s - Query index = 0x%02X\n", __func__,
		 query_offset);
	rc = rmi4_bus_read(fdev, QUERY, query_offset, &query->f11_2d_query5,
			  sizeof(query->f11_2d_query5));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read query5\n",
			__func__);
		return rc;
	}
	query_offset += rc;

	dev_dbg(&fdev->dev, "%s q5 read\n", __func__);
	dev_dbg(&fdev->dev, "%s - Query index = 0x%02X\n", __func__,
		 query_offset);
	rc = rmi4_bus_read(fdev, QUERY, query_offset, &query->f11_2d_query6,
			   sizeof(query->f11_2d_query6));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read query6\n",
			__func__);
		return rc;
	}
	query_offset += rc;

	dev_dbg(&fdev->dev, "%s q6 read\n", __func__);
	dev_dbg(&fdev->dev, "%s - Query index = 0x%02X\n", __func__,
		 query_offset);
	rc = rmi4_bus_read(fdev, QUERY, query_offset,
			   (u8 *)&query->f11_2d_query7__8,
			   sizeof(query->f11_2d_query7__8));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read query7__8\n",
			__func__);
		return rc;
	}
	query_offset += rc;

	dev_dbg(&fdev->dev, "%s q7_8 read\n", __func__);
	dev_dbg(&fdev->dev, "%s - Query index = 0x%02X\n", __func__,
		 query_offset);
	rc = rmi4_bus_read(fdev, QUERY, query_offset, &query->f11_2d_query10,
			   sizeof(query->f11_2d_query10));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read query10\n",
			__func__);
		return rc;
	}

	dev_dbg(&fdev->dev, "%s q10 read\n", __func__);
	dev_dbg(&fdev->dev, "%s - Query index = 0x%02X\n", __func__,
		 query_offset + rc);

	return query_offset + rc;
}

static void rmi4_f11_irq_handler(int irq, void *data)
{
	struct list_head *list;
	struct rmi4_f11_2d_sensor *sensor;
	struct rmi4_function_device *fdev = data;
	struct rmi4_f11_data *ddata = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	list_for_each(list, &ddata->sensor_list) {
		int error;
		int addr_offset = 0;
		sensor = to_rmi4_f11_2d_sensor(list);
		error = rmi4_bus_read(fdev, DATA, addr_offset, sensor->data_pkt,
				      sensor->pkt_size);
		if (0 > error)
			return;

		rmi_f11_finger_handler(fdev, sensor);
		addr_offset += sensor->pkt_size;
	}
}

static int rmi4_f11_apply_pf_data(struct rmi4_function_device *fdev,
				  struct rmi4_f11_2d_sensor *sensor)
{
	int rc;
	u16 max_x;
	u16 max_y;

	/* TODO: Implement handling of platform data. Need to know exactly what
	 *	 is and can be supported by chip in order to get this correct */

	/* Max X and max Y */
	/* TODO: Handle X/Y swapping if required */
	rc = rmi4_bus_read(fdev, CONTROL, F11_CTRL_SENSOR_MAX_X_POS_OFFSET,
			   (u8 *)&max_x, sizeof(max_x));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read MAX X position\n",
			__func__);
		goto error;
	}

	rc = rmi4_bus_read(fdev, CONTROL, F11_CTRL_SENSOR_MAX_Y_POS_OFFSET,
			   (u8 *)&max_y, sizeof(max_y));
	if (0 > rc) {
		dev_err(&fdev->dev, "%s - Failed to read MAX Y position\n",
			__func__);
		goto error;
	}

	sensor->max_x = max_x;
	sensor->max_y = max_y;

error:
	return rc > 0 ? 0 : rc;
}

static void rmi4_f11_set_abs_params(struct rmi4_function_device *fdev,
				    struct rmi4_f11_2d_sensor *sensor)
{
	int x_min = 0;
	int y_min = 0;
	int x_max = sensor->max_x;
	int y_max = sensor->max_y;

	/* TODO: Implement clipping and ralated stuff */
	input_set_abs_params(sensor->input, ABS_MT_PRESSURE, 0,
			     DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
	input_set_abs_params(sensor->input, ABS_MT_TOUCH_MAJOR,
			     0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(sensor->input, ABS_MT_TOUCH_MINOR,
			     0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(sensor->input, ABS_MT_ORIENTATION,
			     0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(sensor->input, ABS_MT_TRACKING_ID,
			     DEFAULT_MIN_ABS_MT_TRACKING_ID,
			     DEFAULT_MAX_ABS_MT_TRACKING_ID, 0, 0);
	/* TODO get max_x_pos (and y) from control registers. */
	input_set_abs_params(sensor->input, ABS_MT_POSITION_X,
			     x_min, x_max, 0, 0);
	input_set_abs_params(sensor->input, ABS_MT_POSITION_Y,
			     y_min, y_max, 0, 0);
}

static int rmi4_f11_create_input_dev(struct rmi4_function_device *fdev,
				     struct rmi4_f11_2d_sensor *sensor)
{
	int err = 0;
	char tmp_buf[100];

	sensor->input = input_allocate_device();
	if (!sensor->input) {
		dev_err(&fdev->dev, "%s - Failed to allocate input device\n",
			__func__);
		err = -ENOMEM;
		goto exit;
	}

	snprintf(tmp_buf, sizeof(tmp_buf), "%s_sensor%d",
		 dev_name(&fdev->dev), sensor->sensor_index);
	sensor->input->name = kstrdup(tmp_buf, GFP_KERNEL);
	snprintf(tmp_buf, sizeof(tmp_buf), "%s/input%d",
		 dev_name(&fdev->dev), sensor->sensor_index);
	sensor->input->phys = kstrdup(tmp_buf, GFP_KERNEL);

	sensor->input->dev.parent = &fdev->dev;
	input_set_drvdata(sensor->input, dev_get_drvdata(&fdev->dev));

	set_bit(EV_SYN, sensor->input->evbit);
	set_bit(EV_KEY, sensor->input->evbit);
	set_bit(EV_ABS, sensor->input->evbit);
	set_bit(BTN_TOUCH, sensor->input->keybit);

	rmi4_f11_set_abs_params(fdev, sensor);

	if (sensor->sens_query.has_rel) {
		set_bit(EV_REL, sensor->input->evbit);
		set_bit(REL_X, sensor->input->relbit);
		set_bit(REL_Y, sensor->input->relbit);
	}

exit:
	return err;
}

static int rmi4_f11_create_mouse_dev(struct rmi4_function_device *fdev,
				     struct rmi4_f11_2d_sensor *sensor)
{
	int err = 0;
	char tmp_buf[100];

	sensor->mouse_input = input_allocate_device();
	if (!sensor->mouse_input) {
		err = -ENOMEM;
		goto exit;
	}

	snprintf(tmp_buf, sizeof(tmp_buf), "%s_mouse%d",
		 dev_name(&fdev->dev), sensor->sensor_index);
	sensor->mouse_input->name = kstrdup(tmp_buf, GFP_KERNEL);
	snprintf(tmp_buf, sizeof(tmp_buf), "%s/mouse%d",
		 dev_name(&fdev->dev), sensor->sensor_index);
	sensor->mouse_input->phys = kstrdup(tmp_buf, GFP_KERNEL);
	input_set_drvdata(sensor->mouse_input, dev_get_drvdata(&fdev->dev));
	sensor->mouse_input->dev.parent = &fdev->dev;

	sensor->mouse_input->id.vendor  = 0x18d1;
	sensor->mouse_input->id.product = 0x0210;
	sensor->mouse_input->id.version = 0x0100;

	set_bit(EV_REL, sensor->mouse_input->evbit);
	set_bit(REL_X, sensor->mouse_input->relbit);
	set_bit(REL_Y, sensor->mouse_input->relbit);

	set_bit(BTN_MOUSE, sensor->mouse_input->evbit);

	set_bit(EV_KEY, sensor->mouse_input->evbit);
	set_bit(BTN_LEFT, sensor->mouse_input->keybit);
	set_bit(BTN_MIDDLE, sensor->mouse_input->keybit);
	set_bit(BTN_RIGHT, sensor->mouse_input->keybit);

exit:
	return err;
}

static int rmi4_f11_register_input_devs(struct rmi4_function_device *fdev)
{
	int err = 0;
	struct list_head *list;
	struct rmi4_f11_2d_sensor *sensor;
	struct rmi4_f11_data *data = dev_get_drvdata(&fdev->dev);

	list_for_each(list, &data->sensor_list) {
		sensor = to_rmi4_f11_2d_sensor(list);
		if (sensor->input) {
			err = input_register_device(sensor->input);
			if (err) {
				dev_err(&fdev->dev,
					"%s - Failed to register input dev\n",
					__func__);
				goto exit;
			}
			sensor->input_dev_registered = true;
		}
		if (sensor->mouse_input) {
			err = input_register_device(sensor->mouse_input);
			if (err) {
				dev_err(&fdev->dev,
					"%s - Failed to register mouse dev\n",
					__func__);
				goto exit;
			}
			sensor->mouse_dev_registered = true;
		}
	}

exit:
	return err;
}

static void rmi4_f11_force_abs_release(struct rmi4_function_device *fdev,
				       struct rmi4_f11_2d_sensor *sensor)
{

	int i;

	if (!sensor->data.abs_pos)
		return;

	for (i = 0; i < sensor->nbr_fingers; i++) {
		dev_dbg(&fdev->dev, "%s - Releasing finger %d\n", __func__, i);
		input_report_abs(sensor->input, ABS_MT_PRESSURE, 0);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, 0);
		input_report_abs(sensor->input, ABS_MT_ORIENTATION, 0);
		input_report_abs(sensor->input, ABS_MT_POSITION_X, 0);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, 0);
		input_report_abs(sensor->input, ABS_MT_TRACKING_ID, i);

		/* MT sync between fingers */
		input_mt_sync(sensor->input);
		sensor->finger_tracker[i] = 0;
	}

}

static void rmi4_f11_notify_callback(enum rmi4_notification_event event,
				     void *data)
{
	struct list_head *n;
	struct list_head *list;
	struct rmi4_f11_2d_sensor *sensor;
	struct rmi4_function_device *fdev = data;
	struct rmi4_f11_data *ddata = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	if (event & RMI4_DRIVER_RESET) {
		list_for_each_safe(list, n, &ddata->sensor_list) {
			sensor = to_rmi4_f11_2d_sensor(list);
			rmi4_f11_force_abs_release(fdev, sensor);
		}
		dev_dbg(&fdev->dev, "%s - Driver has been reset\n", __func__);
	} else if (event & RMI4_CHIP_SUSPEND) {
		list_for_each_safe(list, n, &ddata->sensor_list) {
			sensor = to_rmi4_f11_2d_sensor(list);
			rmi4_f11_force_abs_release(fdev, sensor);
		}
		dev_dbg(&fdev->dev, "%s - Driver has been suspended\n",
			__func__);
	} else {
		dev_warn(&fdev->dev, "%s - Notification not supported\n",
			 __func__);
	}
}

static int rmi4_f11_start(struct rmi4_function_device *fdev)
{
	int i;
	int err;
	int index; /* Used for address offset calculation */
	struct rmi4_f11_data *data;
	struct rmi4_f11_2d_sensor *sensor;
	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&fdev->dev, "%s - Failed to allocate memory\n",
			__func__);
		return -ENOMEM;
	}

	mutex_init(&data->f11_lock);
	INIT_LIST_HEAD(&data->sensor_list);
	dev_set_drvdata(&fdev->dev, data);

	err = rmi4_bus_read(fdev, QUERY, 0, &data->dev_query.f11_2d_query0,
			    sizeof(data->dev_query.f11_2d_query0));
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to read initial data\n",
			__func__);
		goto err_sensor_fail;
	}

	for (i = 0, index = 1; i <= data->dev_query.nbr_of_sensors;
	     i++, index++) {
		dev_dbg(&fdev->dev, "%s - Sensor[%d], read_index = 0x%02X\n",
			 __func__, i, index);

		sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
		if (!sensor) {
			dev_err(&fdev->dev,
				"%s - Failed to allocate memory for sensor\n",
				__func__);
			goto err_sensor_init;
		}
		INIT_LIST_HEAD(&sensor->entry);
		sensor->sensor_index = i;

		err = rmi4_f11_get_query_parameters(fdev, &sensor->sens_query,
						    index);
		if (0 > err) {
			dev_err(&fdev->dev,
				"%s - Failed to read data for sensor %d\n",
				__func__, i);
			goto err_sensor_init;
		}
		index += err;

		rmi4_f11_dump_query(fdev, &sensor->sens_query);

		err = rmi4_f11_apply_pf_data(fdev, sensor);
		if (err) {
			dev_err(&fdev->dev,
				"%s - Failed to read configure sensor %d\n",
				__func__, i);
			goto err_sensor_init;
		}

		rmi4_f11_2d_construct_data(fdev, sensor);

		rmi4_f11_create_input_dev(fdev, sensor);

		if (sensor->sens_query.has_rel)
			rmi4_f11_create_mouse_dev(fdev, sensor);

		list_add_tail(&data->sensor_list, &sensor->entry);
	}

	sensor = NULL;
	err = rmi4_f11_register_input_devs(fdev);
	if (err)
		goto err_sensor_init;

	err = rmi4_bus_request_notification(fdev, RMI4_DRIVER_RESET |
					    RMI4_CHIP_SUSPEND,
					    rmi4_f11_notify_callback, fdev);
	if (err) {
		dev_err(&fdev->dev,
			"%s - Failed to subscribe to notification events\n",
			__func__);
		goto err_sensor_init;
	}

	err = rmi4_bus_request_irq(fdev, fdev, rmi4_f11_irq_handler, 1);
	if (err) {
		dev_err(&fdev->dev, "%s - Failed to subscribe to IRQ\n",
			__func__);
		goto err_notification_request;
	}

	dev_info(&fdev->dev, "Successfully started\n");

	return err;

err_notification_request:
	rmi4_bus_release_notification(fdev, fdev);
err_sensor_init:
	rmi4_f11_unregister_input_devs(fdev);
        rmi4_f11_destroy_sensor(sensor);
err_sensor_fail:
	kfree(data);

	return err;
}

static int rmi4_f11_stop(struct rmi4_function_device *fdev)
{
	struct rmi4_f11_data *data = dev_get_drvdata(&fdev->dev);

	dev_info(&fdev->dev, "%s - Called\n", __func__);
	rmi4_bus_free_irq(fdev, fdev);
	rmi4_f11_unregister_input_devs(fdev);
	kfree(data);
	return 0;
}

static struct rmi4_function_driver rmi4_f11 = {
	.drv = {
		.name = RMI4_DEFAULT_F11_NAME,
	},
	.probe		= rmi4_f11_start,
	.remove	= rmi4_f11_stop,
};

static int __devinit rmi4_f11_init(void)
{
	pr_info("Registering function %s on RMI4 bus\n",
		rmi4_f11.drv.name);
	return rmi4_bus_register_function_driver(&rmi4_f11);
}

static void __devexit rmi4_f11_exit(void)
{
	pr_info("Unregistering function %s from RMI4 bus\n",
		rmi4_f11.drv.name);
	rmi4_bus_unregister_function_driver(&rmi4_f11);
}

module_init(rmi4_f11_init);
module_exit(rmi4_f11_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyericsson.com>");
MODULE_DESCRIPTION("RMI4 F11 function driver");
MODULE_LICENSE("GPL");
