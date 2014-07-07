/*
 * RMI4 bus driver.
 * include/linux/rmi4/rmi4_function_11.h
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
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

#ifndef __RMI4_FUNCTION_11__
#define __RMI4_FUNCTION_11__

#define RMI4_DEFAULT_F11_NAME	"f11"
/**
 * struct rmi_f11_axis_alignmen - target axis alignment
 * @swap_axes: set to TRUE if desired to swap x- and y-axis
 * @flip_x: set to TRUE if desired to flip direction on x-axis
 * @flip_y: set to TRUE if desired to flip direction on y-axis
 */
struct rmi_f11_2d_axis_alignment {
	bool swap_axes;
	bool flip_x;
	bool flip_y;
	int clip_X_low;
	int clip_Y_low;
	int clip_X_high;
	int clip_Y_high;
	int offset_X;
	int offset_Y;
	int rel_report_enabled;
};

struct rmi4_f11_platform_data {
	/* Driver related settings */
	bool swap_axes;
	bool flip_x;
	bool flip_y;
	int clip_X_low;
	int clip_Y_low;
	int clip_X_high;
	int clip_Y_high;
	int offset_X;
	int offset_Y;
	int rel_report_enabled;

	/* Chip related settings */
	/* ctrl0 */
	u8 reporting_mode;
	bool abs_pos_filt;
	bool rel_pos_filt;
	bool rel_ballistics;

	/* ctrl1 */
	u8 palm_detect_thres;
	u8 motion_sensitivity;
	bool man_track_en;
	bool man_tracked_finger

	/* ctrl2__3 */;
	u8 delta_x_threshold;
	u8 delta_y_threshold;

	/* ctrl10 Not supported by Nypon*/
	bool single_tap_int_enable;
	bool tap_n_hold_int_enable;
	bool double_tap_int_enable;
	bool early_tap_int_enable;
	bool flick_int_enable;
	bool press_int_enable;
	bool pinch_int_enable;

	/* ctrl11 */
	bool palm_detect_int_enable;
	bool rotate_int_enable;
	bool touch_shape_int_enable;
	bool scroll_zone_int_enable;
	bool multi_finger_scroll_int_enable;

	/* ctrl12 */
	u8 sensor_map;
	bool xy_sel;

	/* ctrl14 */
	u8 sensitivity_adjustment;
	u8 hysteresis_adjustment;

	/* ctrl27 */
	u8 large_object_sensitivity;
};




#endif /* __RMI4_FUNCTION_11__ */
