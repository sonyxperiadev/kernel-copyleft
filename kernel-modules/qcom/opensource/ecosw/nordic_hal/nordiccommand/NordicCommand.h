/*
 * controller's Command Interface for Nordic Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef X_NORDICCOMMAND_H
#define X_NORDICCOMMAND_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

int init(void);

int deinit(void);

int get_nordic_version(float *ver);

int get_controller_version(float *ver, int lr);

int bind_controller(int lr);

int unbind_controller(int lr);

int cancel_bind_controller(void);

int get_bind_state(int *bind_state);

int enter_dfu_mode(int object);

int set_led(int enable);

int set_vibration(int value);

int get_next_data(void *udata, uint64_t *pts, int8_t *size);

int get_data_size(void);

int get_the_last_data(void *udata, uint64_t *pts, int8_t *size);

#ifdef __cplusplus
};
#endif

#endif //X_NORDICCOMMAND_H
