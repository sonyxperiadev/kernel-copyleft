/*
 * controller's Nordic Hal Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef X_NORDICIOHELPER_H
#define X_NORDICIOHELPER_H

#include <jni.h>
#include <stdint.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <math.h>
#include <cutils/log.h>
#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct nordic_command_helper_ops {
    int (*nordic_command_helper_start)();

    int (*nordic_command_helper_stop)();

    int (*nordic_command_helper_get_nordic_version)(float *version);

    int (*nordic_command_helper_get_controller_version)(float *version, int lr);

    int (*nordic_command_helper_bind_controller)(int lr);

    int (*nordic_command_helper_unbind_controller)(int lr);

    int (*nordic_command_helper_cancel_bind_controller)();

    int (*nordic_command_helper_get_bind_state)(int *bind_state);

    int (*nordic_command_helper_enter_dfu_mode)(int object);

    int (*nordic_command_helper_set_led)(uint8_t enable);

    int (*nordic_command_helper_set_vibration)(int value);

    int (*nordic_command_helper_get_next_data)(void *udata, uint64_t *pts, int8_t *size);

    int (*nordic_command_helper_get_data_size)();

    int (*nordic_command_helper_get_the_last_data)(void *udata, uint64_t *pts, int8_t *size);
} nordic_command_helper_ops_t;

#define NORDIC_COMMAND_HELPER_LIB "libnordic_command.so"

typedef struct {
    void *libHandle;
    nordic_command_helper_ops_t *ops;
    int api_version;
} nordic_command_helper_t;

static inline nordic_command_helper_t *Nordic_Command_Helper_Create() {
    nordic_command_helper_t *me = (nordic_command_helper_t *)
            malloc(sizeof(nordic_command_helper_t));
    if (!me) return NULL;

    me->libHandle = dlopen(NORDIC_COMMAND_HELPER_LIB, RTLD_NOW);
    if (!me->libHandle) {
        __android_log_print(ANDROID_LOG_INFO, "NORDIC_COMMAND_HELPER_LIB", "NORDIC_COMMAND_HELPER_LIB %s", dlerror());
        free(me);
        return NULL;
    }

    me->ops = (nordic_command_helper_ops_t *) malloc(sizeof(nordic_command_helper_ops_t));
    if (!me->ops) {
        dlclose(me->libHandle);
        free(me);
        return NULL;
    }
    me->api_version = 1;

    typedef int (*nordic_command_helper_start_fn)(void);
    me->ops->nordic_command_helper_start = (nordic_command_helper_start_fn) dlsym(me->libHandle, "init");

    typedef int (*nordic_command_helper_stop_fn)(void);
    me->ops->nordic_command_helper_stop = (nordic_command_helper_stop_fn) dlsym(me->libHandle, "deinit");

    typedef int (*nordic_command_helper_get_nordic_version_fn)(float *version);
    me->ops->nordic_command_helper_get_nordic_version = (nordic_command_helper_get_nordic_version_fn) dlsym(me->libHandle, "get_nordic_version");

    typedef int (*nordic_command_helper_get_controller_version_fn)(float *version, int lr);
    me->ops->nordic_command_helper_get_controller_version = (nordic_command_helper_get_controller_version_fn) dlsym(me->libHandle, "get_controller_version");

    typedef int (*nordic_command_helper_bind_controller_fn)(int lr);
    me->ops->nordic_command_helper_bind_controller = (nordic_command_helper_bind_controller_fn) dlsym(me->libHandle, "bind_controller");

    typedef int (*nordic_command_helper_unbind_controller_fn)(int lr);
    me->ops->nordic_command_helper_unbind_controller = (nordic_command_helper_unbind_controller_fn) dlsym(me->libHandle, "unbind_controller");

    typedef int (*nordic_command_helper_cancel_bind_controller_fn)(void);
    me->ops->nordic_command_helper_cancel_bind_controller = (nordic_command_helper_cancel_bind_controller_fn) dlsym(me->libHandle, "cancel_bind_controller");

    typedef int (*nordic_command_helper_get_bind_state_fn)(int *bind_state);
    me->ops->nordic_command_helper_get_bind_state = (nordic_command_helper_get_bind_state_fn) dlsym(me->libHandle, "get_bind_state");

    typedef int (*nordic_command_helper_enter_dfu_mode_fn)(int object);
    me->ops->nordic_command_helper_enter_dfu_mode = (nordic_command_helper_enter_dfu_mode_fn) dlsym(me->libHandle, "enter_dfu_mode");

    typedef int (*nordic_command_helper_set_led_fn)(uint8_t enable);
    me->ops->nordic_command_helper_set_led = (nordic_command_helper_set_led_fn) dlsym(me->libHandle, "set_led");

    typedef int (*nordic_command_helper_set_vibration_fn)(int value);
    me->ops->nordic_command_helper_set_vibration = (nordic_command_helper_set_vibration_fn) dlsym(me->libHandle, "set_vibration");

    typedef int (*nordic_command_helper_get_next_data_fn)(void *udata, uint64_t *pts, int8_t *size);
    me->ops->nordic_command_helper_get_next_data = (nordic_command_helper_get_next_data_fn) dlsym(me->libHandle, "get_next_data");

    typedef int (*nordic_command_helper_get_data_size_fn)(void);
    me->ops->nordic_command_helper_get_data_size = (nordic_command_helper_get_data_size_fn) dlsym(me->libHandle, "get_data_size");

    typedef int (*nordic_command_helper_get_the_last_data_fn)(void *udata, uint64_t *pts, int8_t *size);
    me->ops->nordic_command_helper_get_the_last_data = (nordic_command_helper_get_the_last_data_fn) dlsym(me->libHandle, "get_the_last_data");

    return me;
}

static inline void Nordic_Command_Helper_Destroy(nordic_command_helper_t *me) {
    if (!me) return;

    if (me->libHandle) {
        dlclose(me->libHandle);
    }
    free(me);
    me = NULL;
}

static inline int Nordic_Command_Helper_Start(nordic_command_helper_t *me) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_start) {
        return me->ops->nordic_command_helper_start();
    }

    return -1;
}

static inline int Nordic_Command_Helper_Stop(nordic_command_helper_t *me) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_stop) {
        return me->ops->nordic_command_helper_stop();
    }
    return -1;
}

static inline int Nordic_Command_Helper_Get_Nordic_Version(nordic_command_helper_t *me, float *version) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_get_nordic_version) {
        return me->ops->nordic_command_helper_get_nordic_version(version);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Get_Controller_Version(nordic_command_helper_t *me, float *version, int lr) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_get_controller_version) {
        return me->ops->nordic_command_helper_get_controller_version(version, lr);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Bind_Controller(nordic_command_helper_t *me, int lr) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_bind_controller) {
        return me->ops->nordic_command_helper_bind_controller(lr);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Unbind_Controller(nordic_command_helper_t *me, int lr) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_unbind_controller) {
        return me->ops->nordic_command_helper_unbind_controller(lr);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Cancel_Bind(nordic_command_helper_t *me) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_cancel_bind_controller) {
        return me->ops->nordic_command_helper_cancel_bind_controller();
    }
    return -1;
}

static inline int Nordic_Command_Helper_Get_Bind_State(nordic_command_helper_t *me, int *bind_state) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_get_bind_state) {
        return me->ops->nordic_command_helper_get_bind_state(bind_state);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Enter_DFU_Mode(nordic_command_helper_t *me, int object) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_enter_dfu_mode) {
        return me->ops->nordic_command_helper_enter_dfu_mode(object);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Set_Led(nordic_command_helper_t *me, uint8_t enable) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_set_led) {
        return me->ops->nordic_command_helper_set_led(enable);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Set_Vibration(nordic_command_helper_t *me, int value) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_set_vibration) {
        return me->ops->nordic_command_helper_set_vibration(value);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Get_Next_Data(nordic_command_helper_t *me, void *udata, uint64_t *pts, int8_t *size) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_get_next_data) {
        return me->ops->nordic_command_helper_get_next_data(udata, pts, size);
    }
    return -1;
}

static inline int Nordic_Command_Helper_Get_Data_Size(nordic_command_helper_t *me) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_get_data_size) {
        return me->ops->nordic_command_helper_get_data_size();
    }
    return -1;
}

static inline int Nordic_Command_Helper_Get_The_Lst_Data(nordic_command_helper_t *me, void *udata, uint64_t *pts, int8_t *size) {
    if (!me) return -2;
    if (me->ops->nordic_command_helper_get_the_last_data) {
        return me->ops->nordic_command_helper_get_the_last_data(udata, pts, size);
    }
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif //X_NORDICIOHELPER_H
