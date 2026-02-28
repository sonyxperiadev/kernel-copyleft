/*
 * controller's Command Interface for Nordic Service
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef X_COMMOM_H
#define X_COMMOM_H

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "nordic_command: "

#undef ALOGE
#undef ALOGD
#undef ALOGI

#include <android/log.h>

#define  ALOGE(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  ALOGD(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  ALOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

//char const *const MEM_FILE = "/sys/kernel/sc_nordic52840/jsmem";
//char const *const REQUEST_FILE = "/sys/kernel/sc_nordic52840/jsrequest";
char const *const MEM_FILE = "/sys/devices/platform/soc/894000.spi/spi_master/spi0/spi0.0/jsmem";
char const *const REQUEST_FILE = "/sys/devices/platform/soc/894000.spi/spi_master/spi0/spi0.0/jsrequest";

static int JS_COMMAND_GET_VERSION = 1;
static int JS_COMMAND_VIB_STATE = 2;
static int JS_COMMAND_BOND_HAND = 3;
static int JS_COMMAND_DISCONNECT_BOND = 4;
static int JS_COMMAND_BOND_STATE = 5;
static int JS_COMMAND_DFU_STATE = 6;
static int JS_COMMAND_LEFT_PRODUCT_NAME = 7;
static int JS_COMMAND_RIGHT_PRODUCT_NAME = 8;
static int JS_COMMAND_LEFT_FW_VERSION = 9;
static int JS_COMMAND_RIGHT_FW_VERSION = 10;
static int JS_COMMAND_CANCEL_BOND = 11;
static int JS_COMMAND_NORDIC_STATE = 12;
static int JS_COMMAND_LED_STATE = 13;

static int JS_COMMAND_NEEDACK = 1;
static int JS_COMMAND_NO_NEEDACK = 0;

static int TEMP_BUFFER_SIZE = 40;

#define MAX_PACK_SIZE 100
#define MAX_DATA_SIZE 32

#endif //X_COMMOM_H
