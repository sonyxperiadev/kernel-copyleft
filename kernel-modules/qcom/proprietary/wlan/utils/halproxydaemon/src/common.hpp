/*
* Copyright (c) 2016-2017, 2019 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*
*/

#ifndef __WIFI_HAL_COMMON_HPP__
#define __WIFI_HAL_COMMON_HPP__
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#define MAC_ADDR_LEN 6
#define MAC_OUI_LEN 3
#define ARGV_IFACE_INDEX 2
#define ARGV_CMDID_INDEX 3
#define UNUSED(x)      (void)(x)
#define MAX_WLAN_MAC_PKT_LENGTH 2304
#define MAX_SLEEP_DURATION 1200
#define MAX_CHANNEL_LIST 128
#define MAX_ITERATIONS 20
#define MAX_NUM_INPUT 10
#define RTT_MAX_ITERATIONS 200
#define RTT_MAX_SLEEP_DURATION 12000 // 12 sec max

int mac_addr_aton(u8 *mac_addr, const char *txt, size_t length);
int mac_addr_aton(u8 *mac_addr, const char *txt);
s8 char_to_hex(char c);
int validate_sleep_duration(int sleep_time);
int validate_iterations_count(int iter_count);

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
extern wifi_hal_fn fn;

static inline void read_int(int *num)
{
    char input[MAX_NUM_INPUT];

    fgets(input, sizeof(input), stdin);
    *num = strtol(input, NULL, 10);
}

static inline void read_uint(unsigned int *num)
{
    char input[MAX_NUM_INPUT];

    fgets(input, sizeof(input), stdin);
    *num = strtoul(input, NULL, 10);
}

static inline void read_u16_from_hex(unsigned short int *num)
{
    char input[MAX_NUM_INPUT];

    fgets(input, sizeof(input), stdin);
    *num = (unsigned short)strtoul(input, NULL, 16);
}

static inline void read_string(char *str, int size)
{
    int len;

    fgets(str, size, stdin);

    /* Remove newline, if present */
    len = strlen(str) - 1;
    if (len >= 0) {
        if (str[len] == '\n')
            str[len] = '\0';
    }
}

#endif
