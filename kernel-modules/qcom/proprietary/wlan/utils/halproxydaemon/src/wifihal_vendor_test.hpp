/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef __WIFI_HAL_NUDSTATS_TEST_HPP__
#define __WIFI_HAL_NUDSTATS_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "common.hpp"

namespace NUDStats
{
    /**
     * wlan_nud_stats_data_pkt_flags: Flag representing the various
     * data types for which the stats have to get collected.
     */
    enum wlan_nud_stats_data_pkt_flags {
            WLAN_NUD_STATS_DATA_ARP = 1 << 0,
            WLAN_NUD_STATS_DATA_DNS = 1 << 1,
            WLAN_NUD_STATS_DATA_TCP_HANDSHAKE = 1 << 2,
            WLAN_NUD_STATS_DATA_ICMPV4 = 1 << 3,
            WLAN_NUD_STATS_DATA_ICMPV6 = 1 << 4,
            WLAN_NUD_STATS_DATA_TCP_SYN = 1 << 5,
            WLAN_NUD_STATS_DATA_TCP_SYN_ACK = 1 << 6,
            WLAN_NUD_STATS_DATA_TCP_ACK = 1 << 7,
    };
    class NUDStatsTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *NUD_CMD;
        static const char *NUD_SET;
        static const char *NUD_GET;
        static const char *NUD_CLEAR;
        struct in_addr setip;
        NUDStatsTestSuite(wifi_handle handle);
        /* process the command line args */
        wifi_error processCmd(int argc, char **argv);

        void Usage();
    private:
        wifi_handle wifiHandle_;
        wifi_interface_handle ifaceHandle;
    };
}
#endif
