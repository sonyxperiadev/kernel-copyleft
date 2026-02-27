/*
* Copyright (c) 2014-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFI_HAL_LLSTATS_TEST_HPP__
#define __WIFI_HAL_LLSTATS_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>
#include "common.hpp"
namespace LLStats
{
    class LLStatsTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *LL_CMD;
        static const char *LL_SET;
        static const char *LL_GET;
        static const char *LL_CLEAR;

        LLStatsTestSuite(wifi_handle handle);
        wifi_error processCmd(int argc, char **argv);
        wifi_error  wifi_set_link_stats_test(wifi_interface_handle iface,
                                             u32 mpduSizeThreshold,
                                             u32 aggressiveStatisticsGathering);
        wifi_error wifi_get_link_stats_test(wifi_interface_handle iface);
        wifi_error wifi_clr_link_stats_test(wifi_interface_handle iface,
                                      u32 stats_clear_req_mask,
                                      u8 stop_req);

        void Usage();
        /* process the command line args */
    private:
        wifi_handle wifiHandle_;
        wifi_interface_handle ifaceHandle;

        /* Send a LLStats command to Android HAL */
    };
}
#endif
