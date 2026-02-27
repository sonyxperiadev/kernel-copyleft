/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef __WIFI_HAL_PKT_FILTER_HPP
#define __WIFI_HAL_PKT_FILTER_HPP

#include "wifi_hal.h"

#define WPF_IFACE_IDX 2
#define WPF_CMD_IDX 3
#define WPF_CMD_START_FILTERING 1
#define WPF_CMD_STOP_FILTERING 2
#define WPF_CMD_GET_FILTER_INFO 3
#define PACKET_FILTER_MAX_LENGTH 4096

namespace WPF_TEST
{
    class WPFTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *WPF_CMD;
        static const char *SET;
        static const char *RESET;
        static const char *GET_INFO;

        WPFTestSuite(wifi_handle handle);

        /* execute the command line args */
        wifi_error executeCmd(int argc, char **argv);

        /* process the command line args */
    private:
        wifi_handle wifiHandle_;
        wifi_interface_handle ifaceHandle;

        /* Send a WPF command to Android HAL */
        wifi_error WPFSendFilteringStartRequest(
                        wifi_interface_handle ifaceHandle);
        wifi_error WPFSendFilteringStopRequest(
                        wifi_interface_handle ifaceHandle);
        wifi_error WPFGetFilterFilterInfo(
                        wifi_interface_handle ifaceHandle);
        void Usage();
    };
}

#endif
