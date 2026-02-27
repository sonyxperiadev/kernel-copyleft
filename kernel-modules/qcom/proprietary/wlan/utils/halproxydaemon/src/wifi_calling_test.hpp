/*
* Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFI_HAL_WFC_TEST_HPP__
#define __WIFI_HAL_WFC_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>

typedef struct {
    u32 period_msec;
    u8 src_mac_addr[6];
    u8 dst_mac_addr[6];
    u16 ip_packet_len;
    u16 ether_type;
    u8 ip_packet[0];
} startSendOffloadPkts;

typedef struct {
    s8 max_rssi;
    s8 min_rssi;
} startMonitoringRssi;

namespace WFC_TEST
{
    class WFCTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *WFC_CMD;
        static const char *SEND_OFFLOADED_PKTS;
        static const char *RSSI_MONITORING;

        WFCTestSuite(wifi_interface_handle handle, wifi_request_id request_id);
        /* process the command line args */
        void processCmd(int cmdIndex, void *input_buf);

        /* execute the command line args */
        void executeCmd(int argc, char **argv);

        void setRequestId(int reqId);
        int getRequestId();

        /* process the command line args */
    private:
        wifi_interface_handle wifiHandle_;
        wifi_request_id id;

        /* Send a WFC command to Android HAL */
        void WFCSendOffloadedPktsStartRequest(startSendOffloadPkts *buf);
        void WFCSendOffloadedPktsStopRequest(void);
        void WFCSendStartRssiMonitoringRequest(startMonitoringRssi *buf);
        void WFCSendStopRssiMonitoringRequest(void);
    };
}

#endif
