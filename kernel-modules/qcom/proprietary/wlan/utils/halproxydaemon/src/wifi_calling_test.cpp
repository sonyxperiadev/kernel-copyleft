/*
* Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "wifi_calling_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
static void on_rssi_threshold_breached(wifi_request_id id, u8 *addr, s8 rssi)
{
    fprintf(stderr, "Ap breached: \t");
    fprintf(stderr, "Request id: %d \n", id);
    fprintf(stderr, " MAC_ADDR: " MAC_ADDR_STR, MAC_ADDR_ARRAY(addr));
    fprintf(stderr, "Rssi    : %d \n", rssi);
}

extern wifi_hal_fn fn;
namespace WFC_TEST
{
    /* CLI cmd strings */
    const char *WFCTestSuite::WFC_CMD             = "WFC";
    const char *WFCTestSuite::SEND_OFFLOADED_PKTS = "send_offloaded_pkts";
    const char *WFCTestSuite::RSSI_MONITORING     = "rssi_monitoring";

    /* Constructor */
    WFCTestSuite::WFCTestSuite(wifi_interface_handle handle,
                               wifi_request_id request_id)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "WFCTestSuite::WFCTestSuite: Created a WFC Test "
            "Suite with request_id:%d\n.", request_id);
        id = request_id;
    }

    void WFCTestSuite::setRequestId(int reqId)
    {
        id = reqId;
    }

    int WFCTestSuite::getRequestId()
    {
        return id;
    }

    /* process the command line args */
    void WFCTestSuite::processCmd(int cmdIndex, void *input_buf)
    {
        fprintf(stderr, "%s: Enter \n", __FUNCTION__);
        switch(cmdIndex) {
            case 1:
                WFCSendOffloadedPktsStartRequest(
                        (startSendOffloadPkts *)input_buf);
                break;
            case 2:
                WFCSendOffloadedPktsStopRequest();
                break;
            case 3:
                WFCSendStartRssiMonitoringRequest(
                        (startMonitoringRssi *)input_buf);
                break;
            case 4:
                WFCSendStopRssiMonitoringRequest();
                break;
            default:
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
        }
    }

    void WFCTestSuite::WFCSendOffloadedPktsStartRequest(
                            startSendOffloadPkts *buf)
    {
        int ret = 0;
        fprintf(stderr, "Sending WFC Start Send Offloaded pkts Request \n");

        if (!fn.wifi_start_sending_offloaded_packet) {
            fprintf(stderr, "No API registered for "
                    "wifi_start_sending_offloaded_packet\n");
            return;
        }
        ret = (fn.wifi_start_sending_offloaded_packet)(getRequestId(),
                                                  wifiHandle_,
                                                  buf->ether_type,
                                                  buf->ip_packet,
                                                  buf->ip_packet_len,
                                                  buf->src_mac_addr,
                                                  buf->dst_mac_addr,
                                                  buf->period_msec);
        fprintf(stderr, "completed. Returned value: %d.\n", ret);
        if (ret)
            return;
    }

    void WFCTestSuite::WFCSendOffloadedPktsStopRequest()
    {
        int ret = 0;
        fprintf(stderr, "Sending WFC Stop Send Offloaded pkts Request \n");

        if (!fn.wifi_stop_sending_offloaded_packet) {
            fprintf(stderr, "No API registered for "
                    "wifi_stop_sending_offloaded_packet\n");
            return;
        }

        ret = (fn.wifi_stop_sending_offloaded_packet)(getRequestId(), wifiHandle_);
        fprintf(stderr, "completed. Returned value: %d.\n", ret);
        if (ret)
            return;
    }

    void WFCTestSuite::WFCSendStartRssiMonitoringRequest(
                            startMonitoringRssi *buf)
    {
        int ret = 0;
        wifi_rssi_event_handler handler;
        fprintf(stderr, "Sending WFC Start monitoring rssi Request \n");

        if (!fn.wifi_start_rssi_monitoring) {
            fprintf(stderr, "No API registered for "
                    "wifi_start_rssi_monitoring\n");
            return;
        }

        /* Set the callback handler functions for related events. */
        handler.on_rssi_threshold_breached = on_rssi_threshold_breached;
        ret = (fn.wifi_start_rssi_monitoring)(getRequestId(),
                                         wifiHandle_,
                                         buf->max_rssi,
                                         buf->min_rssi,
                                         handler);
        fprintf(stderr, "completed. Returned value: %d.\n", ret);
        if (ret)
            return;
    }

    void WFCTestSuite::WFCSendStopRssiMonitoringRequest()
    {
        int ret = 0;
        fprintf(stderr, "Sending WFC Stop Rssi monitoring pkts Request \n");

        if (!fn.wifi_stop_rssi_monitoring) {
            fprintf(stderr, "No API registered for "
                    "wifi_stop_rssi_monitoring\n");
            return;
        }
        ret = (fn.wifi_stop_rssi_monitoring)(getRequestId(), wifiHandle_);
        fprintf(stderr, "completed. Returned value: %d.\n", ret);
        if (ret)
            return;
    }
}
