/*
* Copyright (c) 2014-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/


#include "tdls_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include "tdls.h"
#include "common.hpp"

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
static void tdls_on_tdls_state_changed(mac_addr addr, wifi_tdls_status status)
{
    fprintf(stderr, "%s\n", __func__);
    fprintf(stderr, " MAC_ADDR: " MAC_ADDR_STR "\n", MAC_ADDR_ARRAY(addr));
    fprintf(stderr, "Channel : %d \n", status.channel);
    fprintf(stderr, "Global_operating_class : %d \n", status.global_operating_class);
    fprintf(stderr, "State : %d \n", status.state);
    fprintf(stderr, "Reason : %d \n", status.reason);
}

namespace TDLS_TEST
{
    /* CLI cmd strings */
    const char *TDLSTestSuite::TDLS_CMD        = "TDLS";
    const char *TDLSTestSuite::TDLS_ENABLE     = "enable";
    const char *TDLSTestSuite::TDLS_GET_STATUS = "get_status";
    const char *TDLSTestSuite::TDLS_DISABLE    = "disable";

    /* Constructor */
    TDLSTestSuite::TDLSTestSuite(wifi_interface_handle handle)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "TDLSTestSuite::TDLSTestSuite: Created a TDLS Test "
            "Suite \n.");
    }

    /* process the command line args */
    void TDLSTestSuite::executeCmd(int argc, char **argv, int cmdIndex,
                                        u8 *mac)
    {
        fprintf(stderr, "%s: Enter \n", __func__);
        switch(cmdIndex) {
            case 1:
                TDLSSendEnableRequest(argc, argv, mac);
                break;
            case 2:
                TDLSSendDisableRequest(argc, argv, mac);
                break;
            case 3:
                TDLSSendGetStatusRequest(argc, argv, mac);
                break;
            case 4:
                TDLSSendGetCapsRequest(argc, argv);
                break;
            default:
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
        }
    }

    /* Helper routine to print usage */
    void TDLSTestSuite::TDLSPrintCmdUsage(char **argv, const char *cmd,
       const char *sub_cmd, const struct option long_options[], int size)
    {
        fprintf(stderr, "Usage: %s %s %s\n", argv[0], cmd, sub_cmd);
        for(int i = 1; i <= size-2; i++)
        {
            if(long_options[i].has_arg)
                fprintf(stderr, "\t[--%s arg]\n", long_options[i].name);
            else
                fprintf(stderr, "\t[--%s]\n", long_options[i].name);
        }
        return;
    }

    void TDLSTestSuite::TDLSSendEnableRequest(int argc, char **argv, u8 *mac)
    {
        UNUSED(argc), UNUSED(argv);
        int ret = 0;
        fprintf(stderr, "%s: Sending TDLS Enable Request. "
            "\n", __FUNCTION__);

        wifi_tdls_handler handler;
        wifi_tdls_params params;

        fprintf(stderr, "Step 4: Enter channel:\n");
        read_int(&params.channel);
        fprintf(stderr, "Step 4: Enter global_operating_class:\n");
        read_int(&params.global_operating_class);
        fprintf(stderr, "Step 4: Enter max_latency_ms:\n");
        read_int(&params.max_latency_ms);
        fprintf(stderr, "Step 4: Enter min_bandwidth_kbps:\n");
        read_int(&params.min_bandwidth_kbps);

        /* Set the callback handler functions for related events. */
        handler.on_tdls_state_changed = tdls_on_tdls_state_changed;

        ret = wifi_enable_tdls(wifiHandle_, mac,
                                &params,
                                handler);
        fprintf(stderr, "%s: Sending TDLS Enable request"
            "completed. Returned value: %d.\n", __func__, ret);

        if (ret)
            return;
    }

    void TDLSTestSuite::TDLSSendDisableRequest(int argc, char **argv, u8 *mac)
    {
        UNUSED(argc), UNUSED(argv);
        int ret = 0;
        fprintf(stderr, "%s: Sending TDLS Enable Request. "
            "\n", __FUNCTION__);

        ret = wifi_disable_tdls(wifiHandle_, mac);
        fprintf(stderr, "%s: Sending TDLS Disable request"
            "completed. Returned value: %d.\n", __func__, ret);

        if (ret)
            return;
    }

    void TDLSTestSuite::TDLSSendGetStatusRequest(int argc, char **argv, u8 *mac)
    {
        UNUSED(argc), UNUSED(argv);
        int ret = 0;
        fprintf(stderr, "%s: Sending TDLS Get status Request. "
            "\n", __FUNCTION__);
        wifi_tdls_status status;
        memset(&status, 0,sizeof(wifi_tdls_status));

        ret = wifi_get_tdls_status(wifiHandle_, mac, &status);
        fprintf(stderr, "%s: Sending TDLS get status"
            "completed. Returned value: %d.\n", __func__, ret);
        fprintf(stderr, "Channel : %d \n", status.channel);
        fprintf(stderr, "Global_operating_class : %d \n", status.global_operating_class);
        fprintf(stderr, "State : %d \n", status.state);
        fprintf(stderr, "Reason : %d \n", status.reason);

        if (ret)
            return;
    }

    void TDLSTestSuite::TDLSSendGetCapsRequest(int argc, char **argv)
    {
        UNUSED(argc), UNUSED(argv);
        int ret = 0;
        fprintf(stderr, "%s: Sending TDLS Get Cabilities Request. "
            "\n", __FUNCTION__);
        wifi_tdls_capabilities capabilities;
        memset(&capabilities, 0,sizeof(wifi_tdls_capabilities));

        ret = wifi_get_tdls_capabilities(wifiHandle_, &capabilities);
        fprintf(stderr, "%s: Sending TDLS get capabilities"
            "completed. Returned value: %d.\n", __func__, ret);
        fprintf(stderr, " max_concurrent_tdls_session_numChannel : %d\n",
                capabilities.max_concurrent_tdls_session_num);
        fprintf(stderr, " is_global_tdls_supported : %d\n",
                capabilities.is_global_tdls_supported);
        fprintf(stderr, " is_per_mac_tdls_supported : %d\n",
                capabilities.is_per_mac_tdls_supported);
        fprintf(stderr, " is_off_channel_tdls_supported : %d \n",
                capabilities.is_off_channel_tdls_supported);
        if (ret)
            return;
    }
}
