/*
* Copyright (c) 2016 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*
*/

#include "nd_offload.hpp"
#include "common.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
extern wifi_hal_fn fn;

namespace NDO_TEST
{
    /* CLI cmd strings */
    const char *NDOTestSuite::NDO_CMD             = "NDO";

    /* Constructor */
    NDOTestSuite::NDOTestSuite(wifi_handle handle)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "NDOTestSuite \n.");
    }

    /* process the command line args */
    wifi_error NDOTestSuite::executeCmd(int argc, char **argv)
    {
        wifi_error ret;
        int cmdId;

        ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[ARGV_IFACE_INDEX]);
        if(!ifaceHandle)
        {
            fprintf(stderr, "Interface %s is not up, exiting.\n",
                    argv[ARGV_IFACE_INDEX]);
            return WIFI_ERROR_INVALID_ARGS;
        }

        if (argc > ARGV_CMDID_INDEX) {
            if (strcasecmp(argv[ARGV_CMDID_INDEX], "enable") == 0) {
                ret = NDOConfigureNdOffload(ifaceHandle, ENABLE);
            } else if (strcasecmp(argv[ARGV_CMDID_INDEX], "disable") == 0) {
                ret = NDOConfigureNdOffload(ifaceHandle, DISABLE);
            } else {
                ret = WIFI_ERROR_INVALID_ARGS;
                fprintf(stderr, "%s: Unknown Cmd\n", __func__);
                Usage();
            }
        } else {
            while (1) {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Enter ID for ND offload Cmd, as follows:\n");
                fprintf(stderr, "1. Enable ND offload\n");
                fprintf(stderr, "2. Disable ND offload\n");
                fprintf(stderr, "Type 1000 to exit.\n");
                fprintf(stderr, "*********************\n");
                read_int(&cmdId);
		if (cmdId < 0 || cmdId > INT_MAX) {
			fprintf(stderr, "Invalid Cmd : %d \n", cmdId);
			continue;
		}

                if (cmdId == 1000) {
                    ret = WIFI_SUCCESS;
                    break;
                } else if (cmdId < NDO_CMD_ENABLE_OFFLOAD ||
                           cmdId > NDO_CMD_DISABLE_OFFLOAD) {
                    fprintf(stderr, "Invalid Cmd : %d \n", cmdId);
                    continue;
                }

               switch(cmdId) {
                   case NDO_CMD_ENABLE_OFFLOAD:
                       ret = NDOConfigureNdOffload(ifaceHandle, ENABLE);
                       break;
                   case NDO_CMD_DISABLE_OFFLOAD:
                       ret = NDOConfigureNdOffload(ifaceHandle, DISABLE);
                       break;
                   default:
                       ret = WIFI_ERROR_INVALID_ARGS;
                       fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
               }
            }
        }

        return ret;
    }

    wifi_error NDOTestSuite::NDOConfigureNdOffload(
                                            wifi_interface_handle ifaceHandle,
                                            u8 enable)
    {
        wifi_error ret;

        fprintf(stderr, "%s ND offload \n", enable?"enable":"disable");

        if (!fn.wifi_configure_nd_offload) {
            fprintf(stderr, "No API registered for wifi_configure_nd_offload\n");
            return WIFI_ERROR_UNINITIALIZED;
        }
        ret = (fn.wifi_configure_nd_offload)(ifaceHandle,
                                          enable);
        fprintf(stderr, "ND offload %s ret=%d\n", ret?"FAILED":"SUCCESS", ret);
        return ret;
    }

    void NDOTestSuite::Usage()
    {
        fprintf(stderr, "hal_proxy_daemon ndo wlan0 enable");
        fprintf(stderr, "hal_proxy_daemon ndo wlan0 disable");
    }

}
