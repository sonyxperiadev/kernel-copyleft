/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include "packet_filtering.hpp"
#include <string.h>
#include <stdlib.h>
#include "common.hpp"
#include "limits.h"

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
extern wifi_hal_fn fn;
namespace WPF_TEST
{
    /* CLI cmd strings */
    const char *WPFTestSuite::WPF_CMD             = "WPF";
    const char *WPFTestSuite::SET                 = "SET";
    const char *WPFTestSuite::RESET               = "RESET";
    const char *WPFTestSuite::GET_INFO            = "GET_INFO";

    /* Constructor */
    WPFTestSuite::WPFTestSuite(wifi_handle handle)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "WPFTestSuite \n.");
    }

    /* process the command line args */
    wifi_error WPFTestSuite::executeCmd(int argc, char **argv)
    {
        wifi_error ret;
        int cmdId;

        ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[WPF_IFACE_IDX]);
        if(!ifaceHandle)
        {
            fprintf(stderr, "Interface %s is not up, exiting.\n",
                    argv[WPF_IFACE_IDX]);
            return WIFI_ERROR_INVALID_ARGS;
        }

        if (argc > 3) {
            if (strcasecmp(argv[WPF_CMD_IDX], "set") == 0) {
                ret = WPFSendFilteringStartRequest(ifaceHandle);
            } else if (strcasecmp(argv[WPF_CMD_IDX], "reset") == 0) {
                ret = WPFSendFilteringStopRequest(ifaceHandle);
            } else if (strcasecmp(argv[WPF_CMD_IDX], "get_info") == 0) {
                ret = WPFGetFilterFilterInfo(ifaceHandle);
            } else {
                ret = WIFI_ERROR_INVALID_ARGS;
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
                Usage();
            }
        } else {
            while (1) {
                fprintf(stderr, "**********************\n");
                fprintf(stderr, "Enter ID for WPF Cmd, as follows:\n");
                fprintf(stderr, "1. Set packet filter\n");
                fprintf(stderr, "2. Reset packet filter\n");
                fprintf(stderr, "3. Get packet filter info\n");
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
                } else if (cmdId < 1 || cmdId > 4) {
                    fprintf(stderr, "Invalid Cmd : %d \n", cmdId);
                    continue;
                }

               switch(cmdId) {
                   case WPF_CMD_START_FILTERING:
                       ret = WPFSendFilteringStartRequest(ifaceHandle);
                       break;
                   case WPF_CMD_STOP_FILTERING:
                       ret = WPFSendFilteringStopRequest(ifaceHandle);
                       break;
                   case WPF_CMD_GET_FILTER_INFO:
                       ret = WPFGetFilterFilterInfo(ifaceHandle);
                       break;
                   default:
                       ret = WIFI_ERROR_INVALID_ARGS;
                       fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
               }
            }
        }

        return ret;
    }

    wifi_error WPFTestSuite::WPFSendFilteringStartRequest(
                                    wifi_interface_handle ifaceHandle)
    {
        int length;
        wifi_error ret;
        u8 program[2 * PACKET_FILTER_MAX_LENGTH + 1];
        u8 program_hex[PACKET_FILTER_MAX_LENGTH];
        int i;

        FILE *f_read = fopen("/vendor/etc/wifi/packet_filtering.txt", "r");
        if (f_read) {
            fprintf(stderr, "Parse params from "
                    "/vendor/etc/wifi/packet_filtering.txt\n");

            fgets((char *)program, sizeof(program), f_read);
            length = strlen((const char *)program);

            fclose(f_read);

            if (length % 2 != 0) {
                fprintf(stderr, "%s: Odd num of bytes in input the file: %d\n",
                         __FUNCTION__, length);
                return WIFI_ERROR_INVALID_ARGS;
            }
        } else {
            fprintf(stderr, "Failed to open"
                    "/vendor/etc/wifi/packet_filtering.txt\n");
            return WIFI_ERROR_INVALID_ARGS;
        }

        s8 M, L;
        for (i=0; i+1 < length; i+=2) {
            M = char_to_hex(program[i]);
            if (M < 0) {
                return WIFI_ERROR_INVALID_ARGS;
            }
            L = char_to_hex(program[i+1]);
            if (L < 0) {
                return WIFI_ERROR_INVALID_ARGS;
            }
            program_hex[i/2] = (M << 4) | L;
        }

        fprintf(stderr, "Bytes to be sent \n");
        for (i=0; i < length/2; i++) {
            fprintf(stderr, "%02x ", program_hex[i]);
        }
        fprintf(stderr, "\n");

        fprintf(stderr, "Sending WPF Start Request \n");
        ret = (fn.wifi_set_packet_filter)(ifaceHandle,
                                          &program_hex[0],
                                          length/2);
        fprintf(stderr, "%s\n", ret?"FAILED":"SUCCESS");
        return ret;
    }

    wifi_error WPFTestSuite::WPFSendFilteringStopRequest(
                                            wifi_interface_handle ifaceHandle)
    {
        int length = 0;
        wifi_error ret;
        u8 *program = NULL;

        fprintf(stderr, "Sending WPF Stop Request \n");
        ret = (fn.wifi_set_packet_filter)(ifaceHandle,
                                          program,
                                          length);
        fprintf(stderr, "%s\n", ret?"FAILED":"SUCCESS");
        return ret;
    }

    wifi_error WPFTestSuite::WPFGetFilterFilterInfo(
                                            wifi_interface_handle ifaceHandle)
    {
        u32 version, max_len;
        wifi_error ret;

        fprintf(stderr, "Get Version and max len supported \n");
        ret = (fn.wifi_get_packet_filter_capabilities)(ifaceHandle,
                                               &version,
                                               &max_len);
        fprintf(stderr, "%s\n", ret?"FAILED":"SUCCESS");
        if (!ret)
            fprintf(stderr, "Max supported size for version : %d.%d.%d.%d"
                    " is : %d\n", (version >> 24) & 0xFF,
                    (version >> 16) & 0xFF, (version >> 8) & 0xFF,
                    version & 0xFF, max_len);
        return ret;
    }

    void WPFTestSuite::Usage()
    {
        fprintf(stderr, "hal_proxy_daemon wpf wlan0 SET");
        fprintf(stderr, "hal_proxy_daemon wpf wlan0 RESET");
        fprintf(stderr, "hal_proxy_daemon wpf wlan0 GET_INFO");
    }
}
