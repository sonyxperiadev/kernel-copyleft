/*
* Copyright (c) 2014-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFI_HAL_GSCAN_TEST_HPP__
#define __WIFI_HAL_GSCAN_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>
#include "gscan.h"

namespace GSCAN_TEST
{
    class GScanTestSuite
    {
    public:
        /* CLI cmd strings */
        static const char *GSCAN_CMD;
        static const char *GSCAN_START;
        static const char *GSCAN_STOP;
        static const char *GSCAN_GET_VALID_CHANNELS;
        static const char *GSCAN_GET_CAPABILITIES;
        static const char *GSCAN_SET_BSSID_HOTLIST;
        static const char *GSCAN_RESET_BSSID_HOTLIST;
        static const char *GSCAN_SET_SIGNIFICANT_CHANGE;
        static const char *GSCAN_RESET_SIGNIFICANT_CHANGE;
        static const char *GSCAN_GET_CACHED_RESULTS;
        static const char *GSCAN_SET_SSID_HOTLIST;
        static const char *GSCAN_RESET_SSID_HOTLIST;
        static const char *GSCAN_SET_EPNO_LIST;
        static const char *GSCAN_RESET_EPNO_LIST;

        /* Default service name */
        static const char *DEFAULT_SVC;

        /* Default service name */
        static const char *DEFAULT_SVC_INFO;

        GScanTestSuite(wifi_handle handle, wifi_request_id request_id);

        /* process the command line args */
        wifi_error processCmd(int argc, char **argv);

        /* execute the command line args */
        void executeCmd(int argc, char **argv, int cmdIndex, int max,
                            u32 flush, int band, oui scan_oui);

        void setRequestId(int reqId);
        int getRequestId();

    private:
        wifi_handle wifiHandle_;
        wifi_request_id id;
        wifi_interface_handle ifaceHandle;

        /* Send a GscanSCAN command to Android HAL */
        void gscanSendStartRequest(int argc, char **argv);
        void gscanSendStopRequest(int argc, char **argv);
        void gscanSendGetCapabilitiesRequest(int argc, char **argv);
        void gscanSendGetValidChannelsRequest(int argc, char **argv, int max,
                                                       int band);
        void gscanSendSetBssidHotlistRequest(int argc, char **argv);
        void gscanSendResetBssidHotlistRequest(int argc, char **argv);
        void gscanSendSetSignificantChangeRequest(int argc, char **argv);
        void gscanSendResetSignificantChangeRequest(int argc, char **argv);
        void gscanSendGetCachedResultsRequest(int argc, char **argv, int max,
                                                        int flush);
        int gscanParseHex(unsigned char c);
        int gscanParseMacAddress(const char* arg, u8* addr);
        void gscanPrintCmdUsage(
            char **argv,
            const char *cmd,
            const char *sub_cmd,
            const struct option long_options[],
            int size);
        void gscanSendSetMacOui(int argc, char **argv, oui scan_oui);
        void gscanSendSetSsidHotlistRequest(int argc, char **argv);
        void gscanSendResetSsidHotlistRequest(int argc, char **argv);
        void gscanSendSetEpnoListRequest(int argc, char **argv);
        void gscanSendReSetEpnoListRequest(int argc, char **argv);
        void gscanSendSetPasspointListRequest();
        void gscanSendResetPasspointListRequest();
    };
}

typedef enum {
    GSCAN_CMD_START = 1,
    GSCAN_CMD_STOP,
    GSCAN_CMD_GET_VALID_CHANNELS,
    GSCAN_CMD_GET_CAPABILITIES,
    GSCAN_CMD_GET_CACHED_RESULT,

    GSCAN_CMD_SET_BSSID_HOTLIST,  /*6*/
    GSCAN_CMD_RESET_BSSID_HOTLIST,
    GSCAN_CMD_SET_SIGNIFICANT_CHANGE,
    GSCAN_CMD_RESET_SIGNIFICANT_CHANGE,
    GSCAN_CMD_SET_MAC_OUI,

    GSCAN_CMD_SET_EPNO_LIST,    /*11*/
    GSCAN_CMD_SET_PASSPOINT_LIST,
    GSCAN_CMD_RESET_PASSPOINT_LIST,
    GSCAN_CMD_BSSID_BLACKLIST,
    GSCAN_CMD_RESET_EPNO_LIST,
}GSCAN_CMDID;

#endif
