/*
* Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFI_HAL_WIFI_LOGGER_TEST_HPP__
#define __WIFI_HAL_WIFI_LOGGER_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>

namespace WIFI_LOGGER_TEST
{
    typedef struct{
        int versionType;
        u32 ringId;
        u32 verboseLevel;
        u32 maxInterval;
        u32 minDataSize;
        u32 mode;
    } cmdData;

    class WifiLoggerTestSuite {
    public:
        /* CLI cmd strings */
        static const char *WIFI_LOGGER_CMD;
        static const char *WIFI_LOGGER_GET_WIFI_INFO;
        static const char *WIFI_LOGGER_START_LOGGING;
        static const char *WIFI_LOGGER_GET_RING_BUFFERS_STATUS;
        static const char *WIFI_LOGGER_GET_SUPPORTED_FEATURE_SET;
        static const char *WIFI_LOGGER_GET_RING_DATA;
        static const char *WIFI_LOGGER_GET_MEMORY_DUMP;

        /* Default service name */
        static const char *DEFAULT_SVC;

        WifiLoggerTestSuite(wifi_handle handle, wifi_request_id request_id);

        void setRequestId(int reqId);
        int getRequestId();

        /* execute the menu driven commands */
        void  executeCmd(int argc,
                         char **argv,
                         int cmdIndex,
                         cmdData data);

        /* process the command line args */
        void  processCmd(int argc,
                        char **argv,
                        cmdData data);

    private:
        wifi_handle wifiHandle_;
        wifi_interface_handle ifaceHandle;
        wifi_request_id id;

        /* Send a WifiLogger command to Android HAL */
        void wifiLoggerSendRangeRequest(int argc, char **argv);
        void wifiLoggerPrintCmdUsage(
            char **argv,
            const char *cmd,
            const char *sub_cmd,
            const struct option long_options[],
            int size);
        void  wifiLoggerGetWifiInfo(int argc, char **argv, int version_type);
        void  wifiLoggerStartLogging(int argc, char **argv, cmdData data);
        void  wifiLoggerGetRingBufferStatus(int argc, char **argv);
        void  wifiLoggerGetSupportedFeatureSet(int argc, char **argv);
        void  wifiLoggerGetRingData(int argc, char **argv, int ring_id);
        void  wifiLoggerGetMemoryDump(int argc, char **argv);
        void  wifiLoggerGetDriverMemoryDump();
        void  wifiLoggerGetWakeReasonStats();
        void wifiLoggerStartPktFateStats();
        void wifiLoggerGetTxPktFateStats();
        void wifiLoggerGetRxPktFateStats();
    };
}
#endif
