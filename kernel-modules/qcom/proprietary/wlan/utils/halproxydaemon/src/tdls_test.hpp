/*
* Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFI_HAL_TDLS_TEST_HPP__
#define __WIFI_HAL_TDLS_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>
namespace TDLS_TEST
{
    class TDLSTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *TDLS_CMD;
        static const char *TDLS_ENABLE;
        static const char *TDLS_GET_STATUS;
        static const char *TDLS_DISABLE;

        TDLSTestSuite(wifi_interface_handle handle);
        /* process the command line args */
        void processCmd(int argc, char **argv);

        /* execute the command line args */
        void executeCmd(int argc, char **argv, int cmdIndex, mac_addr mac);

        /* Helper routine to print usage */
        void TDLSPrintCmdUsage(char **argv, const char *cmd,
                const char *sub_cmd, const struct option long_options[], int size);

        /* process the command line args */
    private:
        wifi_interface_handle wifiHandle_;

        /* Send a TDLS command to Android HAL */
        void TDLSSendEnableRequest(int argc, char* argv[],mac_addr mac);
        void TDLSSendDisableRequest(int argc, char* argv[],mac_addr mac);
        void TDLSSendGetStatusRequest(int argc, char* argv[],mac_addr mac);
        void TDLSSendGetCapsRequest(int argc, char* argv[]);
    };
}
#endif
