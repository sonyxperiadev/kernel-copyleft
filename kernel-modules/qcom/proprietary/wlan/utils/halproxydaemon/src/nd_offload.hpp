/*
* Copyright (c) 2016 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*
*/

#ifndef __ND_OFFLOAD_HPP
#define __ND_OFFLOAD_HPP

#include "wifi_hal.h"

#define NDO_CMD_ENABLE_OFFLOAD 1
#define NDO_CMD_DISABLE_OFFLOAD 2
#define ENABLE 1
#define DISABLE 0

namespace NDO_TEST
{
    class NDOTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *NDO_CMD;

        NDOTestSuite(wifi_handle handle);

        /* execute the command line args */
        wifi_error executeCmd(int argc, char **argv);

    private:
        wifi_handle wifiHandle_;
        wifi_interface_handle ifaceHandle;

        /* Send a nd offload command to Android HAL */
        wifi_error NDOConfigureNdOffload(
                        wifi_interface_handle ifaceHandle, u8 enable);
        void Usage();
    };
}
#endif
