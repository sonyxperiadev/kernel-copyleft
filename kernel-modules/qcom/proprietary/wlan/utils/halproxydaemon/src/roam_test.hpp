/*
* Copyright (c) 2016 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*
*/

#ifndef __ROAM_TEST_HPP
#define __ROAM_TEST_HPP

#include "wifi_hal.h"
#include "roam.h"

namespace ROAM_TEST
{
    class RoamTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *ROAM_CMD;

        RoamTestSuite(wifi_handle handle);

        /* process the command line args */
        int processRoamCmd(int argc, char *argv[]);

        void roamCmdUsage();

    private:
        wifi_handle wifiHandle_;
        wifi_interface_handle ifaceHandle;

        void roamGetCapabilities();
        void roamEnableFirmwareRoaming(fw_roaming_state_t state);
        void roamSetConfigParams(wifi_roaming_config *config_params);
    };
}

#endif
