/*
* Copyright (c) 2014, 2022 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFIHAL_TEST_HPP__
#define __WIFIHAL_TEST_HPP__

#include "wifi_hal.h"
#include <getopt.h>
#include <netlink/genl/family.h>

namespace WIFIHAL_TEST
{
    typedef struct{
        u32 id;
        const char *description;
    } feature;

    typedef struct{
        u32 no_dfs_flag;
        int set_size_max;
        wifi_request_id reqId;
        wifi_handle wifiHandle;
        wifi_power_scenario scenario;
	u32 band_mask;
	u32 iface_mask;
	u32 filter_mask;
	u32 max_size;
        u32 no_of_channels;
        wifi_coex_unsafe_channel *unsafe_channs;
    } cmdData;

    typedef struct {
        wifi_handle handle;
        char name[IFNAMSIZ+1];
        int  id;
    } interface_info;

    class WifiHalTestSuite
    {
    public:
        /* CLI cmd strings */
        static const char *WIFIHAL_CMD;
        static const char *WIFIHAL_GET_SUPPORTED_FEATURES;
        static const char *WIFIHAL_SET_NO_DFS_FLAG;

        WifiHalTestSuite(wifi_interface_handle handle);

        /* Execute the command line args */
        void executeCmd(int cmdIndex, cmdData data);

    private:
        wifi_interface_handle wifiHandle_;

        /* Send the command to Android HAL */
        void wifihalSendGetSupportedFeatures();
        void wifihalSendSetNoDfsFlag(u32 no_dfs);
        void wifihalSendGetConcurrencyMatrix(int set_size_max);
        void wifihalGetIfaces(wifi_handle handle);
        void wifihalSetIfaceEventHandler(wifi_request_id reqId);
        void wifihalReSetIfaceEventHandler(wifi_request_id reqId);
        void wifihalSetTxPowerScenario(wifi_power_scenario scenario);
        void wifihalReSetTxPowerScenario();
        void wifihalGetUsableChannels(wifi_handle handle, u32 band_mask,
                 u32 iface_mode_mask, u32 filter_mask,
                 u32 max_size);
        void wifihalSetCoexUnsafeChannels(wifi_handle handle, u32 num_channels,
                                          wifi_coex_unsafe_channel *unsafeChannels,
                                          u32 restrictions);
        void wifihalGetChipCapabilities(wifi_handle handle);
    };
}

#endif
