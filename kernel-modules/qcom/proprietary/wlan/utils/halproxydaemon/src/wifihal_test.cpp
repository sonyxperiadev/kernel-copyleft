/*
* Copyright (c) 2014, 2016, 2022 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "wifihal_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <inttypes.h>
#include "common.hpp"

extern wifi_hal_fn fn;
namespace WIFIHAL_TEST
{
    /* Keep this up-to-date with Feature enums defined in wifi_hal.h */
    feature features[] = {
        {0x0001, "WIFI_FEATURE_INFRA               "},
        {0x0002, "WIFI_FEATURE_INFRA_5G            "},
        {0x0004, "WIFI_FEATURE_HOTSPOT             "},
        {0x0008, "WIFI_FEATURE_P2P                 "},
        {0x0010, "WIFI_FEATURE_SOFT_AP             "},
        {0x0020, "WIFI_FEATURE_GSCAN               "},
        {0x0040, "WIFI_FEATURE_NAN                 "},
        {0x0080, "WIFI_FEATURE_D2D_RTT             "},
        {0x0100, "WIFI_FEATURE_D2AP_RTT            "},
        {0x0200, "WIFI_FEATURE_BATCH_SCAN          "},
        {0x0400, "WIFI_FEATURE_PNO                 "},
        {0x0800, "WIFI_FEATURE_ADDITIONAL_STA      "},
        {0x1000, "WIFI_FEATURE_TDLS                "},
        {0x2000, "WIFI_FEATURE_TDLS_OFFCHANNEL     "},
        {0x4000, "WIFI_FEATURE_EPR                 "},
        {0x8000, "WIFI_FEATURE_AP_STA              "},
        {0x10000, "WIFI_FEATURE_LINK_LAYER_STATS   "},
        {0x20000, "WIFI_FEATURE_LOGGER             "},
        {0x40000, "WIFI_FEATURE_HAL_EPNO           "},
        {0x80000, "WIFI_FEATURE_RSSI_MONITOR       "},
        {0x100000, "WIFI_FEATURE_MKEEP_ALIVE       "},
        {0x200000, "WIFI_FEATURE_CONFIG_NDO        "},
        {0x400000, "WIFI_FEATURE_TX_TRANSMIT_POWER "}
    };
    /* Number of WIFI_FEATURE_* enums defined above */
    #define WIFI_FEATURE_ENUM_SIZE(x) (sizeof(x)/sizeof(x[0]))

    /* CLI cmd strings */
    const char *WifiHalTestSuite::WIFIHAL_CMD = "wifihal";
    const char *WifiHalTestSuite::WIFIHAL_GET_SUPPORTED_FEATURES =
        "get_supported_features";
    const char *WifiHalTestSuite::WIFIHAL_SET_NO_DFS_FLAG =
        "set_no_dfs_flag";


    /* Constructor */
    WifiHalTestSuite::WifiHalTestSuite(wifi_interface_handle handle)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "WifiHalTestSuite::WifiHalTestSuite: "
            "Created a Wi-Fi HAL Suite.\n");
    }


    /* Process the command line args */
    void WifiHalTestSuite::executeCmd(int cmdIndex,
                                      cmdData data)
    {
        fprintf(stderr, "%s: Enter \n", __func__);
        switch(cmdIndex) {
            case 1:
                wifihalSendGetSupportedFeatures();
                break;
            case 2:
                wifihalSendSetNoDfsFlag(data.no_dfs_flag);
                break;
            case 3:
                wifihalSendGetConcurrencyMatrix(data.set_size_max);
                break;
            case 4:
                wifihalGetIfaces(data.wifiHandle);
                break;
            case 5:
                wifihalSetIfaceEventHandler(data.reqId);
                break;
            case 6:
                wifihalReSetIfaceEventHandler(data.reqId);
                break;
            case 7:
                wifihalSetTxPowerScenario(data.scenario);
                break;
            case 8:
                wifihalReSetTxPowerScenario();
                break;
            case 9:
                wifihalGetUsableChannels(data.wifiHandle, data.band_mask,
                                         data.iface_mask, data.filter_mask,
                                         data.max_size);
                break;
            case 10:
                wifihalSetCoexUnsafeChannels(data.wifiHandle,
                                             data.no_of_channels,
                                             data.unsafe_channs, data.iface_mask);
                break;
            case 11:
                wifihalGetChipCapabilities(data.wifiHandle);
                break;
            default:
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
        }
    }

    void WifiHalTestSuite::wifihalSendGetSupportedFeatures()
    {
        int ret = 0;
        feature_set set = 0;
        size_t i;

        ret = wifi_get_supported_feature_set(wifiHandle_, &set);
        fprintf(stderr, "wifi_get_supported_feature_set : 0x%" PRIx64 " \n", set);

        if (ret)
        {
            fprintf(stderr, "wifi_get_supported_feature_set failed");
            return;
        }

        for (i = 0; i < WIFI_FEATURE_ENUM_SIZE(features); i++)
        {
            if(set & features[i].id)
                fprintf(stderr, "%s: Supported\n", features[i].description);
            else
                fprintf(stderr, "%s: Not Supported\n",
                features[i].description);
        }
    }


    void WifiHalTestSuite::wifihalSendSetNoDfsFlag(u32 no_dfs)
    {
        int ret = 0;
        fprintf(stderr, "%s: Sending No DFS Flag:%d to the wifi HAL. "
            "\n", __func__, no_dfs);

        ret = wifi_set_nodfs_flag(wifiHandle_, no_dfs);
        if (ret)
        {
            fprintf(stderr, "%s: Set No_DFS Flag failed, returned: %d.\n",
                __func__, ret);
            return;
        }
    }

    void WifiHalTestSuite::wifihalSendGetConcurrencyMatrix(int set_size_max)
    {
        int ret = 0;
        int setSize = 0;
        int i = 0;
        size_t j = 0;
        feature_set *concurrencySet;
        feature_set concurrencyRecord = 0;

        concurrencySet =
            (feature_set *) malloc (set_size_max * sizeof(feature_set));

        if (concurrencySet)
        {
            memset(concurrencySet, 0, set_size_max * sizeof(feature_set));
            fprintf(stderr, "WifiHalTestSuite::wifihalSendGetConcurrencyMatrix"
                ": Sending Get Concurrency MAtrix Request. "
                "max_set_size: %d.\n", set_size_max);

            ret = wifi_get_concurrency_matrix(wifiHandle_,
                                              set_size_max,
                                              concurrencySet, &setSize);
            if (ret)
            {
                fprintf(stderr, "wifi_get_concurrency_matrix failed, "
                    "ret code:%d\n", ret);
                goto cleanup;
            }

            fprintf(stderr, "No. of supported concurrency combinations:%d\n",
                setSize);

            for (i = 0; i < setSize; i++)
            {
                fprintf(stderr, "Supported features combination[%d]: ", i+1);
                concurrencyRecord = concurrencySet[i];
                for (j = 0; j < WIFI_FEATURE_ENUM_SIZE(features); j++)
                {
                    if (concurrencyRecord & features[j].id)
                        fprintf(stderr, "%s ", features[j].description);
                }
                fprintf(stderr, ".\n");
            }
        }
    cleanup:
        if (concurrencySet) {
            free(concurrencySet);
            concurrencySet = NULL;
        }
    }

    void WifiHalTestSuite::wifihalGetIfaces(wifi_handle handle)
    {
        int i = 0, num=0;
        wifi_interface_handle *interfaces;

        fprintf(stderr, "Get the interfaces available:\n");
        wifi_get_ifaces(handle, &num, &interfaces);

        interface_info **iface = (interface_info **)interfaces;
        char name[IFNAMSIZ+1];

        fprintf(stderr, "Number of interfaces available: %d\n", num);

        for(i = 0; i< num; i++) {
            if(wifi_get_iface_name((wifi_interface_handle)iface[i],
                        &name[0], IFNAMSIZ) == WIFI_SUCCESS) {
                fprintf(stderr, "Interface %d : %s\n", i, name);
            } else {
                fprintf(stderr, "Failed to get the iface name");
            }
        }
        fprintf(stderr, "########## Done #############\n");
    }

    void onCountryCodeChanged(char code[2])
    {
        fprintf(stderr, "New Country Code : \"%c%c\"\n", code[0], code[1]);
    }

    void WifiHalTestSuite::wifihalSetIfaceEventHandler(wifi_request_id reqId)
    {
        int ret = 0;
        wifi_event_handler eh;
        eh.on_country_code_changed = onCountryCodeChanged;
        fprintf(stderr, "%s: request Id : %d\n", __func__, reqId);

        ret = wifi_set_iface_event_handler(reqId, wifiHandle_, eh);
        if (ret)
        {
            fprintf(stderr, "wifi_set_iface_event_handler failed, "
                    "ret code:%d\n", ret);
        }
        fprintf(stderr, "########## Done #############\n");
    }

    void WifiHalTestSuite::wifihalReSetIfaceEventHandler(wifi_request_id reqId)
    {
        int ret = 0;
        wifi_event_handler eh;
        eh.on_country_code_changed = onCountryCodeChanged;
        fprintf(stderr, "%s: request Id : %d\n", __func__, reqId);

        ret = wifi_reset_iface_event_handler(reqId, wifiHandle_);
        if (ret)
        {
            fprintf(stderr, "wifi_reset_iface_event_handler failed, "
                    "ret code:%d\n", ret);
        }
        fprintf(stderr, "########## Done #############\n");
    }

    void WifiHalTestSuite::wifihalSetTxPowerScenario(wifi_power_scenario scenario)
    {
        int ret = 0;
        fprintf(stderr, "%s: scenario: %d\n", __func__, scenario);

        ret = wifi_select_tx_power_scenario(wifiHandle_, scenario);
        if (ret)
        {
            fprintf(stderr, "wifi_select_tx_power_scenario failed, "
                    "ret code:%d\n", ret);
        }
        fprintf(stderr, "########## Done #############\n");
    }

    void WifiHalTestSuite::wifihalReSetTxPowerScenario()
    {
        int ret = 0;

        ret = wifi_reset_tx_power_scenario(wifiHandle_);
        if (ret)
        {
            fprintf(stderr, "wifi_reset_tx_power_scenario failed, "
                    "ret code:%d\n", ret);
        }
        fprintf(stderr, "########## Done #############\n");
    }

    void WifiHalTestSuite::wifihalSetCoexUnsafeChannels(wifi_handle handle,
                                                        u32 num_channels,
                                                        wifi_coex_unsafe_channel *unsafeChannels,
                                                        u32 restrictions)
    {
        int ret;

        if (!fn.wifi_set_coex_unsafe_channels) {
            fprintf(stderr, "No API registered for "
                    "wifi_set_coex_unsafe_channels\n");
            return;
        }
        ret = fn.wifi_set_coex_unsafe_channels(handle, num_channels,
                                         unsafeChannels, restrictions);
        fprintf(stderr, "%s: set coex unsafe channels with ret:(%d)\n",
                    __func__, ret);
        if (unsafeChannels)
            free(unsafeChannels);
        return;
    }


#ifdef CONFIG_ANDROID_14
    void WifiHalTestSuite::wifihalGetChipCapabilities(wifi_handle handle)
    {
        int ret;
        wifi_chip_capabilities chip_caps;
        memset(&chip_caps, 0, sizeof(chip_caps));

        if (!fn.wifi_get_chip_capabilities) {
            fprintf(stderr, "No API registered for "
                    "wifi_get_chip_capabilities\n");
            return;
        }
        ret = fn.wifi_get_chip_capabilities(handle, &chip_caps);
        fprintf(stderr, "%s: get chip capabilites with ret:(%d)\n",
                    __func__, ret);
        return;
    }
#else
    void WifiHalTestSuite::wifihalGetChipCapabilities(wifi_handle handle)
    {
      UNUSED(handle);
      fprintf(stderr, "Not supported on this android version\n");
    }
#endif

    void WifiHalTestSuite::wifihalGetUsableChannels(wifi_handle handle,
                                                    u32 band_mask,
                                                    u32 iface_mask,
                                                    u32 filter_mask,
                                                    u32 max_size)
    {
        int ret;
        u32 size = 0;
        wifi_usable_channel *channels = NULL, *temp = NULL;

        channels = (wifi_usable_channel *)malloc(max_size *
                                sizeof(wifi_usable_channel));
        if (!channels)
            return;

        if (!fn.wifi_get_usable_channels) {
            fprintf(stderr, "No API registered for wifi_get_usable_channels\n");
            return;
        }
        ret = fn.wifi_get_usable_channels(handle, band_mask, iface_mask,
                                       filter_mask, max_size, &size,
                                       channels);
        if (ret || !size) {
            fprintf(stderr, "%s: Channels list could not be fetched with ret:(%d) size: %d\n",
                    __func__, ret, size);
            free(channels);
            return;
        }

        fprintf(stderr, "Number of usable channels returned :%d.\n", size);
        if (size > max_size) {
            fprintf(stderr, "%s: returned size %d is greater than max size %d\n",
                    __func__, size, max_size);
            size = max_size;
        }

        temp = channels;

        for (int i = 0; i < size; i++, temp++) {
            fprintf(stderr, "[Info %d]: Frequency: %d Bandwidth: %d Iface mask %d\n",
                    (i+1), temp->freq, temp->width, temp->iface_mode_mask);
        }

        free(channels);
    }
}
