/*
* Copyright (c) 2016 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*
*/

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "roam_test.hpp"
#include "common.hpp"

namespace ROAM_TEST
{
    /* CLI cmd strings */
    const char *RoamTestSuite::ROAM_CMD = "roam";

    /* Constructor */
    RoamTestSuite::RoamTestSuite(wifi_handle handle)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "ROAMTestSuite \n.");
    }
    void RoamTestSuite::roamCmdUsage()
    {
        fprintf(stderr, "\nUsage: hal_proxy_daemon ROAM/roam iface command"
            " args\n\n");
        fprintf(stderr, "Get roaming Capabilities                  :"
                        "1 iterations interval\n");
        fprintf(stderr, "Ex: hal_proxy_daemon roam wlan0 1 1 1\n\n");
        fprintf(stderr, "Enable/Disable firmware roaming           :"
                        "2 state iteration interval\n");
        fprintf(stderr, "Ex: hal_proxy_daemon roam wlan0 2 0 1 1\n\n");
        fprintf(stderr, "Configure roaming parameters              :"
                        "3 num_bssid <bssid_list> num_ssid <ssid_list> "
                        "iteration iterval\n");
        fprintf(stderr, "Ex: hal_proxy_daemon roam wlan0 3 2 AA:AA:AA:AA:AA:AA "
                        "BB:BB:BB:BB:BB:BB 3 AP1 AP2 AP3 1 1\n\n");
    }

    void RoamTestSuite::roamGetCapabilities()
    {
        int ret = WIFI_SUCCESS;
        wifi_roaming_capabilities capa;
        if (!fn.wifi_get_roaming_capabilities) {
            fprintf(stderr, "No API registered for "
                    "wifi_get_roaming_capabilities\n");
            return;
        }

        fprintf(stderr, "%s: Sending Get roaming capabilities request.\n",
                __FUNCTION__);
        ret = (fn.wifi_get_roaming_capabilities)(ifaceHandle, &capa);
        fprintf(stderr, "Roaming capabilities %s ret =%d\n",
                ret ? "FAILED" : "SUCCESS", ret);
        if (ret == WIFI_SUCCESS) {
            fprintf(stderr,"Max_blacklist_size: %d\nMax_whitelist_size: %d\n",
                    capa.max_blacklist_size, capa.max_whitelist_size);
        }
    }

    void RoamTestSuite::roamEnableFirmwareRoaming(fw_roaming_state_t state)
    {
        int ret = WIFI_SUCCESS;

        if (!fn.wifi_enable_firmware_roaming) {
            fprintf(stderr, "No API registered for "
                    "wifi_enable_firmware_roaming\n");
            return;
        }
        if (state != ROAMING_DISABLE && state != ROAMING_ENABLE) {
            fprintf(stderr, "Invalid state provided, Valid state are %d, %d\n",
                    ROAMING_DISABLE, ROAMING_ENABLE);
            return;
        }

        fprintf(stderr, "%s: Sending Enable Firmware roaming request.\n",
                __FUNCTION__);
        ret = (fn.wifi_enable_firmware_roaming)(ifaceHandle, state);
        fprintf(stderr, "Enable Firmware Roaming state %s ret =%d\n",
                ret ? "FAILED" : "SUCCESS", ret);
    }

    void RoamTestSuite::roamSetConfigParams(wifi_roaming_config *config_params)
    {
        int ret = WIFI_SUCCESS;

        if (!fn.wifi_configure_roaming) {
            fprintf(stderr, "No API registered for "
                    "wifi_configure_roaming\n");
            return;
        }

        fprintf(stderr, "%s: Sending configure roaming parameters request.\n",
                __FUNCTION__);
        ret = (fn.wifi_configure_roaming)(ifaceHandle, config_params);
        fprintf(stderr, "configure roaming parameters %s ret =%d\n",
                ret ? "FAILED" : "SUCCESS", ret);
    }

    int RoamTestSuite::processRoamCmd(int argc, char *argv[])
    {
        fw_roaming_state_t state;
        wifi_roaming_config config_params;
        int i;
        char addr[20];
        int cmdId, cmdIndex, iter, sleep_time, ch;
        char ssid[MAX_SSID_LENGTH + 1];
        u32 len;

        if(argc < ARGV_IFACE_INDEX + 1)
        {
            fprintf(stderr, "insufficient ROAM args\n");
            fprintf(stderr, "Usage : hal_proxy_daemon roam interface_name\n");
            return WIFI_ERROR_INVALID_ARGS;
        }

        ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[ARGV_IFACE_INDEX]);
        if(!ifaceHandle)
        {
            fprintf(stderr, "Interface %s is not up, exiting.\n",
                    argv[ARGV_IFACE_INDEX]);
            fprintf(stderr, "Please restart hal_proxy_daemon with a valid"
                " initialized interface\n");
            return WIFI_ERROR_INVALID_ARGS;
        }

        if (argc > 3) {
            cmdIndex = ARGV_CMDID_INDEX;

            if (!strcasecmp(argv[cmdIndex], "-h") ||
                !strcasecmp(argv[cmdIndex], "--help")) {
                roamCmdUsage();
                return WIFI_ERROR_NONE;
            }
            while(cmdIndex < argc) {
                cmdId = atoi(argv[cmdIndex++]);
                switch(cmdId) {
                    case 1:
                        /*hal_proxy_daemon roam wlan0 1 iter inter*/
                        if (argc <= (cmdIndex + 1)) {
                            fprintf(stderr, "Insufficient Args for cmdId :%d\n",
                                    cmdId);
                            return WIFI_ERROR_INVALID_ARGS;
                        }
                        iter = atoi(argv[cmdIndex++]);
                        if (validate_iterations_count(iter))
                            return WIFI_ERROR_INVALID_ARGS;
                        sleep_time = atoi(argv[cmdIndex++]);
                        if (validate_sleep_duration(sleep_time))
                            return WIFI_ERROR_INVALID_ARGS;
                        for (i = 0; i < iter; i++) {
                            roamGetCapabilities();
                            sleep(sleep_time);
                        }
                        break;
                    case 2:
                        /* hal_proxy_daemon roam wlan0 2 state iter inter */
                        if (argc <= (cmdIndex + 2)) {
                            fprintf(stderr, "Insufficient Args for cmdId :%d\n",
                                    cmdId);
                            return WIFI_ERROR_INVALID_ARGS;
                        }
                        state = (fw_roaming_state_t)atoi(argv[cmdIndex++]);
                        iter = atoi(argv[cmdIndex++]);
                        if (validate_iterations_count(iter))
                            return WIFI_ERROR_INVALID_ARGS;
                        sleep_time = atoi(argv[cmdIndex++]);
                        if (validate_sleep_duration(sleep_time))
                            return WIFI_ERROR_INVALID_ARGS;
                        for (i = 0; i < iter; i++) {
                            roamEnableFirmwareRoaming(state);
                            sleep(sleep_time);
                        }
                        break;
                    case 3:
                        /* hal_proxy_daemon roam wlan0 3 bl_no <blList> wl_no <wlList> iter inter */
                        if (argc <= (cmdIndex + 3)) {
                            fprintf(stderr, "Insufficient Args for cmdId :%d\n",
                                    cmdId);
                            return WIFI_ERROR_INVALID_ARGS;
                        }
                        config_params.num_blacklist_bssid =
                                                           atoi(argv[cmdIndex]);
                        if (config_params.num_blacklist_bssid >
                            MAX_BLACKLIST_BSSID) {
                            fprintf(stderr, "Invalid Number of blacklist bssid"
                                    " provided\n");
                            return WIFI_ERROR_INVALID_ARGS;
                        } else if (argc <= (cmdIndex +
                                           (int)config_params.num_blacklist_bssid
                                            + 3)) {
                            fprintf(stderr, "Insufficient Args for cmdId :%d\n",
                                    cmdId);
                            return WIFI_ERROR_INVALID_ARGS;
                        }
                        for (i = 0; i < (int)config_params.num_blacklist_bssid;
                             i++) {
                            if (mac_addr_aton(config_params.blacklist_bssid[i],
                                argv[++cmdIndex]))
                                return WIFI_ERROR_INVALID_ARGS;
                        }

                        config_params.num_whitelist_ssid =
                                                    atoi(argv[++cmdIndex]);
                        if (config_params.num_whitelist_ssid >
                            MAX_WHITELIST_SSID) {
                            fprintf(stderr, "Invalid Number of whitelist ssid "
                                    "provided\n");
                            return WIFI_ERROR_INVALID_ARGS;
                        } else if(argc <= (cmdIndex +
                                           (int)config_params.num_whitelist_ssid
                                           + 2)) {
                            fprintf(stderr, "Insufficient Args for cmdId :%d\n",
                                    cmdId);
                            return WIFI_ERROR_INVALID_ARGS;
                        }
                        for (i = 0; i < (int)config_params.num_whitelist_ssid;
                             i++) {
                            fprintf(stderr, "Enter ssid: ");
                            sscanf(argv[++cmdIndex], "%32s", ssid);
                            fprintf(stderr, "ssid is : %s\n", ssid);
                            len = strlen(ssid);
                            memcpy(config_params.whitelist_ssid[i].ssid_str,
                                   ssid, len);
                            config_params.whitelist_ssid[i].length = len;
                        }

                        iter = atoi(argv[++cmdIndex]);
                        if (validate_iterations_count(iter))
                            return WIFI_ERROR_INVALID_ARGS;
                        sleep_time = atoi(argv[++cmdIndex]);
                        if (validate_sleep_duration(sleep_time))
                            return WIFI_ERROR_INVALID_ARGS;
                        for (i = 0; i < iter; i++) {
                            roamSetConfigParams(&config_params);
                            sleep(sleep_time);
                        }
                        cmdIndex++;
                        break;
                    default:
                        fprintf(stderr, "Unknown Command : %d\n", cmdId);
                        return WIFI_ERROR_NOT_SUPPORTED;
                }
            }
        } else {
            fprintf(stderr, "**********************\n");
            fprintf(stderr, "Step 2: Enter ID for ROAM Cmd, as follows:\n");
            fprintf(stderr, "    Type 1 for Get roaming capabilities\n");
            fprintf(stderr, "    Type 2 for Enable/Disable firmware roaming\n");
            fprintf(stderr, "    Type 3 for Configure roaming parameter\n");
            fprintf(stderr, "    Type 1000 to exit.\n");
            fprintf(stderr, "**********************\n");
            while (1) {
                fprintf(stderr, "Step 2: Enter ROAM Cmd ID:\n");
                read_int(&cmdId);
                if (cmdId == 1000)
                    break;
                switch(cmdId) {
                    case 1:
                        roamGetCapabilities();
                        break;
                    case 2:
                        int temp;
                        fprintf(stderr, "Enter Firmware roam state:\n");
                        read_int(&temp);
                        state = (fw_roaming_state_t)temp;
                        roamEnableFirmwareRoaming(state);
                        break;
                    case 3:
                        fprintf(stderr, "Enter Number of blacklist bssid\n");
                        read_uint(&(config_params.num_blacklist_bssid));
                        if (config_params.num_blacklist_bssid >
                            MAX_BLACKLIST_BSSID) {
                            fprintf(stderr, "Invalid Number of blacklist bssid"
                                    " provided\n");
                            return WIFI_ERROR_INVALID_ARGS;
                        }

                        i = 0;
                        while (i < (int)config_params.num_blacklist_bssid) {
                            fprintf(stderr, "Enter bssid: ");
                            read_string(addr, sizeof(addr));
                            if (mac_addr_aton(config_params.blacklist_bssid[i],
                                addr))
                                continue;
                            i++;
                        }

                        fprintf(stderr, "Enter Number of whitelist ssid\n");
                        read_uint(&(config_params.num_whitelist_ssid));
                        if (config_params.num_whitelist_ssid >
                            MAX_WHITELIST_SSID) {
                            fprintf(stderr, "Invalid Number of whitelist ssid "
                                    "provided\n");
                            return WIFI_ERROR_INVALID_ARGS;
                        }

                        for (i = 0; i < (int)config_params.num_whitelist_ssid;
                             i++) {
                            fprintf(stderr, "Enter ssid: ");
                            read_string(ssid, sizeof(ssid));
                            /* clear input buffer */
                            while ((ch=getchar()) != EOF && ch != '\n')
                                ;

                            fprintf(stderr, "ssid is : %s\n", ssid);
                            len = strlen(ssid);
                            memcpy(config_params.whitelist_ssid[i].ssid_str,
                                   ssid, len);
                            config_params.whitelist_ssid[i].length = len;
                        }
                        roamSetConfigParams(&config_params);
                        break;
                    default:
                        fprintf(stderr, "Unknown Command.\n");
                }
            }
        }
        return WIFI_SUCCESS;
    }

}
