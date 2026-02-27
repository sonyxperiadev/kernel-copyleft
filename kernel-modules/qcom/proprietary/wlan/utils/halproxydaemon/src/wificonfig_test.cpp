/*
* Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include<sys/socket.h>
#include "wificonfig_test.hpp"
#include "common.hpp"

extern struct sockaddr_in si;
extern int app_sock, slen;

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);

extern wifi_hal_fn fn;
namespace WIFI_CONFIG_TEST
{
    /* CLI cmd strings */
    const char *WiFiConfigTestSuite::WIFI_CONFIG_CMD = "wificonfig";
    const char *WiFiConfigTestSuite::WIFI_CONFIG_SET_DTIM = "set_dtim";
    const char *WiFiConfigTestSuite::WIFI_CONFIG_SET_MODULATED_DTIM =
        "set_modulated_dtim";
    const char *WiFiConfigTestSuite::WIFI_CONFIG_SET_GUARD_TIME =
        "set_guard_time";
    const char *WiFiConfigTestSuite::WIFI_CONFIG_SET_AVG_FACTOR =
        "set_avg_factor";
    const char *WiFiConfigTestSuite::WIFI_CONFIG_SET_COUNTRY_CODE =
        "set_country_code";
    const char *WiFiConfigTestSuite::WIFI_CONFIG_SET_VOIP_MODE =
        "set_voip_mode";
    /* Constructor */
    WiFiConfigTestSuite::WiFiConfigTestSuite(wifi_interface_handle handle)
        :ifaceHandle(handle)
    {
        fprintf(stderr, "WiFiConfigTestSuite::WiFiConfigTestSuite: Created a "
            "WiFiConfig Test Suite. \n");
    }

    void WiFiConfigTestSuite::setRequestId(int reqId)
    {
        id = reqId;
        fprintf(stderr, "WiFiConfigTestSuite::setRequestId:%d. \n", reqId);
    }

    int WiFiConfigTestSuite::getRequestId()
    {
        return id;
    }

    /* process the command line args */
    wifi_error WiFiConfigTestSuite::processCmd(int argc, char **argv)
    {
        UNUSED(argc);
        if(!ifaceHandle) {
            fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
            fprintf(stderr, "Please restart hal_proxy_daemon with a valid"
                    " initialized interface\n");
            return WIFI_ERROR_NOT_AVAILABLE;
        }
        if(strcasecmp(argv[3], WIFI_CONFIG_SET_DTIM) == 0)
            wifiConfigSendSetDtimRequest(atoi(argv[4]));
        else if (strcasecmp(argv[3], WIFI_CONFIG_SET_MODULATED_DTIM) == 0)
            wifiConfigSendSetModulatedDtimRequest(atoi(argv[4]));
        return WIFI_SUCCESS;
    }

    /* process the command line args */
    void WiFiConfigTestSuite::executeCmd(int cmdId, cmdData data)
    {
        fprintf(stderr, "%s: Enter \n", __func__);

        switch(cmdId) {
            case 1:
                wifiConfigSendSetDtimRequest(data.dtim);
                break;
            case 2:
                wifiConfigSendSetAvgFactorRequest(data.avgFactor);
                break;
            case 3:
                wifiConfigSendSetGuardTimeRequest(data.guardTime);
                break;
            case 4:
                wifiConfigSendSetCountryCodeRequest(data.countryCode);
                break;
            case 5:
                wifiConfigSendSetLatencyModeRequest(data.latencyMode);
                break;
            case 6:
                wifiConfigSendSetThermalModeRequest(data.wifiHandle,
                                                    data.thermalMode,
                                                    data.completionWindow);
                break;
            case 7:
                wifiConfigMultiStaSendSetPrimaryConnectionRequest(data.wifiHandle,
                                                                  data.ifaceHandle);
                break;
            case 8:
                wifiConfigMultiStaSendSetUseCaseRequest(data.wifiHandle,
                                                        data.caseInfo);
                break;
            case 9:
                wifiConfigSendSetModulatedDtimRequest(data.dtim);
                break;
            case 10:
                wifiConfigSendSetVoipMode(ifaceHandle, data.voip_mode);
                break;
            default:
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
                break;
        }
    }

    void WiFiConfigTestSuite::wifiConfigSendSetDtimRequest(int dtim)
    {
        int ret = 0;

        fprintf(stderr, "%s: Sending WiFi Config Set DTIM Request. \n",
            __func__);
        ret = wifi_extended_dtim_config_set(id, ifaceHandle, dtim);
        fprintf(stderr, "%s: wifi_extended_dtim_config_set returned value:%d."
            " \n", __func__, ret);
    }

    void WiFiConfigTestSuite::wifiConfigSendSetModulatedDtimRequest(int dtim)
    {
        int ret = 0;

        if (!fn.wifi_set_dtim_config) {
            fprintf(stderr, "%s: No API registered for wifi_set_dtim_config\n",
                    __func__);
            return;
        }
        fprintf(stderr, "%s: Sending Modulated DTIM multiplier: %d\n",
                __func__, dtim);
        ret = fn.wifi_set_dtim_config(ifaceHandle, dtim);
        fprintf(stderr, "%s: wifi modulated_dtim_config_set returned value:%d."
            " \n", __func__, ret);
    }

    void WiFiConfigTestSuite::wifiConfigSendSetAvgFactorRequest(u32 avg_factor)
    {
        int ret = 0;

        fprintf(stderr, "%s: Sending WiFi Config Set Avg. Factor Request. for"
        " avg_factor:%d\n", __func__, avg_factor);
        ret = wifi_set_beacon_wifi_iface_stats_averaging_factor(id,
                                                               ifaceHandle,
                                                               avg_factor);
        fprintf(stderr, "%s: wifi_set_beacon_wifi_iface_stats_averaging_factor"
            " returned value:%d. \n", __func__, ret);
    }

    void WiFiConfigTestSuite::wifiConfigSendSetGuardTimeRequest(u32 guard_time)
    {
        int ret = 0;

        fprintf(stderr, "%s: Sending WiFi Config Set Guard Time Request "
        "with guard_time:%d\n", __func__, guard_time);
        ret = wifi_set_guard_time(id, ifaceHandle, guard_time);
        fprintf(stderr, "%s: wifi_set_guard_time"
            " returned value:%d. \n", __func__, ret);
    }

    void WiFiConfigTestSuite::wifiConfigSendSetCountryCodeRequest(
                                                            char *country_code)
    {
        int ret = 0;

        fprintf(stderr, "%s: Sending WiFi Config Set Country Code Request for "
        "CC: %s\n", __func__, country_code);
        ret = wifi_set_country_code(ifaceHandle, country_code);
        fprintf(stderr, "%s: wifi_set_country_code"
            " returned value:%d. \n", __func__, ret);
    }

#ifdef WCNSS_QTI_AOSP
    void WiFiConfigTestSuite::wifiConfigSendSetLatencyModeRequest(
												wifi_latency_mode latency_mode)
    {
        int ret = 0;

        fprintf(stderr, "%s: Sending WiFi Config Set Latency Mode Request for "
        "Mode: %u\n", __func__, latency_mode);
        ret = wifi_set_latency_mode(ifaceHandle, latency_mode);
        fprintf(stderr, "%s: wifi_set_latency_mode"
            " returned value:%d. \n", __func__, ret);
    }

#else /* WCNSS_QTI_AOSP */
    void WiFiConfigTestSuite::wifiConfigSendSetLatencyModeRequest(
												wifi_latency_mode latency_mode)
    {
	fprintf(stderr, "%s: Mode:%u, wifi_set_latency_mode is not supported\n",
				__func__, latency_mode);
    }
#endif /* WCNSS_QTI_AOSP */


    void WiFiConfigTestSuite::wifiConfigSendSetThermalModeRequest(
                                      wifi_handle wifiHandle,
                                      wifi_thermal_mode thermalMode,
                                      u32 completionWindow)
    {
        int ret = 0;

        if (!fn.wifi_set_thermal_mitigation_mode) {
            fprintf(stderr, "No API registered for "
                    "wifi_set_thermal_mitigation_mode\n");
	    return;
        }
        fprintf(stderr, "Sending Thermal Mode: %u, completion window = %u\n",
                        thermalMode, completionWindow);
        ret = fn.wifi_set_thermal_mitigation_mode(wifiHandle, thermalMode,
                                                  completionWindow);
        fprintf(stderr, "wifi_set_thermal_mitigation_mode returned value: %d\n", ret);
    }

    void WiFiConfigTestSuite::wifiConfigMultiStaSendSetPrimaryConnectionRequest(wifi_handle wifiHandle,
		    wifi_interface_handle iface)
    {
        int ret = 0;
        char name[IFNAMSIZ+1];

        if(!fn.wifi_multi_sta_set_primary_connection) {
            fprintf(stderr, "No API registered for "
                    "wifi_multi_sta_set_primary_connection\n");
            return;
        }
        if(!iface || wifi_get_iface_name(iface, &name[0], IFNAMSIZ)) {
           fprintf(stderr, "Failed to get the iface name");
           return;
        }

        fprintf(stderr, "%s: Wifi wulti Sta, setting the Primary Connection: %s\n",
                __func__, name);
        ret = fn.wifi_multi_sta_set_primary_connection(wifiHandle, iface);
        fprintf(stderr, "%s: wifi_multi_sta_set_primary_connection"
            " returned value: %d. \n", __func__, ret);
    }

    void WiFiConfigTestSuite::wifiConfigMultiStaSendSetUseCaseRequest(wifi_handle wifiHandle,
			                                  wifi_multi_sta_use_case caseInfo)
    {
        int ret = 0;

        if(!fn.wifi_multi_sta_set_use_case) {
            fprintf(stderr, "No API registered for "
                    "wifi_multi_sta_set_use_case\n");
            return;
        }

        fprintf(stderr, "%s: wifi multi sta setting the use case to: %d\n",
                __func__, caseInfo);
        ret = fn.wifi_multi_sta_set_use_case(wifiHandle, caseInfo);
        fprintf(stderr, "%s: wifi_multi_sta_set_use_case"
            " returned value: %d. \n", __func__, ret);
    }

    void WiFiConfigTestSuite::wifiConfigSendSetVoipMode(
		wifi_interface_handle iface, wifi_voip_mode mode)
    {
        int ret = 0;

        if(!fn.wifi_set_voip_mode) {
            fprintf(stderr, "No API registered for " " wifi_set_voip_mode\n");
            return;
        }

        fprintf(stderr, "%s: Wifi Set Voip mode:%d\n", __func__, mode);
        ret = fn.wifi_set_voip_mode(iface, mode);
        fprintf(stderr, "%s: wifi_set_voip_mode"
            " returned value: %d. \n", __func__, ret);
    }
}
