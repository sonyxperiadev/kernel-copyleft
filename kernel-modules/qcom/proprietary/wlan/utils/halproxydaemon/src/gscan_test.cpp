/*
* Copyright (c) 2014-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "gscan_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include<sys/socket.h>
#include "common.hpp"

extern struct sockaddr_in si;
extern int app_sock, slen;

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
/* Max number of cached results to be returned to caller */
#define GSCAN_MAX_CACHED_RESULTS    100

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);

static void gscan_on_hotlist_ap_found(wifi_request_id req_id,
        unsigned num_results, wifi_scan_result *results)
{
    unsigned int i;
    wifi_scan_result *result = results;
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n",
        __func__, req_id, num_results);

    for(i = 0; i < num_results && result; i++)
    {
        fprintf(stderr, "Hotlist AP Found: %d\n", i+1);
        fprintf(stderr, "ts  %" PRId64 "\n", result->ts);
        fprintf(stderr, "SSID  %s \n", result->ssid) ;
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                result->bssid[0], result->bssid[1], result->bssid[2],
                result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "channel %u \n", result->channel);
        fprintf(stderr, "rssi  %d \n", (signed char) result->rssi);
        fprintf(stderr, "rtt  %" PRId64 "\n", result->rtt);
        fprintf(stderr, "rtt_sd  %" PRId64 "\n", result->rtt_sd);
        fprintf(stderr, "beacon period  %d \n", result->beacon_period);
        fprintf(stderr, "capability  %d \n", result->capability);
        fprintf(stderr, "IE length  %d\n", result->ie_length);
        if(result->ie_length > 0)
        {
            fprintf(stderr, "IE Data : ");
            for(unsigned int j=0; j < result->ie_length; j++)
                fprintf(stderr, "%2x ", result->ie_data[j]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "\n");
        result = (wifi_scan_result *)((u8 *)result + sizeof(wifi_scan_result)
                     +  result->ie_length);
    }
}

static void gscan_on_hotlist_ap_lost(wifi_request_id req_id,
        unsigned num_results, wifi_scan_result *results)
{
    unsigned int i;
    wifi_scan_result *result = results;
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n",
        __func__, req_id, num_results);

    for(i = 0; i < num_results && result; i++)
    {
        fprintf(stderr, "Hotlist AP Lost: %d\n", i+1);
        fprintf(stderr, "ts  %" PRId64 "\n", result->ts);
        fprintf(stderr, "SSID  %s \n", result->ssid) ;
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                result->bssid[0], result->bssid[1], result->bssid[2],
                result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "channel %u \n", result->channel);
        fprintf(stderr, "rssi  %d \n", (signed char) result->rssi);
        fprintf(stderr, "rtt  %" PRId64 "\n", result->rtt);
        fprintf(stderr, "rtt_sd  %" PRId64 "\n", result->rtt_sd);
        fprintf(stderr, "beacon period  %d \n", result->beacon_period);
        fprintf(stderr, "capability  %d \n", result->capability);
        fprintf(stderr, "IE length  %d\n", result->ie_length);
        if(result->ie_length > 0)
        {
            fprintf(stderr, "IE Data : ");
            for(unsigned int j=0; j < result->ie_length; j++)
                fprintf(stderr, "%2x ", result->ie_data[j]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "\n");
        result = (wifi_scan_result *)((u8 *)result + sizeof(wifi_scan_result)
                     +  result->ie_length);
    }
}

static void gscan_on_significant_change(wifi_request_id req_id,
            unsigned num_results,
            wifi_significant_change_result **results)
{
    fprintf(stderr, "gscan_on_significant_change: request_id:%d, "
        "num_results:%d\n", req_id, num_results);
    wifi_significant_change_result *result = NULL;
    if (results)
        result = results[0];
    for(u32 i = 0; i < num_results && result; i++, result = results[i])
    {
        fprintf(stderr, "AP : %d\n", i+1);
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                result->bssid[0], result->bssid[1], result->bssid[2],
                result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "Channel %u \n", result->channel);
        fprintf(stderr, "Num_rssi : %d \n", result->num_rssi);
        fprintf(stderr, "Rssi list : ");
        for(int j=0; j < result->num_rssi; j++)
            fprintf(stderr, "%d ", (signed char) result->rssi[j]);
        fprintf(stderr, "\n\n");
#if 0
        result = (wifi_significant_change_result *)
        ((u8*)results + sizeof(wifi_significant_change_result) +
        result->num_rssi*sizeof(wifi_rssi)) ;
        */
#endif
    }
}

static void gscan_on_epno_network_found(wifi_request_id req_id,
        unsigned num_results, wifi_scan_result *results)
{
    unsigned int i;
    wifi_scan_result *result = results;
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n",
        __func__, req_id, num_results);

    for(i = 0; i < num_results && result; i++)
    {
        fprintf(stderr, "ePNO Network Found: %d\n", i+1);
        fprintf(stderr, "ts  %" PRId64 "\n", result->ts);
        fprintf(stderr, "SSID  %s \n", result->ssid) ;
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                result->bssid[0], result->bssid[1], result->bssid[2],
                result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "channel %u \n", result->channel);
        fprintf(stderr, "rssi  %d \n", (signed char) result->rssi);
        fprintf(stderr, "rtt  %" PRId64 "\n", result->rtt);
        fprintf(stderr, "rtt_sd  %" PRId64 "\n", result->rtt_sd);
        fprintf(stderr, "beacon period  %d \n", result->beacon_period);
        fprintf(stderr, "capability  %d \n", result->capability);
        fprintf(stderr, "IE length  %d\n", result->ie_length);
        if(result->ie_length > 0)
        {
            fprintf(stderr, "IE Data : ");
            for(unsigned int j=0; j < result->ie_length; j++)
                fprintf(stderr, "%2x ", result->ie_data[j]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "\n");
        result = (wifi_scan_result *)((u8 *)result + sizeof(wifi_scan_result)
                     +  result->ie_length);
    }
}

static void gscan_on_passpoint_network_found(wifi_request_id id,
                                             int net_id,
                                             wifi_scan_result *results,
                                             int anqp_len,
                                             byte *anqp)
{
    wifi_scan_result *result = results;
    fprintf(stderr, "%s: request_id:%d, net_id:%d\n\n",
        __func__, id, net_id);

    fprintf(stderr, "Passpoint Network Match: \n");
    fprintf(stderr, "ts  %" PRId64 "\n", result->ts);
    fprintf(stderr, "SSID  %s \n", result->ssid) ;
    fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
            result->bssid[0], result->bssid[1],
            result->bssid[2], result->bssid[3],
            result->bssid[4], result->bssid[5]);
    fprintf(stderr, "channel %u \n", result->channel);
    fprintf(stderr, "rssi  %d \n", (signed char) result->rssi);
    fprintf(stderr, "rtt  %" PRId64 "\n", result->rtt);
    fprintf(stderr, "rtt_sd  %" PRId64 "\n", result->rtt_sd);
    fprintf(stderr, "beacon period  %d \n", result->beacon_period);
    fprintf(stderr, "capability  %d \n", result->capability);
    fprintf(stderr, "IE length  %d\n", result->ie_length);
    if(result->ie_length > 0 &&
       result->ie_data[0])
    {
        fprintf(stderr, "IE Data : ");
        for(unsigned int j = 0; j < result->ie_length; j++)
            fprintf(stderr, "%2x ", result->ie_data[j]);
            fprintf(stderr, "\n\n");
    }
    fprintf(stderr, "ANQP length    %d\n", anqp_len);
    if(anqp_len > 0 && anqp)
    {
        fprintf(stderr, "ANQP Data : ");
        for(int j = 0; j < anqp_len; j++)
            fprintf(stderr, "%2x ", anqp[j]);
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
}

/* reported when each probe response is received, if report_events
 * enabled in wifi_scan_cmd_params */
static void gscan_on_full_scan_result(wifi_request_id id,
                                      wifi_scan_result *result,
                                      unsigned buckets_scanned)
{
    fprintf(stderr, "Full Scan Result: request_id:%d", id);
    if (result) {
        fprintf(stderr, "ts  %" PRId64, result->ts);
        fprintf(stderr, "SSID  %s ", result->ssid) ;
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x ",
                 result->bssid[0], result->bssid[1], result->bssid[2],
                 result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "channel %u ", result->channel);
        fprintf(stderr, "rssi  %d ", (signed char) result->rssi);
        fprintf(stderr, "rtt  %" PRId64, result->rtt);
        fprintf(stderr, "rtt_sd  %" PRId64, result->rtt_sd);
        fprintf(stderr, "beacon period  %d ", result->beacon_period);
        fprintf(stderr, "capability  %d ", result->capability);
        fprintf(stderr, "IE length  %d\n", result->ie_length);
        fprintf(stderr, "No.of buckets scanned  %d\n", buckets_scanned);
    }
}

/* optional event - indicates progress of scanning statemachine */
static void gscan_on_scan_event(wifi_request_id id, wifi_scan_event event)
{
    fprintf(stderr, "%s: request id:%d, status:%d\n",
            __func__, id, event);
#if 0
    char *message = "HI!! Trying to send message to newly created thread";
    if(sendto(app_sock, message, strlen(message) , 0 , (struct sockaddr *) &si, slen)==-1)
    {
        printf("Failed to send msg\n");
    }
#endif
}

#if 0
static void gscan_on_hotlist_ssid_found(wifi_request_id req_id,
        unsigned num_results, wifi_scan_result *results)
{
    unsigned int i;
    wifi_scan_result *result = results;
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n",
        __func__, req_id, num_results);

    for(i = 0; i < num_results && result; i++)
    {
        fprintf(stderr, "Hotlist SSID Found: %d\n", i+1);
        fprintf(stderr, "ts  %lu \n", result->ts);
        fprintf(stderr, "SSID  %s \n", result->ssid) ;
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                result->bssid[0], result->bssid[1], result->bssid[2],
                result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "channel %u \n", result->channel);
        fprintf(stderr, "rssi  %d \n", (signed char) result->rssi);
        fprintf(stderr, "rtt  %lu \n", result->rtt);
        fprintf(stderr, "rtt_sd  %lu \n", result->rtt_sd);
        fprintf(stderr, "beacon period  %d \n", result->beacon_period);
        fprintf(stderr, "capability  %d \n", result->capability);
        fprintf(stderr, "IE length  %d\n", result->ie_length);
        if(result->ie_length > 0)
        {
            fprintf(stderr, "IE Data : ");
            for(unsigned int j=0; j < result->ie_length; j++)
                fprintf(stderr, "%2x ", result->ie_data[j]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "\n");
        result = (wifi_scan_result *)((u8 *)result + sizeof(wifi_scan_result)
                     +  result->ie_length);
    }
}

static void gscan_on_hotlist_ssid_lost(wifi_request_id req_id,
        unsigned num_results, wifi_scan_result *results)
{
    unsigned int i;
    wifi_scan_result *result = results;
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n",
        __func__, req_id, num_results);

    for(i = 0; i < num_results && result; i++)
    {
        fprintf(stderr, "Hotlist SSID Lost: %d\n", i+1);
        fprintf(stderr, "ts  %lu \n", result->ts);
        fprintf(stderr, "SSID  %s \n", result->ssid) ;
        fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                result->bssid[0], result->bssid[1], result->bssid[2],
                result->bssid[3], result->bssid[4], result->bssid[5]);
        fprintf(stderr, "channel %u \n", result->channel);
        fprintf(stderr, "rssi  %d \n", (signed char) result->rssi);
        fprintf(stderr, "rtt  %lu \n", result->rtt);
        fprintf(stderr, "rtt_sd  %lu \n", result->rtt_sd);
        fprintf(stderr, "beacon period  %d \n", result->beacon_period);
        fprintf(stderr, "capability  %d \n", result->capability);
        fprintf(stderr, "IE length  %d\n", result->ie_length);
        if(result->ie_length > 0)
        {
            fprintf(stderr, "IE Data : ");
            for(unsigned int j=0; j < result->ie_length; j++)
                fprintf(stderr, "%2x ", result->ie_data[j]);
            fprintf(stderr, "\n");
        }
        fprintf(stderr, "\n");
        result = (wifi_scan_result *)((u8 *)result + sizeof(wifi_scan_result)
                     +  result->ie_length);
    }
}
#endif

namespace GSCAN_TEST
{
    /* CLI cmd strings */
    const char *GScanTestSuite::GSCAN_CMD = "gscan";
    const char *GScanTestSuite::GSCAN_START = "start";
    const char *GScanTestSuite::GSCAN_STOP = "stop";
    const char *GScanTestSuite::GSCAN_GET_VALID_CHANNELS =
        "get_valid_channels";
    const char *GScanTestSuite::GSCAN_GET_CAPABILITIES = "get_capabilities";
    const char *GScanTestSuite::GSCAN_SET_BSSID_HOTLIST = "set_bssid_hotlist";
    const char *GScanTestSuite::GSCAN_RESET_BSSID_HOTLIST =
        "reset_bssid_hotlist";
    const char *GScanTestSuite::GSCAN_SET_SIGNIFICANT_CHANGE =
        "set_significant_change";
    const char *GScanTestSuite::GSCAN_RESET_SIGNIFICANT_CHANGE =
        "reset_significant_change";
    const char *GScanTestSuite::GSCAN_GET_CACHED_RESULTS =
        "get_cached_results";
    const char *GScanTestSuite::GSCAN_SET_SSID_HOTLIST = "set_ssid_hotlist";
    const char *GScanTestSuite::GSCAN_RESET_SSID_HOTLIST =
        "reset_ssid_hotlist";
    const char *GScanTestSuite::GSCAN_SET_EPNO_LIST = "set_epno_list";

    /* Constructor */
    GScanTestSuite::GScanTestSuite(wifi_handle handle, wifi_request_id request_id)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "GScanTestSuite::GScanTestSuite: Created a GScan Test "
            "Suite with request_id:%d\n", request_id);
        id = request_id;
    }

    void GScanTestSuite::setRequestId(int reqId)
    {
        id = reqId;
    }

    int GScanTestSuite::getRequestId()
    {
        return id;
    }

    /* process the command line args */
    void GScanTestSuite::executeCmd(int argc, char **argv, int cmdIndex,
                                        int max, u32 flush,
                                        int band, oui scan_oui)
    {
        fprintf(stderr, "%s: Enter \n", __func__);

        if(argc < ARGV_IFACE_INDEX + 1)
        {
            fprintf(stderr, "insufficient GSCAN args\n");
            fprintf(stderr, "Usage : hal_proxy_daemon gscan interface_name\n");
            return;
        }

        ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[ARGV_IFACE_INDEX]);
        if(!ifaceHandle)
        {
            fprintf(stderr, "Interface %s is not up, exiting.\n",
                    argv[ARGV_IFACE_INDEX]);
            fprintf(stderr, "Please restart hal_proxy_daemon with a valid"
                " initialized interface\n");
            return;
        }

        switch(cmdIndex) {
            case 1:
                gscanSendStartRequest(argc, argv);
                break;
            case 2:
                gscanSendStopRequest(argc, argv);
                break;
            case 3:
                gscanSendGetValidChannelsRequest(argc, argv, max, band);
                break;
            case 4:
                gscanSendGetCapabilitiesRequest(argc, argv);
                break;
            case 5:
                gscanSendGetCachedResultsRequest(argc, argv, max, flush);
                break;
            case 6:
                gscanSendSetBssidHotlistRequest(argc, argv);
                break;
            case 7:
                gscanSendResetBssidHotlistRequest(argc, argv);
                break;
            case 8:
                gscanSendSetSignificantChangeRequest(argc, argv);
                break;
            case 9:
                gscanSendResetSignificantChangeRequest(argc, argv);
                break;
            case 10:
                gscanSendSetMacOui(argc, argv, scan_oui);
                break;
            case 11:
                gscanSendSetEpnoListRequest(argc, argv);
                break;
            case 12:
                gscanSendSetPasspointListRequest();
                break;
            case 13:
                gscanSendResetPasspointListRequest();
                break;
            case 15:
                gscanSendReSetEpnoListRequest(argc, argv);
                break;
            default:
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
        }
    }

    /* process the command line args */
    wifi_error GScanTestSuite::processCmd(int argc, char **argv)
    {
        int cmd = 0;
        oui scan_oui;

        if(argc < ARGV_IFACE_INDEX + 1)
        {
            fprintf(stderr, "insufficient GSCAN args\n");
            fprintf(stderr, "Usage : hal_proxy_daemon gscan interface_name\n");
            return WIFI_ERROR_INVALID_ARGS;
        }

        if(argc >= ARGV_CMDID_INDEX + 1) {
            ifaceHandle = wifi_get_iface_handle(wifiHandle_,
                                                argv[ARGV_IFACE_INDEX]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n",
                        argv[ARGV_IFACE_INDEX]);
                fprintf(stderr, "Please restart hal_proxy_daemon with a valid"
                        " initialized interface\n");
                return WIFI_ERROR_NOT_AVAILABLE;
            }
        } else {
            fprintf(stderr, "Please restart hal_proxy_daemon with a valid"
                    " initialized interface\n");
            return WIFI_ERROR_INVALID_ARGS;
        }

        cmd = atoi(argv[ARGV_CMDID_INDEX]);

        switch(cmd) {
            case GSCAN_CMD_START:
                gscanSendStartRequest(argc, argv);
                break;
            case GSCAN_CMD_STOP:
                gscanSendStopRequest(argc, argv);
                break;
            case GSCAN_CMD_GET_VALID_CHANNELS:
                if (ARGV_CMDID_INDEX + 2 < argc)
                    gscanSendGetValidChannelsRequest(argc, argv,
                                              atoi(argv[ARGV_CMDID_INDEX + 1]),
                                              atoi(argv[ARGV_CMDID_INDEX + 2]));
                else
                    fprintf(stderr, "insufficient args for GetValidChannels\n");
                break;
            case GSCAN_CMD_GET_CAPABILITIES:
                gscanSendGetCapabilitiesRequest(argc, argv);
                break;
            case GSCAN_CMD_GET_CACHED_RESULT:
                if (ARGV_CMDID_INDEX + 2 < argc)
                    gscanSendGetCachedResultsRequest(argc, argv,
                                              atoi(argv[ARGV_CMDID_INDEX + 1]),
                                              atoi(argv[ARGV_CMDID_INDEX + 2]));
                else
                    fprintf(stderr, "insufficient args for GetCachedResults\n");
                break;
            case GSCAN_CMD_SET_BSSID_HOTLIST:
                gscanSendSetBssidHotlistRequest(argc, argv);
                break;
            case GSCAN_CMD_RESET_BSSID_HOTLIST:
                gscanSendResetBssidHotlistRequest(argc, argv);
                break;
            case GSCAN_CMD_SET_SIGNIFICANT_CHANGE:
                gscanSendSetSignificantChangeRequest(argc, argv);
                break;
            case GSCAN_CMD_RESET_SIGNIFICANT_CHANGE:
                gscanSendResetSignificantChangeRequest(argc, argv);
                break;
            case GSCAN_CMD_SET_MAC_OUI:
                if ((ARGV_CMDID_INDEX + 1 < argc) &&
                    mac_addr_aton(scan_oui, argv[ARGV_CMDID_INDEX + 1],
                                  MAC_OUI_LEN) == 0)
                    gscanSendSetMacOui(argc, argv, scan_oui);
                break;
            case GSCAN_CMD_SET_EPNO_LIST:
                gscanSendSetEpnoListRequest(argc, argv);
                break;
            case GSCAN_CMD_SET_PASSPOINT_LIST:
                gscanSendSetPasspointListRequest();
                break;
            case GSCAN_CMD_RESET_PASSPOINT_LIST:
                gscanSendResetPasspointListRequest();
                break;
            case GSCAN_CMD_RESET_EPNO_LIST:
                gscanSendReSetEpnoListRequest(argc, argv);
                break;
            default:
                fprintf(stderr, "%s: unknown cmd :%d\n", argv[0], cmd);
                return WIFI_ERROR_INVALID_ARGS;
        }
        return WIFI_ERROR_NONE;
    }

    /* Helper routine to print usage */
    void GScanTestSuite::gscanPrintCmdUsage(char **argv, const char *cmd,
       const char *sub_cmd, const struct option long_options[], int size)
    {
        fprintf(stderr, "Usage: %s %s %s\n", argv[0], cmd, sub_cmd);
        for(int i = 1; i <= size-2; i++)
        {
            if(long_options[i].has_arg)
                fprintf(stderr, "\t[--%s arg]\n", long_options[i].name);
            else
                fprintf(stderr, "\t[--%s]\n", long_options[i].name);
        }
        return;
    }

    void GScanTestSuite::gscanSendGetValidChannelsRequest(
                                                           int argc,
                                                           char **argv,
                                                           int max_channels,
                                                           int band)
    {
        int num_channels;
        wifi_channel *channels = NULL;
        int ret = 0;

        /* A string listing valid short options letters.  */
        const char* const short_options = "hb:m:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "band",         1,   NULL, 'b' },
            { "max_channels", 1,   NULL, 'm' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };

        /* Override with command line arguements */
        int long_index = 0, opt = 0, i;

        fprintf(stderr, "GScanTestSuite::gscanSendGetValidChannelsRequest: "
            "Entry - Sending GSCAN Get Valid Channels Request.\n");
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'b' :
                    band = atoi(optarg);
                    break;
                case 'm' :
                    max_channels = atoi(optarg);
                    break;
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD,
                        GSCAN_GET_VALID_CHANNELS, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        if (band < 0) {
            fprintf(stderr, "Invalid band provided:%d\n",
                band);
            goto cleanup;
        }

        if (max_channels <= 0 || max_channels > MAX_CHANNEL_LIST) {
            fprintf(stderr, "Invalid max_channels provided:%d. Please provide "
                    "value between 1 - %d\n", max_channels, MAX_CHANNEL_LIST);
            goto cleanup;
        }

        channels =
            (wifi_channel *) malloc (max_channels * sizeof(wifi_channel));
        if (!channels)
            goto cleanup;
        memset(channels, 0, max_channels * sizeof(wifi_channel));
        fprintf(stderr, "GScanTestSuite::gscanSendGetValidChannelsRequest: "
            "Sending Get Valid Channels Request. band: %d, "
            "max_channels: %d.\n", band, max_channels);

        ret = wifi_get_valid_channels(ifaceHandle, band, max_channels,
                channels, &num_channels);
        if (ret)
            goto cleanup;
        fprintf(stderr, "GScanTestSuite::gscanSendGetValidChannelsRequest:"
            "Get valid channels event received\n");
        fprintf(stderr, "GScanTestSuite::gscanSendGetValidChannelsRequest:"
            "Num channels : %d \n", num_channels);
        fprintf(stderr, "channels : ");
        for(i = 0; i < num_channels; i++)
        {
            fprintf(stderr, "%u,  ", *(channels + i));
        }
        fprintf(stderr, "\n");
    cleanup:
        if(channels)
            free(channels);
        channels = NULL;
    }

    void GScanTestSuite::gscanSendGetCapabilitiesRequest(int argc, char **argv)
    {
        int ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        wifi_gscan_capabilities capa;
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD, GSCAN_GET_CAPABILITIES,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "%s: Sending Get Capabilities Request. \n",
            __FUNCTION__);
        ret = wifi_get_gscan_capabilities(ifaceHandle, &capa);
        fprintf(stderr, "%s: Received GSCAN Capabilities with value:%d. \n",
            __FUNCTION__, ret);
        if (ret)
            return;
        fprintf(stderr, "%s: Capabilities:\n    max_ap_cache_per_scan:%d,\n "
            "   max_bssid_history_entries:%d,\n    max_hotlist_bssids:%d,\n "
            "   max_hotlist_ssids:%d,\n "
            "   max_rssi_sample_size:%d,\n "
            "   max_scan_buckets:%d,\n "
            "   max_scan_cache_size:%d,\n "
            "   max_scan_reporting_threshold:%d,\n "
            "   max_significant_wifi_change_aps:%d.\n"
            "   max_number_epno_networks:%d,\n "
            "   max_number_epno_networks_by_ssid:%d,\n "
            "   max_number_of_white_listed_ssid:%d,\n ",
            __FUNCTION__, capa.max_ap_cache_per_scan,
            capa.max_bssid_history_entries, capa.max_hotlist_bssids,
            capa.max_hotlist_ssids,
            capa.max_rssi_sample_size, capa.max_scan_buckets,
            capa.max_scan_cache_size, capa.max_scan_reporting_threshold,
            capa.max_significant_wifi_change_aps,
            capa.max_number_epno_networks,
            capa.max_number_epno_networks_by_ssid,
            capa.max_number_of_white_listed_ssid);
    }

    void GScanTestSuite::gscanSendStartRequest(int argc, char **argv)
    {
        fprintf(stderr, "%s: Sending GSCAN Start Request. "
            "\n", __FUNCTION__);
        int i, j, ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD,
                        GSCAN_SET_BSSID_HOTLIST,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        wifi_scan_result_handler handler;
        wifi_scan_cmd_params params;

        FILE *f_read = fopen("/vendor/etc/wifi/gscan_start_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if ( (fscanf(f_read, "%d", &params.num_buckets) &&
                fscanf(f_read, "%d", &params.base_period) &&
                fscanf(f_read, "%d", &params.max_ap_per_scan) &&
                fscanf(f_read, "%d", &params.report_threshold_percent)  &&
                fscanf(f_read, "%d", &params.report_threshold_num_scans)) == 0 ) {
                fprintf(stderr, "%s: Failed parsing GSCAN Start "
                    "params file. Exit\n", __func__);
                fclose(f_read);
                return;
            }
            fprintf(stderr, "Number of buckets:%d\n"
                "base_period:%d\n"
                "max_ap_per_scan:%d\n"
                "report_threshold_percent:%d\n"
                "report_threshold_num_scans:%d\n\n",
                    params.num_buckets,
                    params.base_period,
                    params.max_ap_per_scan,
                    params.report_threshold_percent,
                    params.report_threshold_num_scans);

            /* Outer loop for parsing scan buckets */
            for ( i =0; i < params.num_buckets; i++ )
            {
                if ( (fscanf(f_read, "%d", &params.buckets[i].bucket) &&
                    fscanf(f_read, "%d", (int *)&params.buckets[i].band) &&
                    fscanf(f_read, "%d", (int *)&params.buckets[i].period) &&
                    fscanf(f_read, "%d", (int *)&params.buckets[i].report_events) &&
                    fscanf(f_read, "%d", &params.buckets[i].max_period) &&
                    fscanf(f_read, "%d", &params.buckets[i].base) &&
                    fscanf(f_read, "%d", &params.buckets[i].step_count) &&
                    fscanf(f_read, "%d", &params.buckets[i].num_channels)) == 0 ) {
                    fprintf(stderr, "%s: Failed parsing GSCAN Start "
                        "params file. Exit\n", __func__);
                    fclose(f_read);
                    return;
                }

                fprintf(stderr, "params.buckets[%d].index:%d\n",
                        i, params.buckets[i].bucket);

                fprintf(stderr, "params.buckets[%d].band:%d\n"
                            "params.buckets[%d].period:%d\n"
                            "params.buckets[%d].report_events:%d\n"
                            "params.buckets[%d].max_period:%d\n"
                            "params.buckets[%d].base:%d\n"
                            "params.buckets[%d].step_count:%d\n"
                            "params.buckets[%d].num_channels:%d\n\n",
                            i, params.buckets[i].band,
                            i, params.buckets[i].period,
                            i, params.buckets[i].report_events,
                            i, params.buckets[i].max_period,
                            i, params.buckets[i].base,
                            i, params.buckets[i].step_count,
                            i, params.buckets[i].num_channels);
                /* Inner loop for parsing channel buckets */
                for ( j = 0; j < params.buckets[i].num_channels; j++ )
                {
                    if ( (fscanf(f_read, "%d", &params.buckets[i].channels[j].channel) &&
                        fscanf(f_read, "%d", &params.buckets[i].channels[j].dwellTimeMs) &&
                        fscanf(f_read, "%d", &params.buckets[i].channels[j].passive)) == 0) {
                        fprintf(stderr, "%s: Failed parsing GSCAN Start "
                            "params file. Exit\n", __func__);
                        fclose(f_read);
                        return;
                    }

                    fprintf(stderr, "Channel buckets of Scan Bucket[%d]\n", i);

                    fprintf(stderr, "   buckets[%d].channels[%d].channel:%dMHz\n "
                        "   buckets[%d].channels[%d].dwellTimeMs:%d\n"
                            "   buckets[%d].channels[%d].passive:%d \n\n",
                            i, j, params.buckets[i].channels[j].channel,
                            i, j, params.buckets[i].channels[j].dwellTimeMs,
                            i, j, params.buckets[i].channels[j].passive);
                }
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "gscanSendStartRequest: Failed to "
            "open file /vendor/etc/wifi/gscan_start_params.txt - "
            "use hard-coded defaults\n");

            params.base_period = 20; /*20ms*/
            params.max_ap_per_scan = 3;
            params.num_buckets = 1;
            params.report_threshold_percent= 50;
            params.report_threshold_num_scans= 30;

            params.buckets[0].bucket = 0;
            params.buckets[0].band = WIFI_BAND_BG;
            params.buckets[0].period = 20;
            params.buckets[0].report_events = 0;
            /* Hardcode info for 1 scan bucket. */
            params.buckets[0].num_channels = 1;

            params.buckets[0].channels[0].channel = 2437;
            params.buckets[0].channels[0].dwellTimeMs = 20;
            params.buckets[0].channels[0].passive = 1;

        }

        /* Set the callback handler functions for related events. */
        handler.on_full_scan_result = gscan_on_full_scan_result;
        handler.on_scan_event = gscan_on_scan_event;

        //ret = wifi_start_gscan(id, wifiHandle_,
        ret = wifi_start_gscan(id, ifaceHandle,
                                params,
                                handler);
        fprintf(stderr, "%s: Sending GSCAN Start request "
            "completed. Returned value: %d.\n", __func__, ret);

        if (ret)
            return;
    }

    void GScanTestSuite::gscanSendStopRequest(int argc, char **argv)
    {
        int ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "hr:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "request_id",   1,   NULL, 'r' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };

        /* Override with command line arguements */
        int long_index = 0, opt = 0;

        fprintf(stderr, "GScanTestSuite::gscanSendStopRequest: Entry - Sending "
            "GSCAN Stop Request.\n");
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'r' :
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD, GSCAN_STOP,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "GScanTestSuite::gscanSendStopRequest: Stop GSCAN for "
            "request_id:%d.\n", id);
        ret = wifi_stop_gscan(id, ifaceHandle);
        fprintf(stderr, "%s: Sending GSCAN Stop Request completed. "
            "Returned value: %d.\n", __func__, ret);
        if (ret == WIFI_ERROR_NOT_AVAILABLE)
            fprintf(stderr, "GSCAN isn't running or already stopped. ");
    }

    void GScanTestSuite::gscanSendSetBssidHotlistRequest(int argc, char **argv)
    {
        fprintf(stderr, "gscanSendSetBssidHotlistRequest: Sending GSCAN "
            "SetBssidHotlist Request. \n");
        int i, ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD, GSCAN_SET_BSSID_HOTLIST,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "Parse params from "
            "/vendor/etc/wifi/gscan_set_hotlist_params.txt \n");
        wifi_hotlist_ap_found_handler handler;
        wifi_bssid_hotlist_params params;

        memset(&params, 0, sizeof(wifi_bssid_hotlist_params));

        FILE *f_read = fopen("/vendor/etc/wifi/gscan_set_hotlist_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if (fscanf(f_read, "%d", &params.lost_ap_sample_size) &&
                fscanf(f_read, "%d", &params.num_bssid)) {
                fprintf(stderr, "gscanSendSetBssidHotlistRequest: Parsed "
                    "number of BSSID hotlist:%d\n lost_ap_sample_size:%d\n",
                    params.num_bssid, params.lost_ap_sample_size);
            }
            for ( i = 0; i < params.num_bssid; i++ )
            {
                if ( (fscanf(f_read, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        &params.ap[i].bssid[0], &params.ap[i].bssid[1],
                        &params.ap[i].bssid[2], &params.ap[i].bssid[3],
                        &params.ap[i].bssid[4], &params.ap[i].bssid[5]) &&
                    fscanf(f_read, "%d", &params.ap[i].low) &&
                    fscanf(f_read, "%d", &params.ap[i].high)) == 0 ) {
                    fprintf(stderr, "%s: Failed parsing GSCAN BSSID hotlist "
                        "params file. Exit\n", __func__);
                    fclose(f_read);
                    return;
                }

                fprintf(stderr, "ap[%d].bssid:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\n",
                        i,
                        params.ap[i].bssid[0], params.ap[i].bssid[1],
                        params.ap[i].bssid[2],params.ap[i].bssid[3],
                        params.ap[i].bssid[4], params.ap[i].bssid[5]);

                fprintf(stderr, "ap[%d].low:%d\n ap[%d].high:%d\n\n",
                            i, params.ap[i].low,
                            i, params.ap[i].high);
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "gscanSendSetBssidHotlistRequest: Failed to open "
            "file /vendor/etc/wifi/gscan_set_hotlist_params.txt  - "
            "use hard-coded defaults\n");

            params.lost_ap_sample_size = 10;
            params.num_bssid = 3;
            memcpy(params.ap[0].bssid, "012345", sizeof(mac_addr));
            params.ap[0].low = -80;
            params.ap[0].high = -20;

            memcpy(params.ap[1].bssid, "678901", sizeof(mac_addr));
            params.ap[1].low = -70;
            params.ap[1].high = -30;

            memcpy(params.ap[2].bssid, "333333", sizeof(mac_addr));
            params.ap[2].low = -50;
            params.ap[2].high = -60;
        }

        handler.on_hotlist_ap_found = gscan_on_hotlist_ap_found;
        handler.on_hotlist_ap_lost = gscan_on_hotlist_ap_lost;
        ret = wifi_set_bssid_hotlist(id, ifaceHandle,
                                    params,
                                    handler);
        fprintf(stderr, "gscanSendSetBssidHotlistRequest: Sending GSCAN Set "
            "Bssid Hotlist request completed. Returned value: %d.\n", ret);

        if (ret)
            return;
    }

    void GScanTestSuite::gscanSendResetBssidHotlistRequest(int argc,
                                                                char **argv)
    {
        int ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "hr:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "request_id",   1,   NULL, 'r' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };

        /* Override with command line arguements */
        int long_index = 0, opt = 0;

        fprintf(stderr, "%s: Entry - Sending GSCAN Reset Bssid Hotlist "
            "Request.\n", __func__);

        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'r' :
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD,
                        GSCAN_RESET_BSSID_HOTLIST, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "%s: Reset BSSID Hotlist for request_id:%d.\n",
            __FUNCTION__, id);
        ret = wifi_reset_bssid_hotlist(id, ifaceHandle);
        fprintf(stderr, "%s: Sending GSCAN Reset Bssid Hotlist request"
            "completed. Returned value: %d.\n", __FUNCTION__, ret);
    }

    void GScanTestSuite::gscanSendSetSignificantChangeRequest(int argc,
                                                                char **argv)
    {
        fprintf(stderr, "%s: Sending GSCAN SetSignificantChange Request. "
            "\n", __func__);
        int i, ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD,
                       GSCAN_SET_SIGNIFICANT_CHANGE, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        wifi_significant_change_handler handler;
        wifi_significant_change_params params;

        FILE *f_read =
            fopen("/vendor/etc/wifi/gscan_set_significant_change_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if ( (fscanf(f_read, "%d", &params.num_bssid) &&
                fscanf(f_read, "%d", &params.rssi_sample_size) &&
                fscanf(f_read, "%d", &params.lost_ap_sample_size) &&
                fscanf(f_read, "%d", &params.min_breaching)) == 0 ) {
                fprintf(stderr, "%s: Failed parsing GSCAN Significant "
                    "change params file. Exit\n", __func__);
                fclose(f_read);
                return;
            }
            fprintf(stderr, "Number of AP params:%d\n Rssi_sample_size:%d\n "
                "lost_ap_sample_size:%d\n min_breaching:%d\n\n",
                    params.num_bssid,
                    params.rssi_sample_size,
                    params.lost_ap_sample_size,
                    params.min_breaching);
            for ( i =0; i < params.num_bssid; i++ )
            {
                if ( (fscanf(f_read, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        &params.ap[i].bssid[0], &params.ap[i].bssid[1],
                        &params.ap[i].bssid[2], &params.ap[i].bssid[3],
                        &params.ap[i].bssid[4], &params.ap[i].bssid[5]) &&
                    fscanf(f_read, "%d", &params.ap[i].low) &&
                    fscanf(f_read, "%d", &params.ap[i].high)) == 0 ) {
                    fprintf(stderr, "%s: Failed parsing GSCAN Significant "
                        "change params file. Exit\n", __func__);
                    fclose(f_read);
                    return;
                }

                fprintf(stderr, "ap[%d].bssid:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\n",
                        i,
                        params.ap[i].bssid[0], params.ap[i].bssid[1],
                        params.ap[i].bssid[2],params.ap[i].bssid[3],
                        params.ap[i].bssid[4], params.ap[i].bssid[5]);

                fprintf(stderr, "ap[%d].low:%d\n ap[%d].high:%d\n\n",
                            i, params.ap[i].low,
                            i, params.ap[i].high);
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "gscanSendSetSignificantChangeRequest: Failed to "
            "open file /vendor/etc/wifi/gscan_set_significant_change_params.txt  - "
            "use hard-coded defaults\n");

            params.num_bssid = 3;
            params.rssi_sample_size = 20;
            params.lost_ap_sample_size = 10;
            params.min_breaching = 5;
            memcpy(params.ap[0].bssid, "543210", sizeof(mac_addr));
            params.ap[0].low = -60;
            params.ap[0].high = -50;

            memcpy(params.ap[1].bssid, "678901", sizeof(mac_addr));
            params.ap[1].low = -70;
            params.ap[1].high = -30;

            memcpy(params.ap[2].bssid, "333333", sizeof(mac_addr));
            params.ap[2].low = -80;
            params.ap[2].high = -20;
        }
        handler.on_significant_change = gscan_on_significant_change;
        ret = wifi_set_significant_change_handler(id, ifaceHandle,
                                                params,
                                                handler);
        fprintf(stderr, "gscanSendSetSignificantChangeRequest: Sending GSCAN "
            "Set Significant Change request completed. Returned value: %d.\n",
            ret);

        if (ret)
            return;
    }

    void GScanTestSuite::gscanSendGetCachedResultsRequest(int argc,
                                                                    char **argv,
                                                                    int max,
                                                                    int flush)
    {
        int ret = 0;
        int num = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD,
                        GSCAN_GET_CACHED_RESULTS, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        /* If an invalid value is provided, set to some working default. */
        if (flush < 0) {
            fprintf(stderr, "Invalid flush provided:%d, set to 1\n",
                flush);
            flush = 1;
        }
        /* If an invalid value is provided, set max to some working default. */
        if (max <= 0 || max > GSCAN_MAX_CACHED_RESULTS) {
            fprintf(stderr, "Invalid max provided:%d, set max = %d. \n",
                max, GSCAN_MAX_CACHED_RESULTS);
            max = GSCAN_MAX_CACHED_RESULTS;
        }

        fprintf(stderr, "GScanTestSuite::gscanSendGetCachedResultsRequest: "
            "Sending Get Cached Results Request with i/p values: "
            "flush:%d, max:%d \n", flush, max);
        wifi_cached_scan_results *cached_results = (wifi_cached_scan_results *)
            malloc (max * sizeof(wifi_cached_scan_results));
        if (!cached_results) {
            fprintf(stderr, "Failed to allocate memory for "
                "cached_results\n");
            return;
        }

        if (cached_results)
        {
            memset(cached_results, 0, max * sizeof(wifi_cached_scan_results));
            ret = wifi_get_cached_gscan_results(ifaceHandle,
                    flush, max,
                    cached_results,
                    &num);
            fprintf(stderr, "gscanSendGetCachedResultsRequest: "
                    "Sending Get Cached Results request "
                    "completed. Returned value: %d.\n\n", ret);

            if (ret) {
                free(cached_results);
                return;
            }
            fprintf(stderr, "Number of cached results returned: %d\n",
                    num);
            //wifi_cached_scan_results *result = cached_results;
            int i = 0, j = 0;
            for(i = 0; i < num; i++)
            {
                fprintf(stderr, "TEST-PROXY:  Scan_Id  %d \n",
                    cached_results[i].scan_id);
                fprintf(stderr, "TEST-PROXY:  Flags  %u \n",
                    cached_results[i].flags);
                fprintf(stderr, "TEST-PROXY:  Buckets scanned  %u \n",
                    cached_results[i].buckets_scanned);
                fprintf(stderr, "TEST-PROXY:  Num_Results  %d \n\n",
                    cached_results[i].num_results);
                for(j = 0; j < cached_results[i].num_results; j++)
                {
                    fprintf(stderr, "TEST-PROXY:  Wi-Fi Scan Result : %d\n",
                        j + 1);
                    fprintf(stderr, "TEST-PROXY:  ts  %" PRId64 "\n",
                        cached_results[i].results[j].ts);
                    fprintf(stderr, "TEST-PROXY:  SSID  %s \n",
                        cached_results[i].results[j].ssid) ;
                    fprintf(stderr, "TEST-PROXY:  BSSID: "
                            "%02x:%02x:%02x:%02x:%02x:%02x \n",
                            cached_results[i].results[j].bssid[0],
                            cached_results[i].results[j].bssid[1],
                            cached_results[i].results[j].bssid[2],
                            cached_results[i].results[j].bssid[3],
                            cached_results[i].results[j].bssid[4],
                            cached_results[i].results[j].bssid[5]);
                    fprintf(stderr, "TEST-PROXY:  channel %d \n",
                        cached_results[i].results[j].channel);
                    fprintf(stderr, "TEST-PROXY:  rssi  %d \n",
                            (signed char) cached_results[i].results[j].rssi);
                    fprintf(stderr, "TEST-PROXY:  rtt  %" PRId64 "\n",
                        cached_results[i].results[j].rtt);
                    fprintf(stderr, "TEST-PROXY:  rtt_sd  %" PRId64 "\n",
                            cached_results[i].results[j].rtt_sd);
                }
            }
            free(cached_results);
        }
        else
        {
            fprintf(stderr, "TEST-PROXY:  Failed to alloc: \n");
        }

   }

    void GScanTestSuite::gscanSendResetSignificantChangeRequest(int argc,
                                                                char **argv)
    {
        int ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "hr:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "request_id",   1,   NULL, 'r' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };

        /* Override with command line arguements */
        int long_index = 0, opt = 0;

        fprintf(stderr, "%s: Sending GSCAN Reset Significant Change Request. ",
            __FUNCTION__);

        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'r' :
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    gscanPrintCmdUsage(argv, GSCAN_CMD,
                        GSCAN_RESET_SIGNIFICANT_CHANGE, long_options,
                           sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "%s: Reset Significant Change for request_id:%d.\n",
            __FUNCTION__, id);

        ret = wifi_reset_significant_change_handler(id, ifaceHandle);
        fprintf(stderr, "%s: Sending GSCAN Reset Significant Change request"
            "completed. Returned value: %d.\n", __FUNCTION__, ret);
    }

    void GScanTestSuite::gscanSendSetMacOui(int argc, char **argv, oui scan_oui)
    {
        UNUSED(argc), UNUSED(argv);
        int ret = 0;
        fprintf(stderr, "%s: Sending MAC OUI to the wifiHAL. "
            "\n", __FUNCTION__);

        ret = wifi_set_scanning_mac_oui(ifaceHandle, scan_oui);
        if (ret)
        {
            fprintf(stderr, "%s: Set MAC OUI failed.\n", __func__);
            return;
        }
    }

    int GScanTestSuite::gscanParseHex(unsigned char c)
    {
       if (c >= '0' && c <= '9')
          return c-'0';
       if (c >= 'a' && c <= 'f')
          return c-'a'+10;
       if (c >= 'A' && c <= 'F')
          return c-'A'+10;
       return 0;
    }

    int GScanTestSuite::gscanParseMacAddress(const char* arg, u8* addr)
    {
       if (strlen(arg) != 17)
       {
          fprintf(stderr, "Invalid mac address %s\n", arg);
          fprintf(stderr, "expected format xx:xx:xx:xx:xx:xx\n");
          return -1;
       }

       addr[0] = gscanParseHex(arg[0]) << 4 | gscanParseHex(arg[1]);
       addr[1] = gscanParseHex(arg[3]) << 4 | gscanParseHex(arg[4]);
       addr[2] = gscanParseHex(arg[6]) << 4 | gscanParseHex(arg[7]);
       addr[3] = gscanParseHex(arg[9]) << 4 | gscanParseHex(arg[10]);
       addr[4] = gscanParseHex(arg[12]) << 4 | gscanParseHex(arg[13]);
       addr[5] = gscanParseHex(arg[15]) << 4 | gscanParseHex(arg[16]);
       return 0;
    }

    void GScanTestSuite::gscanSendSetEpnoListRequest(int argc, char **argv)
    {
        fprintf(stderr, "gscanSendSetEpnoListRequest: Sending GSCAN "
            "Set ePno List Request. \n");
        int i = 0, ret = 0;
        const int no_of_params_epno_network = 3;
        unsigned int flags, auth_bit_field;

        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "num_networks",               0,   NULL, 'n' }, //n - num_network
            { "min5GHz_rssi",               0,   NULL, 'f' }, //f - five GHz
            { "min24GHz_rssi",              0,   NULL, 't' }, //t - two GHz
            { "initial_score_max",          0,   NULL, 'i' }, //i - initial
            { "current_connection_bonus",   0,   NULL, 'c' }, //c - current
            { "same_network_bonus",         0,   NULL, 's' }, //s - same network
            { "secure_bonus",               0,   NULL, 'p' }, //p - protected
                                                              //    network
            { "band5GHz_bonus",             0,   NULL, 'b' }, //b - num_network
            { "networks",                   0,   NULL, 'N' }, //N - networks
            { "ssid",                       0,   NULL, 'S' }, //S - num_network
            { "flags",                      0,   NULL, 'F' }, //F - num_network
            { "auth_bit_field",             0,   NULL, 'B' }, //B - num_network
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        const char *ssids[] = {"Hydra-Test1", "Hydra-Test2", "Hydra-Test3"};
        wifi_epno_handler handler;
        wifi_epno_params *epno_params = (wifi_epno_params *) malloc(
                                         sizeof(wifi_epno_params) +
                                         sizeof(wifi_epno_network) *
                                         MAX_EPNO_NETWORKS);
        if (!epno_params) {
            fprintf(stderr, "Failed to allocate memory for wifi_epno_params");
            return;
        }

        if (argc > 3) {
            u8 j = 0;
            while ((opt = getopt_long(argc, argv, short_options,
                    long_options, &long_index )) != -1)
            {
                switch (opt)
                {
                    case 'n':
                        epno_params->num_networks = atoi(optarg);
                        break;
                    case 'f':
                        epno_params->min5GHz_rssi = atoi(optarg);
                        break;
                    case 't':
                        epno_params->min24GHz_rssi = atoi(optarg);
                        break;
                    case 'i':
                        epno_params->initial_score_max = atoi(optarg);
                        break;
                    case 'c':
                        epno_params->current_connection_bonus = atoi(optarg);
                        break;
                    case 's':
                        epno_params->same_network_bonus = atoi(optarg);
                        break;
                    case 'p':
                        epno_params->secure_bonus = atoi(optarg);
                        break;
                    case 'b':
                        epno_params->band5GHz_bonus = atoi(optarg);
                        break;
                    case 'N':
                        while ((opt = getopt_long(argc, argv, short_options,
                                        long_options, &long_index )) != -1)
                        {
                            switch (opt)
                            {
                                case 'S':
                                    memcpy(epno_params->networks[i].ssid, optarg,
                                            strlen(optarg));
                                    j++;
                                    break;
                                case 'F':
                                    epno_params->networks[i].flags = atoi(optarg);
                                    j++;
                                    break;
                                case 'B':
                                    epno_params->networks[i].auth_bit_field
                                                                 = atoi(optarg);
                                    j++;
                                    break;
                                default:
                                    if (j < no_of_params_epno_network) {
                                        fprintf(stderr, "Provide complete network"
                                              " configuration");
                                        gscanPrintCmdUsage(argv, GSCAN_CMD, GSCAN_SET_EPNO_LIST,
                                                long_options,
                                                sizeof(long_options)/sizeof(struct option));
                                        free(epno_params);
                                        return;
                                    }
                            }
                            if (j == no_of_params_epno_network) {
                                j = 0;
                                i++;
                            }
                        }
                        break;
                    case 'h':
                        break;
                    default:
                        gscanPrintCmdUsage(argv, GSCAN_CMD, GSCAN_SET_EPNO_LIST,
                                long_options,
                                sizeof(long_options)/sizeof(struct option));
                        free(epno_params);
                        return;
                }
            }
        } else {
            FILE *f_read = fopen("/vendor/etc/wifi/gscan_set_epno_list_params.txt", "r");
            if (f_read) {
                fprintf(stderr, "Parse params from "
                        "/vendor/etc/wifi/gscan_set_epno_list_params.txt \n");
                if ((fscanf(f_read, "%d", &epno_params->num_networks) &&
                    fscanf(f_read, "%d", &epno_params->min5GHz_rssi) &&
                    fscanf(f_read, "%d", &epno_params->min24GHz_rssi) &&
                    fscanf(f_read, "%d", &epno_params->initial_score_max) &&
                    fscanf(f_read, "%d", &epno_params->current_connection_bonus) &&
                    fscanf(f_read, "%d", &epno_params->same_network_bonus) &&
                    fscanf(f_read, "%d", &epno_params->secure_bonus) &&
                    fscanf(f_read, "%d", &epno_params->band5GHz_bonus)) == 0) {
                    fprintf(stderr, "gscanSendSetEpnoListRequest: Failed to Parse");
                    fclose(f_read);
                    free(epno_params);
                    return;
                }

                fprintf(stderr, "number of ePNO networks:%d\n",
                                 epno_params->num_networks);
                fprintf(stderr, "min5GHz_rssi:%d\n", epno_params->min5GHz_rssi);
                fprintf(stderr, "min24GHz_rssi:%d\n", epno_params->min24GHz_rssi);
                fprintf(stderr, "initial_score_max:%d\n", epno_params->initial_score_max);
                fprintf(stderr, "current_connection_bonus:%d\n",
                                 epno_params->current_connection_bonus);
                fprintf(stderr, "same_network_bonus:%d\n", epno_params->same_network_bonus);
                fprintf(stderr, "secure_bonus:%d\n", epno_params->secure_bonus);
                fprintf(stderr, "band5GHz_bonus:%d\n", epno_params->band5GHz_bonus);

                for ( i = 0; i < epno_params->num_networks; i++ )
                {
                    if ((fscanf(f_read, "%32s", epno_params->networks[i].ssid) &&
                        fscanf(f_read, "%u", &flags) &&
                        fscanf(f_read, "%u", &auth_bit_field)) == 0 ) {
                        fprintf(stderr, "%s: Failed parsing GSCAN Set ePNO list "
                            "params file. Exit\n", __func__);
                        fclose(f_read);
                        free(epno_params);
                        return;
                    }

                    epno_params->networks[i].flags = (char)flags;
                    epno_params->networks[i].auth_bit_field = (char)auth_bit_field;

                    fprintf(stderr, "ePNO-Network[%d].SSID: %s \n",
                                i, epno_params->networks[i].ssid);
                    fprintf(stderr, "ePNO-Network[%d].flags:%d\n "
                                "ePNO-Network[%d].auth_bit_field:%u\n\n",
                                i, epno_params->networks[i].flags,
                                i, epno_params->networks[i].auth_bit_field);
                }
                fclose(f_read);
            } else {
                fprintf(stderr, "gscanSendSetEpnoListRequest: Failed to open "
                "file /vendor/etc/wifi/gscan_set_epno_list_params.txt  - "
                "use hard-coded defaults\n");

                epno_params->min5GHz_rssi = -60;
                epno_params->min24GHz_rssi = -60;
                epno_params->initial_score_max = 5;
                epno_params->current_connection_bonus = 5;
                epno_params->same_network_bonus = 2;
                epno_params->secure_bonus = 10;
                epno_params->band5GHz_bonus = 3;
                epno_params->num_networks = 3;

                strlcpy(epno_params->networks[0].ssid, ssids[0], sizeof(epno_params->networks[0].ssid));
                epno_params->networks[0].flags = 7;
                epno_params->networks[0].auth_bit_field = 1;

                strlcpy(epno_params->networks[1].ssid, ssids[1], sizeof(epno_params->networks[1].ssid));
                epno_params->networks[1].flags = 9;
                epno_params->networks[1].auth_bit_field = 1;

                strlcpy(epno_params->networks[2].ssid, ssids[2], sizeof(epno_params->networks[2].ssid));
                epno_params->networks[2].flags = 15;
                epno_params->networks[2].auth_bit_field = 0;

                for ( i =0; i < epno_params->num_networks; i++ )
                {
                    fprintf(stderr, "ePNO-Network[%d].SSID: %s \n",
                                i, epno_params->networks[i].ssid);
                    fprintf(stderr, "ePNO-Network[%d].flags:%d\n "
                                "ePNO-Network[%d].auth_bit_field:%u\n\n",
                                i, epno_params->networks[i].flags,
                                i, epno_params->networks[i].auth_bit_field);
                }
            }
        }

        handler.on_network_found = gscan_on_epno_network_found;

        ret = wifi_set_epno_list(id, ifaceHandle, epno_params, handler);
        fprintf(stderr, "%s: Sending set EPNO request %s\n",
                __FUNCTION__, ret?"Failed":"Success");
        free(epno_params);

    }


    void GScanTestSuite::gscanSendReSetEpnoListRequest(int argc,
                                                     char **argv)
    {
        UNUSED(argc), UNUSED(argv);
        int ret = 0;

        fprintf(stderr, "%s: Reset EPNO request for request_id:%d.\n",
            __FUNCTION__, id);

        ret = wifi_reset_epno_list(id, ifaceHandle);
        fprintf(stderr, "%s: Sending reset EPNO request %s\n",
                __FUNCTION__, ret?"Failed":"Success");
    }


    void GScanTestSuite::gscanSendSetPasspointListRequest()
    {
        fprintf(stderr, "gscanSendSetPasspointListRequest: Sending GSCAN "
            "Set Passpoint List Request. \n");
        int i, j, ret = 0, num_networks;

        fprintf(stderr, "Parse params from "
            "/vendor/etc/wifi/gscan_set_passpoint_list_params.txt \n");
        wifi_passpoint_event_handler handler;
        wifi_passpoint_network *networks = (wifi_passpoint_network *) malloc
            (sizeof(wifi_passpoint_network) * MAX_EPNO_NETWORKS);
        if (!networks) {
            fprintf(stderr, "Failed to allocate memory for wifi_passpoint_network "
                "*networks. Exit. ");
            return;
        }
        FILE *f_read = fopen("/vendor/etc/wifi/gscan_set_passpoint_list_params.txt", "r");

        fprintf(stderr, "\n");
        byte *ptrTemp;
        int64_t *addTmp;
        if (f_read) {
            if (fscanf(f_read, "%d", &num_networks)) {
                fprintf(stderr, "gscanSendSetPasspointListRequest: Parsed "
                    "number of Passpoint list networks:%d\n", num_networks);
            }
            for ( i = 0; i < num_networks; i++ )
            {
                if ( (fscanf(f_read, "%u", &networks[i].id) &&
                    fscanf(f_read, "%255s", networks[i].realm)) == 0 ) {
                    fprintf(stderr, "%s: Failed parsing GSCAN Set Passpoint "
                        "list params file. Exit\n", __func__);
                    fclose(f_read);
                    free(networks);
                    return;
                }

                for ( j = 0; j < 16; j++ )
                {
                    addTmp = &(networks[i].roamingConsortiumIds[j]);
                    ptrTemp = (byte*)addTmp;
                    if ( (fscanf(f_read, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        ptrTemp,
                        ptrTemp + 1,
                        ptrTemp + 2,
                        ptrTemp + 3,
                        ptrTemp + 4,
                        ptrTemp + 5,
                        ptrTemp + 6,
                        ptrTemp + 7)) == 0)
                    {
                        fprintf(stderr, "%s: Failed parsing GSCAN Set Passpoint "
                            "list params file. Exit\n", __func__);
                        fclose(f_read);
                        free(networks);
                        return;
                    }

                }

                if ( (fscanf(f_read, "%hhx:%hhx:%hhx",
                        &networks[i].plmn[0], &networks[i].plmn[1],
                        &networks[i].plmn[2])) == 0 )
                {
                    fprintf(stderr, "%s: Failed parsing GSCAN Passpoint List "
                        "params file. Exit\n", __func__);
                    fclose(f_read);
                    free(networks);
                    return;
                }
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "gscanSendSetPasspointListRequest: Failed to open "
            "file /vendor/etc/wifi/gscan_set_passpoint_list_params.txt - "
            "use hard-coded defaults\n");
            free(networks);
            return;
        }

        handler.on_passpoint_network_found = gscan_on_passpoint_network_found;

        for ( i = 0; i < num_networks; i++ )
        {
            fprintf(stderr, "Passpoint-Network[%d].id: %d \n",
                        i, networks[i].id);
            fprintf(stderr, "Passpoint-Network[%d].realm: %s \n",
                        i, networks[i].realm);
            for ( j = 0; j < 16; j++ )
            {
                ptrTemp = (byte*)&(networks[i].roamingConsortiumIds[j]);
                fprintf(stderr, "    %hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\n",
                    *ptrTemp,
                    *(ptrTemp + 1),
                    *(ptrTemp + 2),
                    *(ptrTemp + 3),
                    *(ptrTemp + 4),
                    *(ptrTemp + 5),
                    *(ptrTemp + 6),
                    *(ptrTemp + 7));
            }
            fprintf(stderr, "\nPasspoint-Network[%d].plmn: %hhx%hhx%hhx\n\n\n",
                                i,
                                networks[i].plmn[0], networks[i].plmn[1],
                                networks[i].plmn[2]);
        }
        ret = wifi_set_passpoint_list(id, ifaceHandle, num_networks,
                                networks, handler);
        fprintf(stderr, "gscanSendSetPasspointListRequest: Sending GSCAN Set "
            "Passpoint List request completed. Returned value: %d.\n", ret);
        free(networks);

    }

    void GScanTestSuite::gscanSendResetPasspointListRequest()
    {
        int ret = 0;

        fprintf(stderr, "%s: Entry - Sending GSCAN Reset Passpoint List "
            "Request.\n", __func__);

        fprintf(stderr, "%s: Reset Passpoint List for request_id:%d.\n",
            __func__, id);
        ret = wifi_reset_passpoint_list(id, ifaceHandle);
        fprintf(stderr, "%s: Sending GSCAN Reset Passpoint Hotlist request"
            "completed. Returned value: %d.\n", __func__, ret);
    }

}
