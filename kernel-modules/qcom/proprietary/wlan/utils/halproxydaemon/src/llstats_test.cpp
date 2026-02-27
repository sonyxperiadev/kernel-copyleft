/*
* Copyright (c) 2014-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "llstats_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include "link_layer_stats.h"

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define UNUSED(x) (void)(x)

#ifdef WIFI_FEATURE_T2LM_NEGO
#define ANDROID_U_LL_STATS 1
#endif

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
namespace LLStats
{
    /* CLI cmd strings */
    const char *LLStatsTestSuite::LL_CMD = "llstats";
    const char *LLStatsTestSuite::LL_SET = "set";
    const char *LLStatsTestSuite::LL_GET = "get";
    const char *LLStatsTestSuite::LL_CLEAR = "clear";

    bool is_mlo;

    wifi_error LLStatsTestSuite:: wifi_set_link_stats_test(
                                    wifi_interface_handle iface,
                                    u32 mpduSizeThreshold,
                                    u32 aggressiveStatisticsGathering)
    {
        wifi_link_layer_params params;
        params.mpdu_size_threshold = mpduSizeThreshold;
        params.aggressive_statistics_gathering = aggressiveStatisticsGathering;
        wifi_error status;
        status = wifi_set_link_stats(iface, params);
        fprintf(stderr, "wifi_set_link_stats_test: mpdu size threshold: %d, "
            "aggressive statistics gathering: %d\n",
            mpduSizeThreshold,
            aggressiveStatisticsGathering);
        if (!status) {
           printf(" Status Success: %d in %s:%d\n", status, __func__, __LINE__);
        } else {
           printf(" Status Failed : %d in %s:%d\n", status, __func__, __LINE__);
        }
        return status;
    }

    void print_radio_stats(wifi_radio_stat *stats, int num_radios)
    {
        int i, j;

        for (i = 0; i < num_radios; i++) {
            printf("\n################ Radio Stats ########################\n");
            printf("radio :               %u\n", stats->radio);
            printf("on_time :             %u\n", stats->on_time);
            printf("tx_time :             %u\n", stats->tx_time);
            printf("rx_time :             %u\n", stats->rx_time);
            printf("on_time_scan :        %u\n", stats->on_time_scan);
            printf("on_time_nbd :         %u\n", stats->on_time_nbd);
            printf("on_time_gscan :       %u\n", stats->on_time_gscan);
            printf("on_time_roam_scan :   %u\n", stats->on_time_roam_scan);
            printf("on_time_pno_scan)     %u\n", stats->on_time_pno_scan);
            printf("on_time_hs20 :        %u\n", stats->on_time_hs20);
            printf("num_channels :        %u\n", stats->num_channels);
            printf("num_tx_levels :       %u\n", stats->num_tx_levels);

            for (j = 0; j < stats->num_channels; j++) {
                printf("\n");
                printf("center_freq  : %u\n", stats->channels[j].channel.center_freq);
                printf("width        : %u\n", stats->channels[j].channel.width);
                printf("center_freq0 : %u\n", stats->channels[j].channel.center_freq0);
                printf("center_freq1 : %u\n", stats->channels[j].channel.center_freq1);
                printf("on_time      : %u\n", stats->channels[j].on_time);
                printf("cca_busy_time: %u\n", stats->channels[j].cca_busy_time);
                printf("\n");
            }

            if (stats->tx_time_per_levels) {
                printf("Time spent in each transmission power level: \n");
                for (j = 0; j < stats->num_tx_levels; j++) {
                    printf("Power level : %u    tx_time : %u\n", j,
                            stats->tx_time_per_levels[j]);
                }
            }
            printf("\n#####################################################\n");
            stats = (wifi_radio_stat *)((u8 *)stats
                     + sizeof(wifi_radio_stat)
                     + stats->num_channels* sizeof(wifi_channel_stat));
        }
    }

    void print_interface_mode(wifi_interface_link_layer_info *info)
    {
        if (info) {
            if (info->mode == 0)
                printf("Interface Mode : %s\n","STA");
            else if (info->mode == 1)
                printf("Interface Mode : %s\n","SOFTAP");
            else if (info->mode == 2)
                printf("Interface Mode : %s\n","IBSS");
            else if (info->mode == 3)
                printf("Interface Mode : %s\n","P2P_CLIENT");
            else if (info->mode == 4)
                printf("Interface Mode : %s\n","P2P_GO");
            else if (info->mode == 5)
                printf("Interface Mode : %s\n","NAN");
            else if (info->mode == 6)
                printf("Interface Mode : %s\n","MESH");
            else
                printf("Interface Mode : %s\n","Unknown");

            printf("Interface Mac Address : %02x:%02x:%02x:%02x:%02x:%02x \n",
                    info->mac_addr[0], info->mac_addr[1],
                    info->mac_addr[2], info->mac_addr[3],
                    info->mac_addr[4], info->mac_addr[5]);

            if (info->state == 0)
                printf("Interface State : %s\n","DISCONNECTED");
            else if (info->state == 1)
                printf("Interface State : %s\n","AUTHENTICATING");
            else if (info->state == 2)
                printf("Interface State : %s\n","ASSOCIATING");
            else if (info->state == 3)
                printf("Interface State : %s\n","ASSOCIATED");

            printf("Roaming State  : %0d\n",info->roaming);
            printf("Interface Capabilities : %0x\n",
                   info->capabilities);
            printf("Interface SSID : %s\n",info->ssid);
            printf("Interface BSSID : %02x:%02x:%02x:%02x:%02x:%02x \n",
                    info->bssid[0], info->bssid[1],
                    info->bssid[2], info->bssid[3],
                    info->bssid[4], info->bssid[5]);
            printf("AP Country String : %c%c%c\n",
                    info->ap_country_str[0],
                    info->ap_country_str[1],
                    info->ap_country_str[2]);
            printf("Country String : %c%c%c\n",
                    info->country_str[0],
                    info->country_str[1],
                    info->country_str[2]);
        }
    }

    void print_wmm_stats(wifi_wmm_ac_stat *ac)
    {
        int k;

        for (k = 0; k < WIFI_AC_MAX; k++) {
            wifi_wmm_ac_stat *stat = &ac[k];
            printf("ac : %u \n", stat->ac);
            printf("txMpdu : %u \n", stat->tx_mpdu) ;
            printf("rxMpdu : %u \n", stat->rx_mpdu);
            printf("txMcast : %u \n", stat->tx_mcast);
            printf("rxMcast : %u \n", stat->rx_mcast);
            printf("rxAmpdu : %u \n", stat->rx_ampdu);
            printf("txAmpdu : %u \n", stat->tx_ampdu);
            printf("mpduLost : %u \n", stat->mpdu_lost);
            printf("retries : %u \n", stat->retries);
            printf("retriesShort : %u \n",
                    stat->retries_short);
            printf("retriesLong : %u \n",
                    stat->retries_long);
            printf("contentionTimeMin : %u \n",
                    stat->contention_time_min);
            printf("contentionTimeMax : %u \n",
                    stat->contention_time_max);
            printf("contentionTimeAvg : %u \n",
                    stat->contention_time_avg);
            printf("contentionNumSamples : %u \n\n",
                    stat->contention_num_samples);
        }
    }

    wifi_peer_info* print_peer_stats(wifi_peer_info *peer_info, u32 num_peers)
    {
        u32 j, k;
        wifi_rate_stat *rate_stat;

        if (peer_info) {
            for (k = 0; k < num_peers; k++) {
                switch (peer_info->type)
                {
                    case WIFI_PEER_STA:
                        printf("Peer Type : WIFI_PEER_STA\n");
                        break;
                    case WIFI_PEER_AP:
                        printf("Peer Type : WIFI_PEER_AP\n");
                        break;
                    case WIFI_PEER_P2P_GO:
                        printf("Peer Type : WIFI_PEER_P2P_GO\n");
                        break;
                    case WIFI_PEER_P2P_CLIENT:
                        printf("Peer Type : WIFI_PEER_P2P_CLIENT\n");
                        break;
                    case WIFI_PEER_NAN:
                        printf("Peer Type : WIFI_PEER_NAN\n");
                        break;
                    case WIFI_PEER_TDLS:
                        printf("Peer Type : WIFI_PEER_TDLS\n");
                        break;
                    case WIFI_PEER_INVALID:
                        printf("Peer Type : WIFI_PEER_INVALID\n");
                        break;
                    default:
                        printf("Invalid peer type value\n");
                }

                printf("peer mac address " MAC_ADDR_STR "\n",
                        MAC_ADDR_ARRAY(peer_info->peer_mac_address));
                printf("capability : %u\n", peer_info->capabilities);
                printf("Number of rates : %u\n\n", peer_info->num_rate);
                rate_stat = peer_info->rate_stats;

                printf("############# Rate stats: #################\n\n");
                for (j = 0; j < peer_info->num_rate; j++) {
                    printf("Rate Set  = %u\n", j);
                    printf("preamble  %u \n", rate_stat[j].rate.preamble);
                    printf("nss %u\n", rate_stat[j].rate.nss);
                    printf("bw %u\n", rate_stat[j].rate.bw);
                    printf("rateMcsIdx  %u\n", rate_stat[j].rate.rateMcsIdx);
                    printf("reserved %u\n", rate_stat[j].rate.reserved);
                    printf("bitrate %u\n", rate_stat[j].rate.bitrate);
                    printf("txMpdu %u\n", rate_stat[j].tx_mpdu);
                    printf("rxMpdu %u\n", rate_stat[j].rx_mpdu);
                    printf("mpduLost %u\n", rate_stat[j].mpdu_lost);
                    printf("retries %u\n", rate_stat[j].retries);
                    printf("retriesShort %u\n\n", rate_stat[j].retries_short);
                }
                peer_info = (wifi_peer_info *)((u8 *)peer_info + sizeof(wifi_peer_info) +
                             (peer_info->num_rate * sizeof(wifi_rate_stat)));
            }
        }
        return peer_info;
    }

    void link_stats_results_handler(wifi_request_id id,
                                    wifi_iface_stat *iface_stat,
                                    int num_radios,
                                    wifi_radio_stat *radio_stat)
    {
        printf("Stats Received for Request Id : %d\n", id);
        if (radio_stat)
            print_radio_stats(radio_stat, num_radios);

        if (iface_stat) {
            printf("\n################# Interface Stats ##################\n");

            print_interface_mode(&(iface_stat->info));

            printf("Beacon rx : %u\n", iface_stat->beacon_rx);
            printf("average_tsf_offset: %" PRIu64 "\n",
                    iface_stat->average_tsf_offset);
            printf("leaky_ap_detected: %u\n", iface_stat->leaky_ap_detected);
            printf("leaky_ap_avg_num_frames_leaked: %u\n",
                    iface_stat->leaky_ap_avg_num_frames_leaked);
            printf("leaky_ap_guard_time: %u\n",
                    iface_stat->leaky_ap_guard_time);
            printf("mgmt rx : %u\n", iface_stat->mgmt_rx);
            printf("mgmt_action_rx : %u\n", iface_stat->mgmt_action_rx);
            printf("mgmt_action_tx : %u\n", iface_stat->mgmt_action_tx);
            printf("rssi_mgmt : %d\n", iface_stat->rssi_mgmt);
            printf("rssi_data : %d\n", iface_stat->rssi_data);
            printf("rssi_ack : %d\n", iface_stat->rssi_ack);
            printf("num peers : %u\n\n", iface_stat->num_peers);

            printf("############# WMM Stats: #################\n\n");
            print_wmm_stats(iface_stat->ac);

            printf("############# Peer stats: #################\n\n");
            print_peer_stats(iface_stat->peer_info, iface_stat->num_peers);

            printf("Done \n");
        }
    }

#ifdef ANDROID_U_LL_STATS
    void ml_link_stats_results_handler(wifi_request_id id,
                                    wifi_iface_ml_stat *iface_ml_stat,
                                    int num_radios,
                                    wifi_radio_stat *radio_stat)
    {
        u32 link = 0;
        is_mlo = true;

        printf("Stats Received for Request Id : %d\n", id);
        if (radio_stat)
            print_radio_stats(radio_stat, num_radios);

        if (iface_ml_stat) {
            wifi_link_stat *link_stat;

            printf("\n################# Interface Stats ##################\n");

            print_interface_mode(&(iface_ml_stat->info));

            link_stat = iface_ml_stat->links;
            printf("Num links : %u\n", iface_ml_stat->num_links);
            for (link = 0; link < iface_ml_stat->num_links; link++) {
                wifi_peer_info *peer_info;

                printf("\n################# Link Stats ##################\n");
                printf("Link number : %u\n", link);
                printf("Link id : %u\n", link_stat->link_id);
                printf("Link state : %u\n", link_stat->state);
                printf("radio id : %u\n", link_stat->radio);
                printf("Frequency : %u\n", link_stat->frequency);
                printf("Beacon rx : %u\n", link_stat->beacon_rx);
                printf("average_tsf_offset: %" PRIu64 "\n",
                        link_stat->average_tsf_offset);
                printf("leaky_ap_detected: %u\n", link_stat->leaky_ap_detected);
                printf("leaky_ap_avg_num_frames_leaked: %u\n",
                        link_stat->leaky_ap_avg_num_frames_leaked);
                printf("leaky_ap_guard_time: %u\n",
                        link_stat->leaky_ap_guard_time);
                printf("mgmt rx : %u\n", link_stat->mgmt_rx);
                printf("mgmt_action_rx : %u\n", link_stat->mgmt_action_rx);
                printf("mgmt_action_tx : %u\n", link_stat->mgmt_action_tx);
                printf("rssi_mgmt : %d\n", link_stat->rssi_mgmt);
                printf("rssi_data : %d\n", link_stat->rssi_data);
                printf("rssi_ack : %d\n", link_stat->rssi_ack);
                printf("num peers : %u\n\n", link_stat->num_peers);

                printf("############# WMM Stats: #################\n\n");
                printf("Link number : %u\n", link);
                print_wmm_stats(link_stat->ac);

                printf("############# Peer stats: #################\n\n");
                printf("Link number : %u\n", link);
                peer_info = print_peer_stats(link_stat->peer_info, link_stat->num_peers);
                link_stat = (wifi_link_stat *)peer_info;
            }
            printf("Done \n");
        }
    }

    void link_stats_legacy_results_handler(wifi_request_id id,
                                           wifi_iface_stat *iface_stat,
                                           int num_radios,
                                           wifi_radio_stat *radio_stat)
    {
        printf("Stats Received for Request Id : %d\n", id);

        UNUSED(num_radios);
        UNUSED(radio_stat);

        if (iface_stat) {
            printf("\n############## Legacy Interface Stats ###############\n");

            printf("Beacon rx : %u\n", iface_stat->beacon_rx);
            printf("average_tsf_offset: %" PRIu64 "\n",
                    iface_stat->average_tsf_offset);
            printf("leaky_ap_detected: %u\n", iface_stat->leaky_ap_detected);
            printf("leaky_ap_avg_num_frames_leaked: %u\n",
                    iface_stat->leaky_ap_avg_num_frames_leaked);
            printf("leaky_ap_guard_time: %u\n",
                    iface_stat->leaky_ap_guard_time);
            printf("mgmt rx : %u\n", iface_stat->mgmt_rx);
            printf("mgmt_action_rx : %u\n", iface_stat->mgmt_action_rx);
            printf("mgmt_action_tx : %u\n", iface_stat->mgmt_action_tx);
            printf("rssi_mgmt : %d\n", iface_stat->rssi_mgmt);
            printf("rssi_data : %d\n", iface_stat->rssi_data);
            printf("rssi_ack : %d\n", iface_stat->rssi_ack);
            printf("num peers : %u\n\n", iface_stat->num_peers);

            printf("############# WMM Stats: #################\n\n");
            print_wmm_stats(iface_stat->ac);

            printf("Done\n");
        }
    }
#endif

        /* Helper routine to initiialize the Link Layer Stats */
    wifi_error LLStatsTestSuite:: wifi_get_link_stats_test(
                                    wifi_interface_handle iface)
    {
        wifi_error status;
        wifi_stats_result_handler handler;
        is_mlo = false;

        handler.on_link_stats_results = link_stats_results_handler;
#ifdef ANDROID_U_LL_STATS
        handler.on_multi_link_stats_results = ml_link_stats_results_handler;
#endif
        status = wifi_get_link_stats(1, iface, handler);
        if (!status) {
            printf(" Status Success: %d in %s:%d\n", status, __func__, __LINE__);
        } else {
            printf(" Status Failed : %d in %s:%d\n", status, __func__, __LINE__);
        }

#ifdef ANDROID_U_LL_STATS
        if (!is_mlo)
            return status;

        handler.on_multi_link_stats_results = NULL;
        handler.on_link_stats_results = link_stats_legacy_results_handler;

        status = wifi_get_link_stats(1, iface, handler);
        if (!status) {
            printf(" Legacy Status Success: %d in %s:%d\n", status, __func__, __LINE__);
        } else {
            printf(" Legacy Status Failed : %d in %s:%d\n", status, __func__, __LINE__);
        }
#endif
        return status;
    }

    /*
     * WIFI_STATS_RADIO              0x00000001       all radio statistics
     * WIFI_STATS_RADIO_CCA          0x00000002       cca_busy_time (within radio statistics)
     * WIFI_STATS_RADIO_CHANNELS     0x00000004       all channel statistics (within radio statistics)
     * WIFI_STATS_RADIO_SCAN         0x00000008       all scan statistics (within radio statistics)
     * WIFI_STATS_IFACE              0x00000010       all interface statistics
     * WIFI_STATS_IFACE_TXRATE       0x00000020       all tx rate statistics (within interface statistics)
     * WIFI_STATS_IFACE_AC           0x00000040       all ac statistics (within interface statistics)
     * WIFI_STATS_IFACE_CONTENTION   0x00000080       all contention (min, max, avg) statistics (within ac statisctics)
     */
    /* Helper routine to initiialize the Link Layer statistics Handlers */
    wifi_error LLStatsTestSuite:: wifi_clr_link_stats_test (
                                    wifi_interface_handle iface,
                                    u32 stats_clear_req_mask,
                                    u8 stop_req)
    {
        u32 stats_clear_rsp_mask = 0;
        u8  stop_rsp = 0;
        wifi_error status;

        status = wifi_clear_link_stats(iface, stats_clear_req_mask,
                                &stats_clear_rsp_mask, stop_req, &stop_rsp);
        if (!status) {
           printf(" Status Success\n");
           printf(" stats_clear_req_mask : %d, stats_clear_rsp_mask : %d,"
                   "stop_req : %d, stop_rsp : %d\n", stats_clear_req_mask,
                   stats_clear_rsp_mask, stop_req, stop_rsp);
        } else
           printf(" Status Failed\n");
        return status;
    }
    /* Constructor */
    LLStatsTestSuite::LLStatsTestSuite(wifi_handle handle)
        :wifiHandle_(handle)
    {

    }

    /* Constructor */
    void LLStatsTestSuite::Usage()
    {
        fprintf(stderr, "Usage:\n");
        fprintf(stderr, "$hal_proxy_daemon llstats iface_name set"
                " <mpdu_size_threshold> <aggressive_statistics_gathering>\n");
        fprintf(stderr, "Ex: $hal_proxy_daemon llstats wlan0 set 100 1\n\n");
        fprintf(stderr, "$hal_proxy_daemon llstats iface_name get\n");
        fprintf(stderr, "Ex: $hal_proxy_daemon llstats wlan0 get\n\n");
        fprintf(stderr, "$hal_proxy_daemon llstats iface_name clear"
                " <stats_clear_req_mask(in decimal)> <stop_req>\n");
        fprintf(stderr, "Ex: $hal_proxy_daemon llstats wlan0 clear 255 1\n\n");
    }

    /* process the command line args */
    wifi_error LLStatsTestSuite::processCmd(int argc, char **argv)
    {
        if (argc < 4) {
            Usage();
            fprintf(stderr, "%s: insufficient LL Stats args\n", argv[0]);
            return WIFI_ERROR_INVALID_ARGS;
        }

        if (strcasecmp(argv[3], LL_SET) == 0) {
            if (argc < 6) {
                fprintf(stderr, "%s: insufficient args for LL Stats set\n",
                        __func__);
                fprintf(stderr, "Usage : hal_proxy_daemon llstats iface_name"
                        " set <MPDU Size Threshold>"
                        " <Aggressive Statistics Gathering>\n");
                return WIFI_ERROR_INVALID_ARGS;
            }

            //TODO : Take the Interface name as an argument
            ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[2]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
                return WIFI_ERROR_INVALID_ARGS;
            }

            return  wifi_set_link_stats_test(ifaceHandle, atoi(argv[4]), atoi(argv[5]));
        }

        if (strcasecmp(argv[3], LL_GET) == 0) {

            //TODO : Take the Interface name as an argument
            ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[2]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
                return WIFI_ERROR_INVALID_ARGS;
            }
            return wifi_get_link_stats_test(ifaceHandle);
        }

        if (strcasecmp(argv[3], LL_CLEAR) == 0) {
            if (argc < 6) {
                fprintf(stderr, "%s: insufficient args for LL Stats clear\n",
                        __func__);
                fprintf(stderr, "Usage : hal_proxy_daemon llstats"
                        " iface_name clear clr_req_mask stop_req\n");
                return WIFI_ERROR_INVALID_ARGS;
            }

            ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[2]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
                return WIFI_ERROR_INVALID_ARGS;
            }

            fprintf(stderr, "%s: calling wifi_clr_link_stats_test with "
                "req_mask:0x%x and stop_req:%d\n", __func__,
                (u32)strtoul(argv[4], NULL, 0), (u8)atoi(argv[5]));
            return wifi_clr_link_stats_test(ifaceHandle,
                strtoul(argv[4], NULL, 0), (u8)atoi(argv[5]));
        }
        fprintf(stderr, "%s: unknown cmd %s\n", argv[0], argv[3]);
        fprintf(stderr,
            "USAGE: ex: \n%s llstats wlan0 set <MPDU Size Threshold>"
            " <Aggressive Statistics Gathering> \n", argv[0]);
        return WIFI_ERROR_NOT_SUPPORTED;
    }

}
