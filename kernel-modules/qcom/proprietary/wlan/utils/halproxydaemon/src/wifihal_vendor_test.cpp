/*
 * Copyright (c) 2017-2018, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include "nud_stats.h"
#include "wifihal_vendor_test.hpp"

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);
namespace NUDStats
{
    /* CLI cmd strings */
    const char *NUDStatsTestSuite::NUD_CMD = "nud_stats";
    const char *NUDStatsTestSuite::NUD_SET = "set";
    const char *NUDStatsTestSuite::NUD_GET = "get";
    const char *NUDStatsTestSuite::NUD_CLEAR = "clear";

    /* Constructor */
    NUDStatsTestSuite::NUDStatsTestSuite(wifi_handle handle)
        :wifiHandle_(handle)
    {

    }

    /* Constructor */
    void NUDStatsTestSuite::Usage()
    {
        fprintf(stderr, "Usage:\n");
        fprintf(stderr, "$hal_proxy_daemon nud_stats iface_name set"
                " <gateway_ipv4addr> <packet_type> <domain_name>"
                " <src_port> <dst_port> <ipv4_addr> <ipv6_addr> \n");
        fprintf(stderr, "Ex: $hal_proxy_daemon nud_stats wlan0 set 192.168.1.1"
                        " 15 www.google.com 256 36115 192.168.43.1 0:0:0:0:0:0:0:0\n\n");
        fprintf(stderr, "$hal_proxy_daemon nud_stats iface_name get\n");
        fprintf(stderr, "Ex: $hal_proxy_daemon nud_stats wlan0 get\n\n");
        fprintf(stderr, "$hal_proxy_daemon nud_stats iface_name clear <packet_type>\n");
        fprintf(stderr, "Ex: $hal_proxy_daemon nud_stats wlan0 clear 15\n\n");
    }

    const char *print_packet_type(u32 pkt_type)
    {
         if (pkt_type & WLAN_NUD_STATS_DATA_ARP)
             return "ARP_DATA_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_DNS)
             return "DNS_DATA_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_TCP_HANDSHAKE)
             return "TCP_HANDSHAKE_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_ICMPV4)
             return "ICMPV4_DATA_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_ICMPV6)
             return "ICMPV6_DATA_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_TCP_SYN)
             return "TCP_SYN_DATA_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_TCP_SYN_ACK)
             return "TCP_SYN_ACK_DATA_PACKET";
         else if (pkt_type & WLAN_NUD_STATS_DATA_TCP_ACK)
             return "TCP_ACK_DATA_PACKET";
         else
             return "UNKNOWN_DATA_PACKET";
    }

    void pkt_stats_results_handler(nud_stats *stats,
                                   int num_stats,
                                   cmdData *pkt_stats)
    {
         cmdData *pktstats = (cmdData *) pkt_stats;
         int i=0;
         char ipv6_address[INET6_ADDRSTRLEN];
         char ipv4_address[INET_ADDRSTRLEN];

         fprintf(stderr, "NUD stats\n");
         fprintf(stderr, "arp_req_count_from_netdev: %d\n",
                 stats->arp_req_count_from_netdev);
         fprintf(stderr, "arp_req_count_to_lower_mac: %d\n",
                 stats->arp_req_count_to_lower_mac);
         fprintf(stderr, "arp_req_rx_count_by_lower_mac: %d\n",
                 stats->arp_req_rx_count_by_lower_mac);
         fprintf(stderr, "arp_req_count_tx_success: %d\n",
                 stats->arp_req_count_tx_success);
         fprintf(stderr, "arp_rsp_rx_count_by_lower_mac: %d\n",
                 stats->arp_rsp_rx_count_by_lower_mac);
         fprintf(stderr, "arp_rsp_rx_count_by_upper_mac: %d\n",
                 stats->arp_rsp_rx_count_by_upper_mac);
         fprintf(stderr, "arp_rsp_count_to_netdev: %d\n",
                 stats->arp_rsp_count_to_netdev);
         fprintf(stderr, "arp_rsp_count_out_of_order_drop: %d\n",
                 stats->arp_rsp_count_out_of_order_drop);
         fprintf(stderr, "ap_link_active: %s\n",
                 stats->ap_link_active?"Yes": "No");
         fprintf(stderr, "is_duplicate_addr_detection: %s\n",
                 stats->is_duplicate_addr_detection?"Yes": "No");
         fprintf(stderr, "PKT stats\n");

         if(pktstats) {
            for(i=0; i < num_stats;i++) {
                if (inet_ntop(AF_INET6, pktstats->ipv6_addr, ipv6_address,
                              INET6_ADDRSTRLEN) == NULL) {
                    fprintf(stderr, "%s: failed to convert ipv6 address format",
                            __FUNCTION__);
                }
                if (inet_ntop(AF_INET, (u8 *)&pktstats->ipv4_addr.s_addr, ipv4_address,
                              INET_ADDRSTRLEN) == NULL) {
                    fprintf(stderr, "%s: failed to convert ipv4 address format",
                            __FUNCTION__);
                }
                printf("\n################ Packet Stats ########################\n");
                printf(" pkt_type: %s domain_name: %s"
                       " src_port: %d dst_port: %d"
                       " ipv4_address: %s ipv6_address: %s\n"
                       " req_from_netdev: %d count_to_lower: %d"
                       " count_by_lower: %d count_tx_succ: %d"
                       " rsp_count_lower: %d rsp_count_upper: %d"
                       " rsp_count_netdev: %d out_of_order_drop: %d \n",
                       print_packet_type(pktstats->pkt_Type), pktstats->domain_name,
                       pktstats->src_port, pktstats->dst_port,
                       ipv4_address, ipv6_address,
                       pktstats->stats.pkt_req_count_from_netdev,
                       pktstats->stats.pkt_req_count_to_lower_mac,
                       pktstats->stats.pkt_req_rx_count_by_lower_mac,
                       pktstats->stats.pkt_req_count_tx_success,
                       pktstats->stats.pkt_rsp_rx_count_by_lower_mac,
                       pktstats->stats.pkt_rsp_rx_count_by_upper_mac,
                       pktstats->stats.pkt_rsp_count_to_netdev,
                       pktstats->stats.pkt_rsp_count_out_of_order_drop);
                 pktstats = (cmdData *)((u8 *)pktstats + sizeof(cmdData));
            }
         }
    }

    /* process the command line args */
    wifi_error NUDStatsTestSuite::processCmd(int argc, char **argv)
    {
        if (argc < 4) {
            Usage();
            fprintf(stderr, "%s: insufficient NUD Stats args\n", argv[0]);
            return WIFI_ERROR_INVALID_ARGS;
        }

        if (strcasecmp(argv[3], NUD_SET) == 0) {
            cmdData set_data;
            memset(&set_data,0,sizeof(cmdData));
            if (argc < 5 ) {
                fprintf(stderr, "%s: insufficient args for NUD Stats set\n",
                        __func__);
                fprintf(stderr, "Usage : hal_proxy_daemon NUDStats iface_name"
                        " set <IPv4 address of Default Gateway>"
                        " <packet_type> <domain_name> <src_port_no>"
                        " <dst_port_no> <ipv4_address> <ipv6_addres> \n");
                return WIFI_ERROR_INVALID_ARGS;
            }

            //TODO : Take the Interface name as an argument
            ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[2]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
                return WIFI_ERROR_INVALID_ARGS;
            }

            if (inet_aton(argv[4], &setip) == 0) {
                fprintf(stderr, "Invalid gw_addr :%s\n", argv[4]);
                return WIFI_ERROR_INVALID_ARGS;
            }
            if (argc < 11 ) {
                fprintf(stderr, "No Packet stats tracking\n");
            } else {
                set_data.pkt_Type = strtoul(argv[5], NULL, 0);
                set_data.domain_name = argv[6];
                set_data.src_port = strtoul(argv[7], NULL, 0);
                set_data.dst_port = strtoul(argv[8], NULL, 0);

                if (inet_aton(argv[9], &set_data.ipv4_addr) == 0) {
                    fprintf(stderr, "Invalid ipv4_addr :%s\n", argv[9]);
                }

                if (inet_pton(AF_INET6, argv[10], set_data.ipv6_addr) == 0) {
                    fprintf(stderr, "Invalid ipv6_addr :%s\n", argv[10]);
                    return WIFI_ERROR_INVALID_ARGS;
                }
            }
            return  wifi_set_nud_stats(ifaceHandle, setip.s_addr, set_data);
        }

        if (strcasecmp(argv[3], NUD_GET) == 0) {
            wifi_error ret;
            pkt_stats_result_handler handler;
            handler.on_pkt_stats_results = pkt_stats_results_handler;
            //TODO : Take the Interface name as an argument
            ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[2]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
                return WIFI_ERROR_INVALID_ARGS;
            }
            ret = wifi_get_nud_stats(ifaceHandle,handler);

            if (ret < 0) {
                fprintf(stderr, "Failed to get stats\n");
                return ret;
            }
	    return ret;
        }

        if (strcasecmp(argv[3], NUD_CLEAR) == 0) {
            cmdData clear_data;
            memset(&clear_data,0,sizeof(cmdData));

            ifaceHandle = wifi_get_iface_handle(wifiHandle_, argv[2]);
            if(!ifaceHandle)
            {
                fprintf(stderr, "Interface %s is not up, exiting.\n", argv[2]);
                return WIFI_ERROR_INVALID_ARGS;
            }
            if (argc > 4) {
                clear_data.pkt_Type = strtoul(argv[4], NULL, 0);
            }

            return wifi_clear_nud_stats(ifaceHandle,clear_data);
        }
        fprintf(stderr, "%s: unknown cmd %s\n", argv[0], argv[3]);
        Usage();
        return WIFI_ERROR_NOT_SUPPORTED;
    }

}
