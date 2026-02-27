/*
* Copyright (c) 2014, 2016, 2018, 2019 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "rtt_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include<sys/socket.h>
#include "common.hpp"
#include <vector>

extern struct sockaddr_in si;
extern int app_sock, slen;
extern int glb_total_iter,glb_curr_iter,glb_sleep_time;
extern std::vector <int> glb_distance_mm;


#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"


void PrintWifiInfoElement(wifi_information_element *elem)
{
    if (elem) {
        char buf[512];
        int index;
        char* ptr;
        int pos;

        memset(buf, 0, sizeof(buf));
        ptr = (char*)elem->data;
        pos = 0;
        for (index=0; index < (int)elem->len; index++)
        {
            pos += snprintf(&(buf[pos]), sizeof(buf) - pos, "%02x ",
                            *ptr++);
            if (pos > 508)
            {
                break;
            }
        }

        fprintf(stderr,"id=[%d]\n", elem->id);
        fprintf(stderr,"HEX DUMP len=[%d]\n", elem->len);
        fprintf(stderr,"buf:%s\n", buf);
    }
    else {
        fprintf(stderr,"Wifi Information Element NULL\n");
    }
}


static int computeMedian(std::vector <int> &rtt_array)
{
  int array_size = rtt_array.size();

  std::sort(rtt_array.begin(), rtt_array.end());

  if((array_size%2) != 0)
  {
    /* odd array length, middle position will be median */
    return rtt_array[array_size/2];
  }
  else
  {
    /*
     * even array lenght median will be
     * average sum of two middle index values
     */
    return ((rtt_array[array_size/2]+rtt_array[(array_size/2)-1])/2);
  }
}



static void rtt_median_on_results(wifi_request_id /*id*/, unsigned num_results,
                           wifi_rtt_result* rtt_result[])
{

    wifi_rtt_result *result = NULL;
    int median = 0;

    if (rtt_result) {
        for(u32 i = 0; i < num_results; i++)
        {
            result = rtt_result[i];
            glb_distance_mm.push_back(result->distance_mm);
            if (glb_total_iter == glb_curr_iter) {

                median = computeMedian(glb_distance_mm);

                fprintf(stderr,"\n******************  DONE ******************\n");
                fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                        result->addr[0], result->addr[1], result->addr[2],
                        result->addr[3], result->addr[4], result->addr[5]);

                fprintf(stderr, "Estimated distance: %.2f m\n",(float)median/1000);
                fprintf(stderr,"*************************************************\n");
                glb_distance_mm.clear();
                glb_total_iter = 0;
                glb_curr_iter = 0;

            } else {
                fprintf(stderr, ".");
                glb_curr_iter++;
            }
        }
    }
}

static void rtt_on_results(wifi_request_id id, unsigned num_results,
                           wifi_rtt_result* rtt_result[])
{
    fprintf(stderr, "**********************************************\n.");
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n.",
        __func__, id, num_results);
    wifi_rtt_result *result = NULL;
    if (rtt_result) {
        for(u32 i = 0; i < num_results; i++)
        {
            result = rtt_result[i];
            if (result) {
                fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                        result->addr[0], result->addr[1], result->addr[2],
                        result->addr[3], result->addr[4], result->addr[5]);
                fprintf(stderr, "burst_num  %d \n", result->burst_num);
                fprintf(stderr, "measurement_number  %d \n", result->measurement_number);
                fprintf(stderr, "success_number  %d \n", result->success_number);
                fprintf(stderr, "number_per_burst_peer  %d \n", result->number_per_burst_peer);
                fprintf(stderr, "status:  %u \n", result->status);
                fprintf(stderr, "retry_after_duration  %d \n", result->retry_after_duration);
                fprintf(stderr, "type:  %u \n", result->type);
                fprintf(stderr, "rssi:  %d \n", result->rssi);
                fprintf(stderr, "rssi_spread:  %d \n", result->rssi_spread);
                fprintf(stderr, "tx_rate:preamble(%02x), nss(%d), bw(%d)"
                                "rateMcsIdx(%d) bitrate(%d Kbps)\n",
                        result->tx_rate.preamble, result->tx_rate.nss, result->tx_rate.bw,
                        result->tx_rate.rateMcsIdx, result->tx_rate.bitrate);
                fprintf(stderr, "rx_rate:preamble(%02x), nss(%d), bw(%d)"
                                "rateMcsIdx(%d) bitrate(%d Kbps)\n",
                        result->rx_rate.preamble, result->rx_rate.nss, result->rx_rate.bw,
                        result->rx_rate.rateMcsIdx, result->rx_rate.bitrate);
                fprintf(stderr, "rtt  %" PRId64 "\n", result->rtt);
                fprintf(stderr, "rtt_sd  %" PRId64 "\n", result->rtt_sd);
                fprintf(stderr, "rtt_spread  %" PRId64 "\n",
                        result->rtt_spread);
                fprintf(stderr, "distance  %u \n", result->distance_mm);
                fprintf(stderr, "distance_sd  %u \n", result->distance_sd_mm);
                fprintf(stderr, "distance_spread  %d\n", result->distance_spread_mm);
                fprintf(stderr, "ts  %" PRId64 "\n", result->ts);
                fprintf(stderr, "burst_duration  %d \n", result->burst_duration);
                fprintf(stderr, "negotiated_burst_num  %d \n", result->negotiated_burst_num);
                fprintf(stderr, "***********LCI Wifi Element***********\n");
                PrintWifiInfoElement(result->LCI);
                fprintf(stderr, "***********LCR Wifi Element***********\n");
                PrintWifiInfoElement(result->LCR);
            }
        }
    }
    fprintf(stderr, "*************************************************\n.");
}

#ifdef CONFIG_ANDROID_14
static void rtt_median_on_results_v2(wifi_request_id /*id*/, unsigned num_results,
                           wifi_rtt_result_v2* rtt_result[])
{

    wifi_rtt_result *result = NULL;
    int median = 0;

    if (rtt_result) {
        for(u32 i = 0; i < num_results; i++)
        {
            fprintf(stderr, "Frequency %u\n", rtt_result[i]->frequency);
            fprintf(stderr, "Packet Bandwidth %u\n", rtt_result[i]->packet_bw);
            result = &rtt_result[i]->rtt_result;
            glb_distance_mm.push_back(result->distance_mm);
            if (glb_total_iter == glb_curr_iter) {

                median = computeMedian(glb_distance_mm);

                fprintf(stderr,"\n******************  DONE ******************\n");
                fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                        result->addr[0], result->addr[1], result->addr[2],
                        result->addr[3], result->addr[4], result->addr[5]);

                fprintf(stderr, "Estimated distance: %.2f m\n",(float)median/1000);
                fprintf(stderr,"*************************************************\n");
                glb_distance_mm.clear();
                glb_total_iter = 0;
                glb_curr_iter = 0;

            } else {
                fprintf(stderr, ".");
                glb_curr_iter++;
            }
        }
    }
}

static void  rtt_on_results_v2(wifi_request_id id, unsigned num_results,
                           wifi_rtt_result_v2* rtt_result[])
{
    fprintf(stderr, "**********************************************\n.");
    fprintf(stderr, "%s: request_id:%d, num_results:%d\n.",
        __func__, id, num_results);
    wifi_rtt_result *result = NULL;
    if (rtt_result) {
        for(u32 i = 0; i < num_results; i++)
        {
            result = &rtt_result[i]->rtt_result;
            if (result) {
                fprintf(stderr, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x \n",
                        result->addr[0], result->addr[1], result->addr[2],
                        result->addr[3], result->addr[4], result->addr[5]);
                fprintf(stderr, "burst_num  %d \n", result->burst_num);
                fprintf(stderr, "measurement_number  %d \n", result->measurement_number);
                fprintf(stderr, "success_number  %d \n", result->success_number);
                fprintf(stderr, "number_per_burst_peer  %d \n", result->number_per_burst_peer);
                fprintf(stderr, "status:  %u \n", result->status);
                fprintf(stderr, "retry_after_duration  %d \n", result->retry_after_duration);
                fprintf(stderr, "type:  %u \n", result->type);
                fprintf(stderr, "rssi:  %d \n", result->rssi);
                fprintf(stderr, "rssi_spread:  %d \n", result->rssi_spread);
                fprintf(stderr, "tx_rate:preamble(%02x), nss(%d), bw(%d)"
                                "rateMcsIdx(%d) bitrate(%d Kbps)\n",
                        result->tx_rate.preamble, result->tx_rate.nss, result->tx_rate.bw,
                        result->tx_rate.rateMcsIdx, result->tx_rate.bitrate);
                fprintf(stderr, "rx_rate:preamble(%02x), nss(%d), bw(%d)"
                                "rateMcsIdx(%d) bitrate(%d Kbps)\n",
                        result->rx_rate.preamble, result->rx_rate.nss, result->rx_rate.bw,
                        result->rx_rate.rateMcsIdx, result->rx_rate.bitrate);
                fprintf(stderr, "rtt  %" PRId64 "\n", result->rtt);
                fprintf(stderr, "rtt_sd  %" PRId64 "\n", result->rtt_sd);
                fprintf(stderr, "rtt_spread  %" PRId64 "\n",
                        result->rtt_spread);
                fprintf(stderr, "distance  %u \n", result->distance_mm);
                fprintf(stderr, "distance_sd  %u \n", result->distance_sd_mm);
                fprintf(stderr, "distance_spread  %d\n", result->distance_spread_mm);
                fprintf(stderr, "ts  %" PRId64 "\n", result->ts);
                fprintf(stderr, "burst_duration  %d \n", result->burst_duration);
                fprintf(stderr, "negotiated_burst_num  %d \n", result->negotiated_burst_num);
                fprintf(stderr, "***********LCI Wifi Element***********\n");
                PrintWifiInfoElement(result->LCI);
                fprintf(stderr, "***********LCR Wifi Element***********\n");
                PrintWifiInfoElement(result->LCR);
            }
            fprintf(stderr, "Frequency %u\n", rtt_result[i]->frequency);
            fprintf(stderr, "Packet Bandwidth %u\n", rtt_result[i]->packet_bw);
        }
    }
    fprintf(stderr, "*************************************************\n.");
}
#endif

namespace RTT_TEST
{
    /* CLI cmd strings */
    const char *RttTestSuite::RTT_CMD = "rtt";
    const char *RttTestSuite::RTTM_CMD = "rttm";
    const char *RttTestSuite::RTT_RANGE_REQUEST = "range_request";
    const char *RttTestSuite::RTT_RANGE_CANCEL = "range_cancel";
    const char *RttTestSuite::RTT_GET_CAPABILITIES = "get_capabilities";
    const char *RttTestSuite::RTT_SET_LCI = "set_lci";
    const char *RttTestSuite::RTT_SET_LCR = "set_lcr";
    const char *RttTestSuite::RTT_GET_RESPONDER_INFO = "get_rsp_info";
    const char *RttTestSuite::RTT_ENABLE_RESPONDER = "enable_rsp";
    const char *RttTestSuite::RTT_DISABLE_RESPONDER = "disable_rsp";
    int ret;

    /* Constructor */
    RttTestSuite::RttTestSuite(wifi_interface_handle handle, wifi_request_id request_id)
        :wifiHandle_(handle)
    {
        fprintf(stderr, "RttTestSuite::RttTestSuite: Created a Rtt Test "
            "Suite with request_id:%d\n.", request_id);
        id = request_id;
    }

    void RttTestSuite::setRequestId(int reqId)
    {
        id = reqId;
    }

    int RttTestSuite::getRequestId()
    {
        return id;
    }

    /* process the command line args */
    void RttTestSuite::executeCmd(int argc,
                                      char **argv,
                                      int cmdIndex)
    {
        fprintf(stderr, "%s: Enter \n", __func__);
        switch(cmdIndex) {
            case 1:
                rttSendRangeRequest(argc, argv);
                break;
            case 2:
                rttSendCancelRangeRequest(argc, argv);
                break;
            case 3:
                rttSendGetCapabilitiesRequest(argc, argv);
                break;
           case 4:
               rttSetLCI(argc, argv);
               break;
           case 5:
               rttSetLCR(argc, argv);
               break;
           case 6:
               rttGetResponderInfo(argc, argv);
               break;
           case 7:
               rttEnableResponder(argc, argv);
               break;
           case 8:
               rttDisableResponder(argc, argv);
               break;
            default:
                fprintf(stderr, "%s: Unknown Cmd ID.\n", __func__);
        }
    }

    /* process the command line args */
    void RttTestSuite::processCmd(int argc, char **argv)
    {
        if(argc <3)
        {
            fprintf(stderr, "%s: insufficient rtt args\n", argv[0]);
            return;
        }

        if(strcasecmp(argv[2], RTT_RANGE_REQUEST) == 0)
            return rttSendRangeRequest(argc, argv);

        if(strcasecmp(argv[2], RTT_RANGE_CANCEL) == 0)
            return rttSendCancelRangeRequest(argc, argv);

        if(strcasecmp(argv[2], RTT_GET_CAPABILITIES) == 0)
            return rttSendGetCapabilitiesRequest(argc, argv);

        if(strcasecmp(argv[2], RTT_SET_LCI) == 0)
            return rttSetLCI(argc, argv);

        if(strcasecmp(argv[2], RTT_SET_LCR) == 0)
            return rttSetLCR(argc, argv);

        if(strcasecmp(argv[2], RTT_GET_RESPONDER_INFO) == 0)
            return rttGetResponderInfo(argc, argv);

        if(strcasecmp(argv[2], RTT_ENABLE_RESPONDER) == 0)
            return rttEnableResponder(argc, argv);

        if(strcasecmp(argv[2], RTT_DISABLE_RESPONDER) == 0)
            return rttDisableResponder(argc, argv);

        fprintf(stderr, "%s: unknown arg %s\n", argv[0], argv[2]);
    }

    /* Helper routine to print usage */
    void RttTestSuite::rttPrintCmdUsage(char **argv, const char *cmd,
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

    void RttTestSuite::rttSendGetCapabilitiesRequest(int argc, char **argv)
    {
        int ret = 0;
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        wifi_rtt_capabilities capa;
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    rttPrintCmdUsage(argv, RTT_CMD, RTT_GET_CAPABILITIES,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "%s: Sending Get Capabilities Request. \n",
            __FUNCTION__);
        ret = wifi_get_rtt_capabilities(wifiHandle_, &capa);
        fprintf(stderr, "%s: Received RTT Capabilities return val:%d. \n",
            __FUNCTION__, ret);
        if (ret)
            return;

        fprintf(stderr, "%s: Capabilities:\n"
                        "rtt_one_sided_supported:%01x\n"
                        "rtt_ftm_supported:%01x\n"
                        "lci_support:%02x\n"
                        "lcr_support:%02x\n"
                        "preamble_support:%02x\n"
                        "bw_support:%02x\n"
                        "responder_supported:%02x\n"
                        "mc_version:%d\n",
                        __func__,
                        capa.rtt_one_sided_supported,
                        capa.rtt_ftm_supported,
                        capa.lci_support,
                        capa.lcr_support,
                        capa.preamble_support,
                        capa.bw_support,
                        capa.responder_supported,
                        capa.mc_version);
    }

    void RttTestSuite::rttSendRangeRequest(int argc, char **argv)
    {
        fprintf(stderr, "%s: Sending RTT Range Request. "
            "\n", __func__);
        wifi_rtt_event_handler nHandler;
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
                    rttPrintCmdUsage(argv, RTT_CMD,
                        RTT_RANGE_REQUEST,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        unsigned num_rtt_config;
        wifi_rtt_config rtt_config[1000];
        u32 dummyIndex;

        memset(rtt_config, 0, sizeof(rtt_config));
        FILE *f_read = fopen("/vendor/etc/wifi/rtt_range_request_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if ( (fscanf(f_read, "%d", &num_rtt_config)) == 0 ) {
                fprintf(stderr, "%s: Failed parsing RTT Range Request "
                    "params file. Exit\n", __func__);
                fclose(f_read);
                return;
            }
            fprintf(stderr, "Number of RTT Config records:%d\n\n",
                    num_rtt_config);
            if (num_rtt_config > (sizeof(rtt_config)/sizeof(rtt_config[0]))) {
                fprintf(stderr, "Max Number of records suppored is %zu\n",
                        (sizeof(rtt_config)/sizeof(wifi_rtt_config)));
                fclose(f_read);
                return;
            }

            /* Outer loop for parsing scan buckets */
            for ( i = 0; i < (int)num_rtt_config; i++ )
            {
                if (
                    (fscanf(f_read, "%d", &dummyIndex) &&
                    fscanf(f_read, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                            &rtt_config[i].addr[0], &rtt_config[i].addr[1],
                            &rtt_config[i].addr[2], &rtt_config[i].addr[3],
                            &rtt_config[i].addr[4], &rtt_config[i].addr[5]) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].type) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].peer) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].channel.width) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].channel.center_freq) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].channel.center_freq0) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].channel.center_freq1) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].burst_period) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].num_burst) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].num_frames_per_burst) &&
                    fscanf(f_read,
                        "%d", &rtt_config[i].num_retries_per_rtt_frame) &&
                    fscanf(f_read, "%d", &rtt_config[i].num_retries_per_ftmr) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].LCI_request) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].LCR_request) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].burst_duration) &&
                    fscanf(f_read, "%d", (int *)&rtt_config[i].preamble) &&
                    fscanf(f_read, "%hhx", (unsigned char *)&rtt_config[i].bw)) == 0 ) {
                    fprintf(stderr, "%s: Failed parsing RTT Range Request "
                        "params file. Exit\n", __func__);
                    fclose(f_read);
                    return;
                }

                fprintf(stderr, "rtt_config[%d].index:%d\n",
                        i, dummyIndex);

                fprintf(stderr,
                        "rtt_config[%d].bssid:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\n",
                        i,
                        rtt_config[i].addr[0], rtt_config[i].addr[1],
                        rtt_config[i].addr[2], rtt_config[i].addr[3],
                        rtt_config[i].addr[4], rtt_config[i].addr[5]);

                fprintf(stderr, "rtt_config[%d].type:%d\n "
                        "rtt_config[%d].peer:%d\n"
                        "rtt_config[%d].channel.width:%d\n"
                        "rtt_config[%d].channel.center_freq:%d\n"
                        "rtt_config[%d].channel.center_freq0:%d\n"
                        "rtt_config[%d].channel.center_freq1:%d\n"
                        "rtt_config[%d].burst_period:%d\n"
                        "rtt_config[%d].num_burst:%d\n"
                        "rtt_config[%d].num_frames_per_burst:%d\n"
                        "rtt_config[%d].num_retries_per_rtt_frame:%d\n"
                        "rtt_config[%d].num_retries_per_ftmr:%d\n"
                        "rtt_config[%d].LCI_request:%d\n"
                        "rtt_config[%d].LCR_request:%d\n"
                        "rtt_config[%d].burst_duration:%d\n"
                        "rtt_config[%d].preamble:%02x\n"
                        "rtt_config[%d].bw:%02x\n"
                        "\n",
                        i, rtt_config[i].type,
                        i, rtt_config[i].peer,
                        i, rtt_config[i].channel.width,
                        i, rtt_config[i].channel.center_freq,
                        i, rtt_config[i].channel.center_freq0,
                        i, rtt_config[i].channel.center_freq1,
                        i, rtt_config[i].burst_period,
                        i, rtt_config[i].num_burst,
                        i, rtt_config[i].num_frames_per_burst,
                        i, rtt_config[i].num_retries_per_rtt_frame,
                        i, rtt_config[i].num_retries_per_ftmr,
                        i, rtt_config[i].LCI_request,
                        i, rtt_config[i].LCR_request,
                        i, rtt_config[i].burst_duration,
                        i, rtt_config[i].preamble,
                        i, rtt_config[i].bw
                        );
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "rttSendRangeRequest: Failed to "
            "open file /vendor/etc/wifi/rtt_range_request_params.txt - "
            "use hard-coded defaults\n");
            num_rtt_config = 1;
            memcpy(&rtt_config[0].addr[0], "012345", sizeof(mac_addr));
            rtt_config[0].type = RTT_TYPE_1_SIDED;
            rtt_config[0].peer = RTT_PEER_AP;
            rtt_config[0].channel.width = WIFI_CHAN_WIDTH_20;
            rtt_config[0].channel.center_freq = 2412;
            rtt_config[0].channel.center_freq0 = 2407;
            rtt_config[0].channel.center_freq1 = 2417;
            rtt_config[0].burst_period = 10;
            rtt_config[0].num_burst = 5;
            rtt_config[0].num_frames_per_burst = 10;
            rtt_config[0].num_retries_per_rtt_frame = 30;
            rtt_config[0].num_retries_per_ftmr = 5;
            rtt_config[0].LCI_request = 0;
            rtt_config[0].LCR_request = 0;
            rtt_config[0].burst_duration = 15;
            rtt_config[0].preamble = WIFI_RTT_PREAMBLE_LEGACY;
            rtt_config[0].bw = WIFI_RTT_BW_20;
        }

        if(glb_total_iter) {

             /* Set the callback handler functions for related events. */
             nHandler.on_rtt_results = rtt_median_on_results;
#ifdef CONFIG_ANDROID_14
             nHandler.on_rtt_results_v2 = rtt_median_on_results_v2;
#endif
             fprintf(stderr, "%s: Sending RTT Range request"
                     " completed. Returned value: 0 \n", __func__);
             fprintf(stderr,"Waiting for Results");

             glb_curr_iter = 1;

             for(i=1; i <= glb_total_iter; i++)
             {
                ret = wifi_rtt_range_request(id, wifiHandle_,
                                             1, rtt_config, nHandler);
                usleep(glb_sleep_time * 1000);
             }
        }
        else
        {

             /* Set the callback handler functions for related events. */
             nHandler.on_rtt_results = rtt_on_results;
#ifdef CONFIG_ANDROID_14
             nHandler.on_rtt_results_v2 = rtt_on_results_v2;
#endif
             ret = wifi_rtt_range_request(id, wifiHandle_,
                                          num_rtt_config, rtt_config, nHandler);
             fprintf(stderr, "%s: Sending RTT Range request"
                     " completed. Returned value: %d.\n", __func__, ret);
        }

        if (ret)
            return;
    }

    void RttTestSuite::rttSendCancelRangeRequest(int argc, char **argv)
    {
        int i, ret = 0;
        unsigned num_devices = 0;
        mac_addr addr[1000];

        fprintf(stderr, "rttSendCancelRangeRequest: Sending RTT "
            "CancelRange Request. \n");

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
                    rttPrintCmdUsage(argv, RTT_CMD, RTT_RANGE_CANCEL,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        fprintf(stderr, "Parse params from "
            "/vendor/etc/wifi/rtt_range_cancel_params.txt \n");

        /* Initialize the newly allocated memory area with 0. */
        memset(addr, 0, 1000*sizeof(mac_addr));

        FILE *f_read = fopen("/vendor/etc/wifi/rtt_range_cancel_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if (fscanf(f_read, "%d", &num_devices)) {
                fprintf(stderr, "rttSendCancelRangeRequest: Parsed "
                    "number of MAC Addresses:%d\n", num_devices);
            }
            if (num_devices) {
                for ( i = 0; i < (int)num_devices; i++ )
                {
                    if ( (fscanf(f_read, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                            &addr[i][0], &addr[i][1],
                            &addr[i][2], &addr[i][3],
                            &addr[i][4], &addr[i][5])) == 0 ) {
                        fprintf(stderr, "%s: Failed parsing RTT Range Cancel "
                            "params file. Exit\n", __func__);
                        fclose(f_read);
                        return;
                    }

                    fprintf(stderr, "addr[%d]:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\n",
                            i,
                            addr[i][0], addr[i][1],
                            addr[i][2], addr[i][3],
                            addr[i][4], addr[i][5]);
                }
                fprintf(stderr, "\n");
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "rttSendCancelRangeRequest: Failed to open "
            "file /vendor/etc/wifi/rtt_range_cancel_params.txt  - "
            "use hard-coded defaults\n");

            num_devices = 3;
            memcpy(&addr[0], "012345", sizeof(mac_addr));
            memcpy(&addr[1], "678901", sizeof(mac_addr));
            memcpy(&addr[2], "333333", sizeof(mac_addr));
        }

        ret = wifi_rtt_range_cancel(id, wifiHandle_, num_devices, addr);
        fprintf(stderr, "rttSendCancelRangeRequest: Sending RTT Range "
            "Cancel request completed. Returned value: %d.\n", ret);

        if (ret)
            return;
    }

    int RttTestSuite::rttParseHex(unsigned char c)
    {
       if (c >= '0' && c <= '9')
          return c-'0';
       if (c >= 'a' && c <= 'f')
          return c-'a'+10;
       if (c >= 'A' && c <= 'F')
          return c-'A'+10;
       return 0;
    }

    int RttTestSuite::rttParseMacAddress(const char* arg, u8* addr)
    {
       if (strlen(arg) != 17)
       {
          fprintf(stderr, "Invalid mac address %s\n", arg);
          fprintf(stderr, "expected format xx:xx:xx:xx:xx:xx\n");
          return -1;
       }

       addr[0] = rttParseHex(arg[0]) << 4 | rttParseHex(arg[1]);
       addr[1] = rttParseHex(arg[3]) << 4 | rttParseHex(arg[4]);
       addr[2] = rttParseHex(arg[6]) << 4 | rttParseHex(arg[7]);
       addr[3] = rttParseHex(arg[9]) << 4 | rttParseHex(arg[10]);
       addr[4] = rttParseHex(arg[12]) << 4 | rttParseHex(arg[13]);
       addr[5] = rttParseHex(arg[15]) << 4 | rttParseHex(arg[16]);
       return 0;
    }

    int RttTestSuite::rttParseHexString(
        const char* input,
        u8* output,
        int* outputlen)
    {
        int i = 0;
        int j = 0;
        for(i = 0; ((i < (int)strlen(input)) && (j < *(outputlen))); i+= 2)
        {
            output[j] = rttParseHex(input[i]);
            if((i + 1) < (int)strlen(input))
            {
                output[j] = ((output[j] << 4) | rttParseHex(input[i + 1]));
            }
            j++;
        }
        *outputlen = j;
        fprintf(stderr, "rttParseHexString: Input:%s inputlen:%d "
                        "outputlen:%d\n", input, (int)strlen(input),
                        (int)*outputlen);
        return 0;
    }

    void
    RttTestSuite::rtthexdump(
        uint8_t* data,
        size_t len)
    {
        char buf[512];
        uint16_t index;
        uint8_t* ptr;
        int pos;

        memset(buf, 0, sizeof(buf));
        ptr = data;
        pos = 0;
        for (index=0; index<len; index++)
        {
            pos += snprintf(&(buf[pos]), sizeof(buf) - pos, "%02x ", *ptr++);
            if (pos > 508)
            {
                break;
            }
        }

        fprintf(stderr,"HEX DUMP len=[%d]\n", (int)len);
        fprintf(stderr,"buf:%s\n", buf);
    }

    void RttTestSuite::rttSetLCI(int argc, char **argv)
    {
        fprintf(stderr, "%s: Sending RTT Set LCI Request. "
            "\n", __FUNCTION__);
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        /* Override with command line arguements */
        int long_index = 0, opt = 0, ret;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    rttPrintCmdUsage(argv, RTT_CMD,
                        RTT_SET_LCI,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        wifi_lci_information rtt_lci;

        memset(&rtt_lci, 0, sizeof(rtt_lci));
        FILE *f_read = fopen("/vendor/etc/wifi/rtt_lci_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if ((fscanf(f_read, "%ld", &rtt_lci.latitude) &&
                fscanf(f_read, "%ld", &rtt_lci.longitude) &&
                fscanf(f_read, "%d", &rtt_lci.altitude) &&
                fscanf(f_read, "%d", (int*)&rtt_lci.latitude_unc) &&
                fscanf(f_read, "%d", (int*)&rtt_lci.longitude_unc) &&
                fscanf(f_read, "%d", (int*)&rtt_lci.altitude_unc) &&
                fscanf(f_read, "%d", (int*)&rtt_lci.motion_pattern) &&
                fscanf(f_read, "%d", &rtt_lci.floor) &&
                fscanf(f_read, "%d", &rtt_lci.height_above_floor) &&
                fscanf(f_read, "%d", &rtt_lci.height_unc)) == 0 ) {
                fprintf(stderr, "%s: Failed parsing RTT Set LCI Request "
                    "params file. Exit\n", __func__);
                fclose(f_read);
                return;
            }
            fclose(f_read);
        } else {
            fprintf(stderr, "%s: Failed to "
            "open file /vendor/etc/wifi/rtt_lci_params.txt - "
            "use hard-coded defaults\n", __FUNCTION__);
            rtt_lci.latitude = 100000000;
            rtt_lci.longitude = 200000000;
            rtt_lci.altitude = 100;
            rtt_lci.latitude_unc = 10;
            rtt_lci.longitude_unc = 10;
            rtt_lci.altitude_unc = 10;
            rtt_lci.motion_pattern = WIFI_MOTION_EXPECTED;
            rtt_lci.floor = 3;
            rtt_lci.height_above_floor = 8;
            rtt_lci.height_unc = 0;
        }

        fprintf(stderr, "rtt_lci.latitude:%ld\n"
                "rtt_lci.longitude:%ld\n"
                "rtt_lci.altitude:%d\n"
                "rtt_lci.lattitude_unc:%d\n"
                "rtt_lci.longitude_unc:%d\n"
                "rtt_lci.altitude_unc:%d\n"
                "rtt_lci.motion_pattern:%d\n"
                "rtt_lci.floor:%d\n"
                "rtt_lci.height_above_floor:%d\n"
                "rtt_lci.height_unc:%d\n",
                rtt_lci.latitude,
                rtt_lci.longitude,
                rtt_lci.altitude,
                rtt_lci.latitude_unc,
                rtt_lci.longitude_unc,
                rtt_lci.altitude_unc,
                rtt_lci.motion_pattern,
                rtt_lci.floor,
                rtt_lci.height_above_floor,
                rtt_lci.height_unc);

        ret = wifi_set_lci(id, wifiHandle_, &rtt_lci);

        fprintf(stderr, "%s: Sending RTT Set LCI request"
            " completed. Returned value: %d.\n", __FUNCTION__, ret);

        if (ret)
            return;
    }

    void RttTestSuite::rttSetLCR(int argc, char **argv)
    {
        fprintf(stderr, "%s: Sending RTT Set LCR Request. "
            "\n", __FUNCTION__);
        /* A string listing valid short options letters.  */
        const char* const short_options = "h:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
            };
        /* Override with command line arguements */
        int long_index = 0, opt = 0, ret;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'h':
                default:
                    rttPrintCmdUsage(argv, RTT_CMD,
                        RTT_SET_LCR,
                        long_options,
                        sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }

        wifi_lcr_information rtt_lcr;
        char civic_info_string[1024];

        memset(&rtt_lcr, 0, sizeof(rtt_lcr));
        memset(civic_info_string, 0, sizeof(civic_info_string));
        FILE *f_read = fopen("/vendor/etc/wifi/rtt_lcr_params.txt", "r");
        fprintf(stderr, "\n");
        if (f_read) {
            if ((fscanf(f_read, "%c%c\n", &rtt_lcr.country_code[0],
                        &rtt_lcr.country_code[1]) &&
                fgets(civic_info_string, 1024, f_read)) == 0 ) {
                fprintf(stderr, "%s: Failed parsing RTT Set LCR Request "
                    "params file. Exit\n", __func__);
                fclose(f_read);
                return;
            }

            rtt_lcr.length = strlen(civic_info_string);
            if (strlen(civic_info_string) > sizeof(rtt_lcr.civic_info))
                rtt_lcr.length = sizeof(rtt_lcr.civic_info);
            memcpy(rtt_lcr.civic_info, civic_info_string,
                   rtt_lcr.length);
            fclose(f_read);
        } else {
            char civic_info[] = "Civic Info String";
            fprintf(stderr, "%s: Failed to "
            "open file /vendor/etc/wifi/rtt_lcr_params.txt - "
            "use hard-coded defaults\n", __FUNCTION__);
            rtt_lcr.country_code[0] = '0';
            rtt_lcr.country_code[1] = '0';
            rtt_lcr.length = sizeof(civic_info);
            memcpy(rtt_lcr.civic_info, civic_info, sizeof(civic_info));
        }

        fprintf(stderr, "rtt_lcr.country_code:%c%c\n"
                "rtt_lcr.length:%d\n",
                rtt_lcr.country_code[0],
                rtt_lcr.country_code[1],
                rtt_lcr.length);
        fprintf(stderr, "rtt_lcr.civic_info:%s",
                rtt_lcr.civic_info);

        ret = wifi_set_lcr(id, wifiHandle_, &rtt_lcr);
        fprintf(stderr, "%s: Sending RTT Set LCR request"
            " completed. Returned value: %d.\n", __FUNCTION__, ret);

        if (ret)
            return;
    }

    void RttTestSuite::rttGetResponderInfo(int argc, char **argv)
    {
        UNUSED(argc), UNUSED(argv);

        if (!fn.wifi_rtt_get_responder_info) {
            fprintf(stderr, "No API registered for"
                    " wifi_rtt_get_responder_info\n");
            return;
        }

        fprintf(stderr, "%s: Sending RTT Get channel Request. "
            "\n", __FUNCTION__);

        wifi_rtt_responder responder_info;

        memset(&responder_info, 0, sizeof(wifi_rtt_responder));

        ret = fn.wifi_rtt_get_responder_info(wifiHandle_, &responder_info);
        fprintf(stderr, "%s: Sending RTT Get responder information : %s\n",
                __FUNCTION__, ret?"Failed":"Success");
        if (ret == 0) {
            fprintf(stderr, " Channel width: %d\n",
                    responder_info.channel.width);
            fprintf(stderr, " Channel center_freq: %d\n",
                    responder_info.channel.center_freq);
            fprintf(stderr, "Channel center_freq0: %d\n",
                    responder_info.channel.center_freq0);
            fprintf(stderr, "Channel center_freq1: %d\n",
                    responder_info.channel.center_freq1);
            fprintf(stderr, "Measurement Preamble: %02x\n",
                    responder_info.preamble);
        }
    }

    void RttTestSuite::rttEnableResponder(int argc, char **argv)
    {
        UNUSED(argc), UNUSED(argv);
        wifi_channel_info channel_hint;
        int max_duration_seconds;
        wifi_rtt_responder responder_info;

        if (!fn.wifi_enable_responder) {
            fprintf(stderr, "No API registered for wifi_enable_responder\n");
            return;
        }
        memset(&channel_hint, 0, sizeof(wifi_channel_info));
        memset(&responder_info, 0, sizeof(wifi_rtt_responder));

        FILE *f_read = fopen("/vendor/etc/wifi/rtt_enable_responder.txt", "r");
        if (f_read) {
            fprintf(stderr, "Parse params from "
                    "/vendor/etc/wifi/rtt_enable_responder.txt \n");
            if ((fscanf(f_read, "%d", &channel_hint.width) &&
                fscanf(f_read, "%d", &channel_hint.center_freq) &&
                fscanf(f_read, "%d", &channel_hint.center_freq0) &&
                fscanf(f_read, "%d", &channel_hint.center_freq1) &&
                fscanf(f_read, "%d", &max_duration_seconds)) == 0) {
                fprintf(stderr, "Failed to Parse");
                fclose(f_read);
                return;
            }

            fclose(f_read);
        } else {
            fprintf(stderr, "Failed to open "
                    "/vendor/etc/wifi/rtt_enable_responder.txt\n");

            channel_hint.width = WIFI_CHAN_WIDTH_20;
            channel_hint.center_freq = 2412;
            channel_hint.center_freq0 = 2412;
            channel_hint.center_freq1 = 0;
            max_duration_seconds = 5;

        }
        fprintf(stderr, "Channel_hint width         : %d\n",
                channel_hint.width);
        fprintf(stderr, "Channel_hint center_freq   : %d\n",
                channel_hint.center_freq);
        fprintf(stderr, "Channel_hint center_freq0  : %d\n",
                channel_hint.center_freq0);
        fprintf(stderr, "Channel_hint center_freq1  : %d\n",
                channel_hint.center_freq1);
        fprintf(stderr, "max_duration_seconds: %d\n", max_duration_seconds);

        fprintf(stderr, "%s: Sending RTT Responder enable Request.\n",
                __FUNCTION__);

        ret = fn.wifi_enable_responder(id, wifiHandle_, channel_hint,
                                    max_duration_seconds, &responder_info);
        fprintf(stderr, "%s: Sending RTT Responder enable Request : %s\n",
                __FUNCTION__, ret?"Failed":"Success");
        if (ret == 0) {
            fprintf(stderr, "Channel_used width         : %d\n",
                    responder_info.channel.width);
            fprintf(stderr, "Channel_used center_freq   : %d\n",
                    responder_info.channel.center_freq);
            fprintf(stderr, "Channel_used center_freq0  : %d\n",
                    responder_info.channel.center_freq0);
            fprintf(stderr, "Channel_used center_freq1  : %d\n",
                    responder_info.channel.center_freq1);
            fprintf(stderr, "Measurement Preamble used  : %02x\n",
                    responder_info.preamble);
        }
    }

    void RttTestSuite::rttDisableResponder(int argc, char **argv)
    {
        UNUSED(argc), UNUSED(argv);
        fprintf(stderr, "%s: Sending RTT Responder disable Request. "
            "\n", __FUNCTION__);

        ret = wifi_disable_responder(id, wifiHandle_);
        fprintf(stderr, "%s: Sending RTT Responder disable Request : %s\n",
                __FUNCTION__, ret?"Failed":"Success");
    }
}
