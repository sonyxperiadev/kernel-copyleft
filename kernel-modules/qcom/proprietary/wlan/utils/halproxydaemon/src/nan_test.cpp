/*
* Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#include "nan_test.hpp"
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <net/if.h>

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define NDP_MAX_INSTANCE_IDS 20
#define NAN_MAX_PMKID_LEN    18
#define SYS_CMD_LEN          50
int gExitflag = 0;
wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name);

namespace NAN_TEST
{
    /* CLI cmd strings */
    const char *NanTestSuite::NAN_CMD = "nan";
    const char *NanTestSuite::NAN_PUBLISH = "publish";
    const char *NanTestSuite::NAN_SUBSCRIBE = "subscribe";
    const char *NanTestSuite::NAN_ENABLE = "enable";
    const char *NanTestSuite::NAN_DISABLE = "disable";
    const char *NanTestSuite::NAN_PUBLISH_CANCEL = "publish_cancel";
    const char *NanTestSuite::NAN_SUBSCRIBE_CANCEL = "subscribe_cancel";
    const char *NanTestSuite::NAN_CONFIG = "config";
    const char *NanTestSuite::NAN_STATS = "stats";
    const char *NanTestSuite::NAN_TCA = "tca";
    const char *NanTestSuite::NAN_FOLLOWUP = "followup";
    const char *NanTestSuite::NAN_BEACONSDF = "beaconsdf";
    const char *NanTestSuite::NAN_GETSTAPARAMETER = "sta_get_parameter";
    const char *NanTestSuite::NAN_GETCAPABILITIES = "get_capabilities";
    const char *NanTestSuite::NDI_CREATE = "ndi_create";
    const char *NanTestSuite::NDI_DELETE = "ndi_delete";
    const char *NanTestSuite::NDP_REQUEST = "ndp_request";
    const char *NanTestSuite::NDP_RESPONSE = "ndp_response";
    const char *NanTestSuite::NDP_END = "ndp_end";
    const char *NanTestSuite::NDP_SCHEDULE_UPDATE = "schedule_update";
    const char *NanTestSuite::NAN_DEBUG = "debug";

    const char *NanTestSuite::DEFAULT_SVC = "GoogleNanCluster";
    const char *NanTestSuite::DEFAULT_SVC_INFO = "GoogleHangout";
    const char *NanTestSuite::NAN_AWARE_IFACE = "wifi-aware0";

    /* Constructor */
    NanTestSuite::NanTestSuite(wifi_handle handle)
        :wifiHandle_(handle)
    {
        NanCallbackHandler callbackHandler =
        {
            .NotifyResponse = NanTestSuite::nanNotifyResponse,
#if QTI_BSP
            .EventPublishReplied = NanTestSuite::nanEventPublishReplied,
#endif
            .EventPublishTerminated = NanTestSuite::nanEventPublishTerminated,
            .EventMatch = NanTestSuite::nanEventMatch,
            .EventMatchExpired = NanTestSuite::nanEventMatchExpired,
            .EventSubscribeTerminated = NanTestSuite::nanEventSubscribeTerminated,
            .EventFollowup = NanTestSuite::nanEventFollowup,
            .EventDiscEngEvent = NanTestSuite::nanEventDiscEngEvent,
            .EventDisabled = NanTestSuite::nanEventDisabled,
            .EventTca = NanTestSuite::nanEventTca,
            .EventBeaconSdfPayload = NanTestSuite::nanEventBeaconSdfPayload,
            .EventDataRequest = NanTestSuite::ndpEventRequest,
            .EventDataConfirm = NanTestSuite::ndpEventConfirm,
            .EventDataEnd = NanTestSuite::ndpEventEnd,
            .EventTransmitFollowup = NanTestSuite::nanEventSelfTransmitFollowup,
            .EventRangeRequest = NanTestSuite::nanRangeRequestReceivedInd,
            .EventRangeReport = NanTestSuite::nanRangeReportInd,
            .EventScheduleUpdate = NanTestSuite::ndpEventScheduleUpdate
        };

        if (if_nametoindex(NAN_AWARE_IFACE)) {
            strlcpy(cur_nan_iface, (char*)NAN_AWARE_IFACE, IFNAMSIZ);
            ifaceHandle = wifi_get_iface_handle(wifiHandle_, cur_nan_iface);
        } else {
            strlcpy(cur_nan_iface, "wlan0", IFNAMSIZ);
            ifaceHandle = wifi_get_iface_handle(wifiHandle_, cur_nan_iface);
        }

        if(!ifaceHandle)
        {
            fprintf(stderr, "Interface %s is not existing.\n", cur_nan_iface);
            fprintf(stderr, "Please check Interface %s is present at this path: %s\n",
                    cur_nan_iface, "/sys/class/net/");
            return;
        }
        nan_register_handler(ifaceHandle, callbackHandler);
        gExitflag = 0;
    }

    /* process the command line args */
    void NanTestSuite::processCmd(int argc, char **argv)
    {
        if(argc <3)
        {
            fprintf(stderr, "%s: insufficient NAN args\n", argv[0]);
            return;
        }

        if(strcasecmp(argv[2], NAN_ENABLE) == 0)
            return nanSendEnableRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_DISABLE) == 0)
            return nanSendDisableRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_PUBLISH) == 0)
            return nanSendPublishRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_PUBLISH_CANCEL) == 0)
            return nanSendPublishCancelRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_SUBSCRIBE) == 0)
            return nanSendSubscribeRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_SUBSCRIBE_CANCEL) == 0)
            return nanSendSubscribeCancelRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_CONFIG) == 0)
            return nanSendConfigRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_FOLLOWUP) == 0)
            return nanSendTransmitFollowupRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_STATS) == 0)
            return nanSendStatsRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_TCA) == 0)
            return nanSendTCARequest(argc, argv);

        if(strcasecmp(argv[2], NAN_BEACONSDF) == 0)
            return nanSendBeaconSdfRequest(argc, argv);

        if(strcasecmp(argv[2], NAN_GETCAPABILITIES) == 0)
            return nanGetCapabilities(argc, argv);

        if(strcasecmp(argv[2], NDI_CREATE) == 0)
            return ndpInterfaceCreate(argc, argv);

        if(strcasecmp(argv[2], NDI_DELETE) == 0)
            return ndpInterfaceDelete(argc, argv);

        if(strcasecmp(argv[2], NDP_REQUEST) == 0)
            return ndpInitiatorRequest(argc, argv);

        if(strcasecmp(argv[2], NDP_RESPONSE) == 0)
            return ndpIndicationResponse(argc, argv);

        if(strcasecmp(argv[2], NDP_END) == 0)
            return ndpEnd(argc, argv);

        if(strcasecmp(argv[2], NAN_GETSTAPARAMETER) == 0)
            return nanGetStaParameter(argc, argv);

        if(strcasecmp(argv[2], NAN_DEBUG) == 0)
            return nanSendDebugRequest(argc, argv);

        if(strcasecmp(argv[2], "poll") == 0)
        {
            nanGetStaParameter(argc, argv);
            exit(0);
        }

        fprintf(stderr, "%s: unknown  arg %s\n", argv[0], argv[2]);
    }

    /* NotifyResponse invoked to notify the status of the Request */
    void NanTestSuite::nanNotifyResponse(transaction_id id, NanResponseMsg *rsp_data)
    {
        fprintf(stderr, "%s: req_id %d status %d response_type %d\n",
          __FUNCTION__,
          id,
          rsp_data->status,
          rsp_data->response_type
        );

        if(rsp_data->response_type == NAN_RESPONSE_STATS)
        {
           fprintf(stderr, "%s: stats_type %d\n",
             __FUNCTION__,
             rsp_data->body.stats_response.stats_type
           );
           nanPrintStatsResponse(&rsp_data->body.stats_response);
           if (gExitflag)
           {
                fprintf(stderr, "Exiting...\n");
                exit(0);
           }
        }

        if(rsp_data->response_type == NAN_RESPONSE_PUBLISH)
        {
            fprintf(stderr, "%s: publish_id %d\n",
                  __FUNCTION__,
                  rsp_data->body.publish_response.publish_id);
            if (gExitflag)
            {
                fprintf(stderr, "Exiting...\n");
                exit(0);
            }
        }
        if(rsp_data->response_type == NAN_RESPONSE_SUBSCRIBE)
        {
            fprintf(stderr, "%s: subscribe_id %d\n",
                  __FUNCTION__,
                  rsp_data->body.subscribe_response.subscribe_id);
            if (gExitflag)
            {
                fprintf(stderr, "Exiting...\n");
                exit(0);
            }
        }
        if(rsp_data->response_type == NAN_GET_CAPABILITIES)
        {
           nanPrintCapabilities(&rsp_data->body.nan_capabilities);
           if (gExitflag)
           {
                fprintf(stderr, "Exiting...\n");
                exit(0);
           }
        }
        if (rsp_data->response_type == NAN_RESPONSE_ENABLED)
        {
            // Exit in JOINED/STARTED event if gExitflag is set
        }
        if (rsp_data->response_type == NAN_DP_INITIATOR_RESPONSE)
        {
             fprintf(stderr, "%s: ndp_instance_id %d\n",
                  __FUNCTION__,
                  rsp_data->body.data_request_response.ndp_instance_id);
            if (gExitflag)
            {
                fprintf(stderr, "Exiting...\n");
                exit(0);
            }
        }
        else if ((rsp_data->response_type == NAN_RESPONSE_DISABLED) ||
                 (rsp_data->response_type == NAN_RESPONSE_PUBLISH_CANCEL) ||
                 (rsp_data->response_type == NAN_RESPONSE_SUBSCRIBE_CANCEL) ||
                 (rsp_data->response_type == NAN_RESPONSE_TRANSMIT_FOLLOWUP) ||
                 (rsp_data->response_type == NAN_RESPONSE_TCA) ||
                 (rsp_data->response_type == NAN_RESPONSE_BEACON_SDF_PAYLOAD) ||
                 (rsp_data->response_type == NAN_RESPONSE_STATS) ||
                 (rsp_data->response_type == NAN_RESPONSE_CONFIG) ||
                 (rsp_data->response_type == NAN_RESPONSE_BEACON_SDF_PAYLOAD) ||
                 (rsp_data->response_type == NAN_DP_INTERFACE_CREATE) ||
                 (rsp_data->response_type == NAN_DP_INTERFACE_DELETE) ||
                 (rsp_data->response_type == NAN_DP_RESPONDER_RESPONSE) ||
                 (rsp_data->response_type == NAN_DP_END))
        {
           if (gExitflag)
           {
               fprintf(stderr, "Exiting...\n");
               exit(0);
           }
        }
    }

#if QTI_BSP
    /* Events Callback */
    void NanTestSuite::nanEventPublishReplied(NanPublishRepliedInd *event)
    {
        fprintf(stderr, "%s: Requestor Instance Id %08x "
                 MAC_ADDR_STR "rssi:%d\n",
          __FUNCTION__,
          event->requestor_instance_id,
          MAC_ADDR_ARRAY(event->addr),
          event->rssi_value
        );
    }
#endif

    /* Events Callback */
    void NanTestSuite::nanEventPublishTerminated(NanPublishTerminatedInd *event)
    {
        fprintf(stderr, "%s: publish_id %d reason %d\n",
          __FUNCTION__,
         event->publish_id,
         event->reason
        );
    }

    /* Events Callback */
    void NanTestSuite::nanEventMatch(NanMatchInd *event)
    {
        fprintf(stderr, "%s: publish_subscribe_id %d "
                "Requestor Instance Id %08x "
                MAC_ADDR_STR " rssi:%d "
                "Match Occured Flag:%d Out of Resource Flag:%d "
                "Is NDP = %d\n"
                "NDP Type = %d\n",
          __FUNCTION__,
          event->publish_subscribe_id,
          event->requestor_instance_id,
          MAC_ADDR_ARRAY(event->addr),
          event->rssi_value,
          event->match_occured_flag,
          event->out_of_resource_flag,
          event->peer_sdea_params.config_nan_data_path,
          event->peer_sdea_params.ndp_type
        );
        /* Print the SSI */
        fprintf(stderr, "Printing SSI:");
        nanhexdump(event->service_specific_info, event->service_specific_info_len);
        /* Print the match filter */
        fprintf(stderr, "Printing sdf match filter:");
        nanhexdump(event->sdf_match_filter, event->sdf_match_filter_len);
        /* Print the conn_capability */
        fprintf(stderr, "Printing PostConnectivity Capability \n");
        if (event->is_conn_capability_valid)
        {
            fprintf(stderr, "Wfd supported:%s\n",
                    (event->conn_capability.is_wfd_supported? "yes": "no"));
            fprintf(stderr, "Wfds supported:%s\n",
                    (event->conn_capability.is_wfds_supported? "yes": "no"));
            fprintf(stderr, "TDLS supported:%s\n",
                    (event->conn_capability.is_tdls_supported? "yes": "no"));
            fprintf(stderr, "IBSS supported:%s\n",
                    (event->conn_capability.is_ibss_supported? "yes": "no"));
            fprintf(stderr, "Mesh supported:%s\n",
                    (event->conn_capability.is_mesh_supported? "yes": "no"));
            fprintf(stderr, "Infra Field:%d\n",
                    event->conn_capability.wlan_infra_field);
        }
        else
            fprintf(stderr,"PostConnectivity Capability not present\n");
        /* Print the discovery_attr */
        fprintf(stderr, "Printing PostDiscovery Attribute \n");
        if (event->num_rx_discovery_attr)
        {
            int idx = 0;
            for(idx = 0; idx < event->num_rx_discovery_attr; idx++)
            {
                fprintf(stderr, "PostDiscovery Attribute - %d\n", idx);
                fprintf(stderr,"Conn Type:%d Device Role:%d" MAC_ADDR_STR "\n",
                        event->discovery_attr[idx].type,
                        event->discovery_attr[idx].role,
                        MAC_ADDR_ARRAY(event->discovery_attr[idx].addr));
                fprintf(stderr,"Duration:%d MapId:%d "
                               "avail_interval_bitmap:%04x \n",
                        event->discovery_attr[idx].duration,
                        event->discovery_attr[idx].mapid,
                        event->discovery_attr[idx].avail_interval_bitmap);
                fprintf(stderr,"Printing Mesh Id:");
                nanhexdump(event->discovery_attr[idx].mesh_id,
                           event->discovery_attr[idx].mesh_id_len);
                fprintf(stderr,"Printing Infrastructure Ssid:");
                nanhexdump(event->discovery_attr[idx].infrastructure_ssid_val,
                           event->discovery_attr[idx].infrastructure_ssid_len);
            }
        }
        else
            fprintf(stderr,"PostDiscovery attribute not present\n");
        /* Print the fam */
        if (event->num_chans)
        {
            nanPrintFurtherAvailabilityChan(event->num_chans, &event->famchan[0]);
        }
        else
            fprintf(stderr,"Further Availability Map not present\n");
        if(event->cluster_attribute_len)
        {
            fprintf(stderr,"Printing Cluster Attribute:");
            nanhexdump(event->cluster_attribute, event->cluster_attribute_len);
        }
        else
            fprintf(stderr,"Cluster Attribute not present\n");
        if(event->peer_cipher_type)
        {
            fprintf(stderr," Peer Cipher Type = %d \n", event->peer_cipher_type);
        }
        else
            fprintf(stderr,"Peer Cipher Type not present \n");
        if(event->scid_len)
        {
            fprintf(stderr,"SCID Contents, Total SCID Length = %d \n",
                    event->scid_len);
            u32 idx = 0;
            for(idx = 0; idx < event->scid_len; idx += NAN_MAX_PMKID_LEN)
            {
                fprintf(stderr,"Printing PMK_ID[%d]:", idx/NAN_MAX_PMKID_LEN);
                nanhexdump(&event->scid[idx], NAN_MAX_PMKID_LEN);
            }
        }
        else
            fprintf(stderr,"SCID not present, SCID Length = %d \n",
                    event->scid_len);
        if(event->peer_sdea_params.security_cfg)
        {
            fprintf(stderr,"Secuirty Required = %d \n", event->peer_sdea_params.security_cfg);
        }
        else
            fprintf(stderr," Secuirty is not required \n");

        if(event->peer_sdea_params.ranging_state)
        {
            fprintf(stderr,"Enable Ranging = %d \n", event->peer_sdea_params.ranging_state);
        }
        else
            fprintf(stderr," Ranging is not Enabled \n");

        fprintf(stderr, "Range Measurement %d", event->range_info.range_measurement_mm);
        fprintf(stderr, "Range Event Type %d", event->range_info.ranging_event_type);
        if (event->sdea_service_specific_info_len)
        {
            fprintf(stderr, "Printing SDEA SSI:");
            nanhexdump(event->sdea_service_specific_info, event->sdea_service_specific_info_len);
        }
    }

    /* Events Callback */
    void NanTestSuite::nanEventMatchExpired(NanMatchExpiredInd *event)
    {
        fprintf(stderr, "%s: Requestor Instance Id %08x\n",
          __FUNCTION__,
          event->requestor_instance_id
        );
    }

    /* Events Callback */
    void NanTestSuite::nanEventSubscribeTerminated(NanSubscribeTerminatedInd *event)
    {
        fprintf(stderr, "%s: subscribe_id %d reason %d\n",
          __FUNCTION__,
         event->subscribe_id,
         event->reason
        );
    }

    /* Events Callback */
    void NanTestSuite::nanEventFollowup(NanFollowupInd* event)
    {
        fprintf(stderr, "%s: publish_subscribe_id %d "
                "Requestor Instance Id %08x dw_or_faw %d "
                MAC_ADDR_STR " \n",
          __FUNCTION__,
          event->publish_subscribe_id,
          event->requestor_instance_id,
          event->dw_or_faw,
          MAC_ADDR_ARRAY(event->addr)
        );

        /* Print the SSI */
        fprintf(stderr, "Printing SSI:");
        nanhexdump(event->service_specific_info, event->service_specific_info_len);

        if (event->sdea_service_specific_info_len)
        {
            fprintf(stderr, "Printing SDEA SSI:");
            nanhexdump(event->sdea_service_specific_info, event->sdea_service_specific_info_len);
        }
    }

    /* Events Callback */
    void NanTestSuite::nanEventDiscEngEvent(NanDiscEngEventInd *event)
    {
        fprintf(stderr, "%s: event_type %d\n",
          __FUNCTION__,
         event->event_type
        );

        if(event->event_type == NAN_EVENT_ID_JOINED_CLUSTER)
        {
            fprintf(stderr, "%s: Joined cluster " MAC_ADDR_STR "\n",
                    __FUNCTION__,
                    MAC_ADDR_ARRAY(event->data.cluster.addr)
                    );
            if (gExitflag)
            {
                fprintf(stderr, "Exiting...\n");
                exit(0);
            }
        }
        if(event->event_type == NAN_EVENT_ID_STARTED_CLUSTER)
        {
            fprintf(stderr, "%s: Started cluster " MAC_ADDR_STR "\n",
                    __FUNCTION__,
                    MAC_ADDR_ARRAY(event->data.cluster.addr)
                    );
            if (gExitflag)
            {
                fprintf(stderr, "Exiting...\n");
                exit(0);
            }
        }
        if(event->event_type == NAN_EVENT_ID_DISC_MAC_ADDR)
        {
            fprintf(stderr, "%s: Self STA " MAC_ADDR_STR "\n",
                    __FUNCTION__,
                    MAC_ADDR_ARRAY(event->data.mac_addr.addr)
                    );
        }
    }

    /* Events Callback */
    void NanTestSuite::nanEventDisabled(NanDisabledInd *event)
    {
        fprintf(stderr, "%s: reason %d\n",
          __FUNCTION__,
         event->reason
        );
        if (event->reason == NAN_STATUS_SUCCESS && if_nametoindex(NAN_AWARE_IFACE)) {
            int bytes_copied = 0;
            char system_command[SYS_CMD_LEN];

            bytes_copied = snprintf(system_command, SYS_CMD_LEN, "ifconfig %s down", NAN_AWARE_IFACE);
            if (bytes_copied < 0 || bytes_copied >= SYS_CMD_LEN) {
                fprintf(stderr, "ERROR: %s failed to write System Command Buffer",__func__);
                return;
            }
            system(system_command);
        }
    }

    /* Events Callback */
    void NanTestSuite::nanEventTca(NanTCAInd *event)
    {
        fprintf(stderr, "%s: tca_type %d "
                "threshold risen %d threshold fallen %d\n",
          __FUNCTION__,
         event->tca_type,
         event->rising_direction_evt_flag,
         event->falling_direction_evt_flag
        );

        if(event->tca_type == NAN_TCA_ID_CLUSTER_SIZE)
        {
           fprintf(stderr, "%s: cluster_size %d\n",
             __FUNCTION__,
             event->data.cluster.cluster_size
           );
        }
    }

    void NanTestSuite::nanEventBeaconSdfPayload(NanBeaconSdfPayloadInd* event)
    {
        fprintf(stderr, "%s: addr " MAC_ADDR_STR
                "isVsa %d isBeaconSdfPayload %d\n",
          __FUNCTION__,
         MAC_ADDR_ARRAY(event->addr),
         event->is_vsa_received,
         event->is_beacon_sdf_payload_received
        );

        if(event->is_vsa_received)
        {
            fprintf(stderr,"%s: Printing VSA************\n", __FUNCTION__);
            fprintf(stderr,"vsa_received_on:%d\n", event->vsa.vsa_received_on);
            fprintf(stderr,"vendor_oui:0x%08x\n", event->vsa.vendor_oui);
            fprintf(stderr,"vsa_len:%d\n", event->vsa.attr_len);
            nanhexdump(event->vsa.vsa, event->vsa.attr_len);
            fprintf(stderr,"%s: Done VSA************\n", __FUNCTION__);
        }

        if(event->is_beacon_sdf_payload_received)
        {
            fprintf(stderr,"%s: Printing BeaconSdfPayloadReceive**********\n",
                    __FUNCTION__);
            fprintf(stderr,"Frame_len:%d\n", event->data.frame_len);
            nanhexdump(event->data.frame_data, event->data.frame_len);
            fprintf(stderr,"%s: Done BeaconSdfPayloadReceive**********\n",
                    __FUNCTION__);
        }
    }

    /* Events Callback */
    void NanTestSuite::ndpEventRequest(NanDataPathRequestInd *event)
    {
        fprintf(stderr,"Received NDP Request Indication**********\n");
        fprintf(stderr, "%s: Service Instance Id: %d "
                " Peer Discovery MAC ADDR " MAC_ADDR_STR
                " NDP Instance Id: %d"
                " App Info  len %d"
                " App Info %s\n",
                __FUNCTION__,
                event->service_instance_id,
                MAC_ADDR_ARRAY(event->peer_disc_mac_addr),
                event->ndp_instance_id,
                event->app_info.ndp_app_info_len,
                event->app_info.ndp_app_info);
    }

    /* Events Callback */
    void NanTestSuite::ndpEventConfirm(NanDataPathConfirmInd *event)
    {
        fprintf(stderr,"Received NDP Confirm Indication**********\n");
        fprintf(stderr, "%s: NDP Instance Id: %d "
                " Peer NDI MAC ADDR " MAC_ADDR_STR
                " NDP Response code: %d"
                " App Info  len %d"
                " App Info %s\n",
                __FUNCTION__,
                event->ndp_instance_id,
                MAC_ADDR_ARRAY(event->peer_ndi_mac_addr),
                event->rsp_code,
                event->app_info.ndp_app_info_len,
                event->app_info.ndp_app_info);

        fprintf(stderr, "*******Channel Info******\n");
        fprintf(stderr, "No. channel = %d \n", event->num_channels);
        for (u32 i = 0; i < event->num_channels; i++) {
            fprintf(stderr, "Id:%d Channel:%d Channel Width:%d NSS %d\n",
                            i,
                            event->channel_info[i].channel,
                            event->channel_info[i].bandwidth,
                            event->channel_info[i].nss);
        }

    }

    void NanTestSuite::ndpEventScheduleUpdate(NanDataPathScheduleUpdateInd *event)
    {
        fprintf(stderr,"*******Received NDP Schedule Update Indication*******\n");
        fprintf(stderr, "%s:"
                " Peer MAC ADDR " MAC_ADDR_STR
                " NDP Schedule Update reason: %d\n",
               __FUNCTION__,
                MAC_ADDR_ARRAY(event->peer_mac_addr),
                event->schedule_update_reason_code);

        fprintf(stderr, "******Channel Info******\n");
        fprintf(stderr, "No. channel = %d \n", event->num_channels);
        for (u32 i = 0; i < event->num_channels; i++) {
            fprintf(stderr, "Id:%d Channel:%d Channel Width:%d NSS %d\n",
                            i,
                            event->channel_info[i].channel,
                            event->channel_info[i].bandwidth,
                            event->channel_info[i].nss);
        }
        fprintf(stderr, "%s: NDP num instance Ids %d\n",
          __FUNCTION__,
         event->num_ndp_instances);
        for(int i = 0; i < event->num_ndp_instances; i++) {
            fprintf(stderr, "Instance Ids %d\n", event->ndp_instance_id[i]);
        }
    }

    /* Events Callback */
    void NanTestSuite::ndpEventEnd(NanDataPathEndInd *event)
    {
        fprintf(stderr,"Received NDP End Indication**********\n");
        fprintf(stderr, "%s: NDP num instance Ids %d\n",
          __FUNCTION__,
         event->num_ndp_instances);
        for(int i = 0; i < event->num_ndp_instances; i++) {
            fprintf(stderr, "Instance Ids %d\n", event->ndp_instance_id[i]);
        }
    }

    /* Events Callback */
    void NanTestSuite::nanEventSelfTransmitFollowup(NanTransmitFollowupInd *event)
    {
        fprintf(stderr, "%s: transaction_id %d reason %d\n",
          __FUNCTION__,
         event->id,
         event->reason
        );
    }

    /* Events Callback */
    void NanTestSuite::nanRangeRequestReceivedInd(NanRangeRequestInd *event)
    {
        fprintf(stderr, "%s: received \n", __FUNCTION__);
        fprintf(stderr, "%s:  Publish Id: %d "
                " Range Request received from MAC ADDR " MAC_ADDR_STR,
                __FUNCTION__,
                event->publish_id,
                MAC_ADDR_ARRAY(event->range_req_intf_addr)
               );
    }

    /* Events Callback */
    void NanTestSuite::nanRangeReportInd(NanRangeReportInd *event)
    {
        fprintf(stderr, "%s: received \n", __FUNCTION__);
        fprintf(stderr, "%s:  Publish Id: %d"
                " Range Report MAC ADDR " MAC_ADDR_STR
                " Range measurement %d \n",
                __FUNCTION__,
                event->publish_id,
                MAC_ADDR_ARRAY(event->range_req_intf_addr),
                event->range_measurement_mm
               );
    }

    /* Helper routine to print usage */
    void NanTestSuite::nanPrintCmdUsage(char **argv, const char *cmd,
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

    void NanTestSuite::nanSendEnableRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hc:l:r:s:o:p:g:a:b:d:e:f:i:j:k:m:" \
            "n:q:t:u:v:w:x:y:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",                   0,   NULL, 'h' },
            { "cluster_low",            1,   NULL, 'c' },
            { "cluster_high",           1,   NULL, 'l' },
            { "rssi_close",             1,   NULL, 'r' },
            { "rssi_middle",            1,   NULL, 's' },
            { "hop_count_limit",        1,   NULL, 'o' },
            { "master_pref",            1,   NULL, 'p' },
            { "5G_Support",             1,   NULL, 'g' },
            { "2.4G_Support",           1,   NULL, 'a' },
            { "2.4G_Beacon_use",        1,   NULL, 'b' },
            { "2.4G_Disc_use",          1,   NULL, 'd' },
            { "5G_Beacon_use",          1,   NULL, 'e' },
            { "5G_Disc_use",            1,   NULL, 'f' },
            { "5G_rssi_close",          1,   NULL, 'i' },
            { "5G_rssi_middle",         1,   NULL, 'j' },
            { "2.4G_rssi_proximity",    1,   NULL, 'k' },
            { "5G_rssi_proximity",      1,   NULL, 'm' },
            { "rssi_window_val",        1,   NULL, 'n' },
            { "oui_val",                1,   NULL, 'q' },
            { "nan_interface_addr",     1,   NULL, 't' },
            { "nan_cluster_attr_val",   1,   NULL, 'u' },
            { "nan_socialchannel_param",1,   NULL, 'v' },
            { "nan_debugflags_param",   1,   NULL, 'w' },
            { "random_factor_force",    1,   NULL, 'x' },
            { "hop_count_force",        1,   NULL, 'y' },
            { "sid_beacon",             1,   NULL, 'z' },
            { "exitflag",               1,   NULL, 1000 },
            { "transaction_id",         1,   NULL, 1005 },
            { "channel_24g",            1,   NULL, 1006 },
            { "channel_5g",             1,   NULL, 1007 },
            { "disc_mac_addr_rand_int", 1,   NULL, 1008 },
            { "cfg_discovery_ind",      1,   NULL, 1009 },
            { "subscribe_sid",          1,   NULL, 1010 },
            { "disc_bcn_int",           1,   NULL, 1011 },
            { "nss",                    1,   NULL, 1012 },
            { "enable_ranging",         1,   NULL, 1013 },
            { "enable_dw_termination",  1,   NULL, 1014 },
            { "2.4G_dw_interval_val",   1,   NULL, 1015 },
            { "5G_dw_interval_val",     1,   NULL, 1016 },
            { "ndpe_attr_supported",    1,   NULL, 1017 },
            { NULL,                     0,   NULL,  0  }/* Required at end of array.  */
        };

        int interval;
        NanEnableRequest req;
        memset(&req, 0, sizeof(NanEnableRequest));
        transaction_id id = 0;
        req.cluster_low = 0;
        req.cluster_high = 0;
        req.master_pref = 0;
        /*
           By default discovery MAC address randomization is enabled
           and default interval value is 30 minutes i.e. 1800 seconds
        */
        req.config_disc_mac_addr_randomization = 1;
        req.disc_mac_addr_rand_interval_sec = 1800;
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
               case 'c' :
                   req.cluster_low = (u16) strtoul(optarg, NULL, 0);
                   break;
               case 'l' :
                   req.cluster_high = (u16) strtoul(optarg, NULL, 0);
                   break;
               case 'r' :
                   req.config_2dot4g_rssi_close = 1;
                   req.rssi_close_2dot4g_val = atoi(optarg);
                   break;
               case 's' :
                   req.config_2dot4g_rssi_middle = 1;
                   req.rssi_middle_2dot4g_val = atoi(optarg);
                   break;
               case 'o' :
                   req.config_hop_count_limit = 1;
                   req.hop_count_limit_val = atoi(optarg);
                   break;
               case 'p' :
                   req.master_pref = atoi(optarg);
                   break;
               case 'g':
                   req.config_support_5g = 1;
                   req.support_5g_val = atoi(optarg);
                   break;
               case 'a':
                   req.config_2dot4g_support = 1;
                   req.support_2dot4g_val = atoi(optarg);
                   break;
               case 'b':
                   req.config_2dot4g_beacons = 1;
                   req.beacon_2dot4g_val = atoi(optarg);
                   break;
               case 'd':
                   req.config_2dot4g_sdf = 1;
                   req.sdf_2dot4g_val = atoi(optarg);
                   break;
               case 'e':
                   req.config_5g_beacons = 1;
                   req.beacon_5g_val = atoi(optarg);
                   break;
               case 'f':
                   req.config_5g_sdf = 1;
                   req.sdf_5g_val = atoi(optarg);
                   break;
               case 'i':
                   req.config_5g_rssi_close = 1;
                   req.rssi_close_5g_val = atoi(optarg);
                   break;
               case 'j':
                   req.config_5g_rssi_middle = 1;
                   req.rssi_middle_5g_val = atoi(optarg);
                   break;
               case 'k':
                   req.config_2dot4g_rssi_proximity = 1;
                   req.rssi_proximity_2dot4g_val = atoi(optarg);
                   break;
               case 'm':
                   req.config_5g_rssi_close_proximity = 1;
                   req.rssi_close_proximity_5g_val = atoi(optarg);
                   break;
               case 'n':
                   req.config_rssi_window_size = 1;
                   req.rssi_window_size_val = atoi(optarg);
                   break;
               case 'q':
               {
                   req.config_oui = 1;
                   u32 oui = 0;
                   size_t size = sizeof(oui);
                   nanParseHexString(optarg, (u8*)&oui,
                                     (int*)&size);
                   req.oui_val = ntohl(oui);
                   break;
               }
               case 't':
                   req.config_intf_addr = 1;
                   nanParseMacAddress(optarg, req.intf_addr_val);
                   break;
               case 'u':
                   req.config_cluster_attribute_val = atoi(optarg);
                   break;
               case 'v':
                   req.config_scan_params = 1;
                   nanParseSocialChannelParams(optarg, &req.scan_params_val);
                   break;
#if 0
               case 'w':
                   req.config_debug_flags = 1;
                   req.debug_flags_val = strtoull(optarg, NULL, 0);
                   break;
#endif
               case 'x':
                   req.config_random_factor_force = 1;
                   req.random_factor_force_val = atoi(optarg);
                   break;
               case 'y':
                   req.config_hop_count_force = 1;
                   req.hop_count_force_val = atoi(optarg);
                   break;
               case 'z':
                   req.config_sid_beacon = 1;
                   req.sid_beacon_val = atoi(optarg);
                   break;
                case 1000:
                    gExitflag = atoi(optarg);
                    break;
               case 1005:
                    id = atoi(optarg);
                    break;
               case 1006:
                  req.config_24g_channel = 1;
                  req.channel_24g_val = atoi(optarg);
                  break;
               case 1007:
                  req.config_5g_channel = 1;
                  req.channel_5g_val = atoi(optarg);
                  break;
               case 1008:
                  /*
                    By default discovery MAC address randomization is enabled
                    and maximum interval value is 30 minutes i.e. 1800 seconds
                    Nan enable and config can specify the interval value in seconds.
                    Value of zero disables discovery MAC address randomization
                  */
                  req.config_disc_mac_addr_randomization = 1;
                  interval = atoi(optarg);
                  if (interval <= 1800) {
                       req.disc_mac_addr_rand_interval_sec = interval;
                  } else {
                       fprintf(stderr, "%s: Invalid discovery MAC address randomization interval %d.\n"
                               "Maximum interval value is 30 minutes i.e. 1800 seconds\n",
                               __FUNCTION__, interval);
                       return;
                  }
                  break;
               case 1009:
                  req.discovery_indication_cfg = atoi(optarg);
                  break;
               case 1010:
                   req.config_subscribe_sid_beacon = 1;
                   req.subscribe_sid_beacon_val = atoi(optarg);
                   break;
#if NAN_CERT_VERSION >= 4
               case 1011:
                   req.config_discovery_beacon_int = 1;
                   req.discovery_beacon_interval = atoi(optarg);
                  break;
               case 1012:
                   req.config_nss = 1;
                   req.nss = atoi(optarg);
                   break;
                case 1013:
                   req.config_enable_ranging = 1;
                   req.enable_ranging = atoi(optarg);
                   break;
                case 1014:
                   req.config_dw_early_termination = 1;
                   req.enable_dw_termination = atoi(optarg);
                   break;
#else
               case 1011:
               case 1012:
               case 1013:
               case 1014:
                   fprintf(stderr, "%s: disc_bcn_int, nss, ranging, dw termination not\n"
                           "supported in this version of HAL current HAL Version is %d,"
                           "required version is 4 or above\n", __func__, NAN_CERT_VERSION);
#endif
               case 1015:
                   req.config_dw.config_2dot4g_dw_band = 1;
                   req.config_dw.dw_2dot4g_interval_val = atoi(optarg);
                   break;
               case 1016:
                   req.config_dw.config_5g_dw_band = 1;
                   req.config_dw.dw_5g_interval_val = atoi(optarg);
                   break;
               case 1017:
#if (NAN_MAJOR_VERSION > 2) || (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)
                   req.config_ndpe_attr = 1;
                   req.use_ndpe_attr = atoi(optarg);
#else
                   fprintf(stderr, "%s: ndpe_attr_supported not supported in this version of HAL\n"
                           "current HAL Version is %d, required version is 4 or above\n",
                           __func__, NAN_CERT_VERSION);
#endif
                   break;
               case 'h':
               default:
                   nanPrintCmdUsage(argv, NAN_CMD, NAN_ENABLE, long_options,
                                    sizeof(long_options)/sizeof(struct option));
                   return;
            }
        }

        fprintf(stderr, "%s: hop_count_limit %d master_pref %d\n",
            __FUNCTION__, req.hop_count_limit_val, req.master_pref);
        fprintf(stderr, "%s: cluster_low 0x%04x cluster_high 0x%04x\n",
            __FUNCTION__, req.cluster_low, req.cluster_high);
        if (if_nametoindex(NAN_AWARE_IFACE)) {
            int bytes_copied = 0;
            char system_command[SYS_CMD_LEN];

            bytes_copied = snprintf(system_command, SYS_CMD_LEN, "ifconfig %s up", NAN_AWARE_IFACE);
            if (bytes_copied < 0 || bytes_copied >= SYS_CMD_LEN) {
                fprintf(stderr, "ERROR: %s failed to write System Command Buffer",__func__);
                return;
            }
            system(system_command);
        }
        nan_enable_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendPublishRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options =
                                    "hl:t:p:r:u:x:b:s:i:a:c:d:e:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",               0,   NULL, 'h' },
            { "publish_id",         1,   NULL, 'l' },
            { "ttl",                1,   NULL, 't' },
            { "period",             1,   NULL, 'p' },
            { "publish_match",      1,   NULL, 'r' },
            { "publish_type",       1,   NULL, 'u' },
            { "tx_type",            1,   NULL, 'x' },
            { "publish_count",      1,   NULL, 'b' },
            { "service_name",       1,   NULL, 's' },
            { "ssi",                1,   NULL, 'i' },
            { "rx_match_filter",    1,   NULL, 'a' },
            { "tx_match_filter",    1,   NULL, 'c' },
            { "rssi_threshold",     1,   NULL, 'd' },
            { "connmap",            1,   NULL, 'e' },
            { "ndp_type",           1,   NULL, 'n' },
            { "service_responder_policy",  1,   NULL, 'q' },
	    { "csid",               1,   NULL, 'k' },
            { "pmk",                1,   NULL, 'm' },
            { "enable_security",    1,   NULL, 'y' },
            { "enable_ranging",     1,   NULL, 'g' },
            { "exitflag",           1,   NULL, 'z' },
            { "transaction_id",     1,   NULL, 1005 },
            { "indication_cfg",     1,   NULL, 1006 },
            { "ranging_interval_msec",  1,   NULL, 1007 },
            { "cfg_ranging_ind",        1,   NULL, 1008 },
            { "cfg_dist_ingress_mm",    1,   NULL, 1009 },
            { "cfg_dist_egress_mm",     1,   NULL, 1010 },
            { "ranging_auto_resp_cfg",  1,   NULL, 1011 },
            { "cfg_range_result",       1,   NULL, 1012 },
            { "range_to_peer",          1,   NULL, 1013 },
            { "range_response",         1,   NULL, 1014 },
            { "sdea_service_specific_info",     1,   NULL, 1015 },
            { "passphrase",             1,   NULL, 1016},
            { "qos_required",           1,   NULL, 1017},
            { "ipv6_address",           1,   NULL, 1018},
            { "trans_port",             1,   NULL, 1019},
            { "trans_proto",            1,   NULL, 1020},
            { NULL,                 0,   NULL,  0  }/* Required at end of array. */
        };

        NanPublishRequest req;
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
        NdpIpTransParams ndpIpTransParam;
#endif
        transaction_id id = 0;
        const char* def_svc = DEFAULT_SVC;
        int size= 0;
        int period;
        memset(&req, 0, sizeof(NanPublishRequest));
        req.ttl = 0;
        req.period = 1;
        req.publish_type = NAN_PUBLISH_TYPE_UNSOLICITED;
        req.tx_type = NAN_TX_TYPE_BROADCAST;
        req.publish_count = 0;
        strlcpy((char*)req.service_name, def_svc, NAN_MAX_SERVICE_NAME_LEN);
        req.service_name_len = strlen((char*)req.service_name);

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
               case 'l' :
                   req.publish_id = atoi(optarg);
                   break;
               case 't' :
                   req.ttl = atoi(optarg);
                   break;
               case 'p' :
                   /* period: Awake DW Interval for publish(service)
                      Indicates the interval between two Discovery Windows in which
                      the device supporting the service is awake to transmit or
                      receive corresponding the Service Discovery frames.
                      The unit is 512TU.
                      Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16.
                   */
                   period = atoi(optarg);
                   if (period && !(period & (period - 1)) && period <= 16) {
                       req.period = period;
                   } else {
                       fprintf(stderr, "%s: Invalid period value %d.\n"
                               "Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16.\n",
                               __FUNCTION__, period);
                       return;
                   }
                   break;
               case 'r' :
                   req.publish_match_indicator = (NanMatchAlg)atoi(optarg);
                   break;
               case 'u' :
                   req.publish_type  = (NanPublishType)atoi(optarg);
                   break;
               case 'x' :
                   req.tx_type = (NanTxType)atoi(optarg);
                   break;
               case 'b' :
                   req.publish_count = atoi(optarg);
                   break;
               case 's' :
                   strlcpy((char*)req.service_name, optarg, NAN_MAX_SERVICE_NAME_LEN);
                   req.service_name_len = strlen((char*)req.service_name);
                   fprintf(stderr, "%s: Name %s.\n",
                           __FUNCTION__, req.service_name);
                   break;
               case 'i' :
                   strlcpy((char*)req.service_specific_info, optarg,
                            NAN_MAX_SERVICE_SPECIFIC_INFO_LEN);
                   req.service_specific_info_len = strlen((char*)req.service_specific_info);
                   break;
               case 'a' :
                   size = NAN_MAX_MATCH_FILTER_LEN;
                   nanParseHexString(optarg, &req.rx_match_filter[0],
                                     (int*)&size);
                   req.rx_match_filter_len = size;
                   break;
               case 'c' :
                   size = NAN_MAX_MATCH_FILTER_LEN;
                   nanParseHexString(optarg, &req.tx_match_filter[0],
                                     (int*)&size);
                   req.tx_match_filter_len = size;
                   break;
               case 'd':
                   req.rssi_threshold_flag = atoi(optarg);
                   break;
               case 'e':
                   size = sizeof(req.connmap);
                   nanParseHexString(optarg, &req.connmap, &size);
                   break;
               case 'n':
                   req.sdea_params.config_nan_data_path = 1;
                   req.sdea_params.ndp_type = (NdpType)atoi(optarg);
                   break;
               case 'k':
                   req.cipher_type = atoi(optarg);
                   break;
               case 'q':
                    req.service_responder_policy = (NanServiceAcceptPolicy)atoi(optarg);
                    break;
               case 'm':
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PMK;
                   size = (int)strlen(optarg) >> 1;
                   if(size > NAN_PMK_INFO_LEN) {

                       fprintf(stderr, "%s:Failed:PMK Length exceeded = %d \n",
                               __FUNCTION__, size);
                       return;
                   }
                   nanParseHexString(optarg, &req.key_info.body.pmk_info.pmk[0],
                                     (int*)&size);
                   req.key_info.body.pmk_info.pmk_len = size;
                   break;
               case 'y':
                   req.sdea_params.security_cfg =
                                    (NanDataPathSecurityCfgStatus)atoi(optarg);
                   break;
               case 'g':
                   req.sdea_params.ranging_state = (NanRangingState)atoi(optarg);
                   break;
               case 'z':
                   gExitflag = atoi(optarg);
                   break;
               case 1005:
                   id = atoi(optarg);
                   break;
               case 1006:
                   req.recv_indication_cfg = atoi(optarg);
                   break;
               case 1007:
                   req.ranging_cfg.ranging_interval_msec = atoi(optarg);
                   break;
               case 1008:
                   req.ranging_cfg.config_ranging_indications = atoi(optarg);
                   break;
               case 1009:
                   req.ranging_cfg.distance_ingress_mm = atoi(optarg);
                   break;
               case 1010:
                   req.ranging_cfg.distance_egress_mm = atoi(optarg);
                   break;
               case 1011:
                   req.ranging_auto_response = (NanRangingAutoResponse)atoi(optarg);
                   break;
               case 1012:
                   req.sdea_params.range_report = (NanRangeReport)atoi(optarg);
                   break;
               case 1013:
                   nanParseMacAddress(optarg, req.range_response_cfg.peer_addr);
                   break;
               case 1014:
                   req.range_response_cfg.ranging_response = (NanRangeResponse)atoi(optarg);
                   break;
               case 1015 :
                   strlcpy((char*)req.sdea_service_specific_info, optarg,
                            NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN);
                   req.sdea_service_specific_info_len = strlen((char*)req.sdea_service_specific_info);
                   break;
               case 1016:
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PASSPHRASE;
                   req.key_info.body.passphrase_info.passphrase_len = strlen((char*)optarg);
                   if (req.key_info.body.passphrase_info.passphrase_len > NAN_SECURITY_MAX_PASSPHRASE_LEN)
                   {
                       fprintf(stderr, "%s:Failed:passphrase length exceeded = %d \n",
                               __FUNCTION__, req.key_info.body.passphrase_info.passphrase_len);
                       return;
                   }
                   memcpy((char*)req.key_info.body.passphrase_info.passphrase,
                           optarg, req.key_info.body.passphrase_info.passphrase_len);
                   fprintf(stderr, "%s: passphrase = %s and passphrase_len = %d.\n",
                           __FUNCTION__, (char*)optarg,
                           req.key_info.body.passphrase_info.passphrase_len);
                   break;
               case 1017:
                   req.sdea_params.qos_cfg = (NanQosCfgStatus)atoi(optarg);
                   break;
               case 1018:
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
                   nanParseIPV6Address(optarg, ndpIpTransParam.ipv6_intf_addr);
                   ndpIpTransParam.ipv6_addr_present = 1;
#else
                   fprintf(stderr, "%s: IPV6 is not supported in this version of HAL\n"
                           "current HAL Version is %d.%d, required version is 2.1 or above\n",
                           __FUNCTION__, NAN_MAJOR_VERSION, NAN_MINOR_VERSION);
#endif
                   break;
                case 1019:
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
                   ndpIpTransParam.transport_port = atoi(optarg);
                   ndpIpTransParam.trans_port_present = 1;
#else
                   fprintf(stderr, "%s: transport port is not supported in this version of HAL\n"
                           "current HAL Version is %d.%d, required version is 2.1 or above\n",
                           __FUNCTION__, NAN_MAJOR_VERSION, NAN_MINOR_VERSION);
#endif
                   break;
                case 1020:
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
                   ndpIpTransParam.transport_protocol = atoi(optarg);
                   ndpIpTransParam.trans_proto_present = 1;
#else
                   fprintf(stderr, "%s: transport protocol is not supported in this version of HAL\n"
                           "current HAL Version is %d.%d, required version is 2.1 or above\n",
                           __FUNCTION__, NAN_MAJOR_VERSION, NAN_MINOR_VERSION);
#endif
                   break;
               case 'h':
               default:
                   nanPrintCmdUsage(argv, NAN_CMD, NAN_PUBLISH, long_options,
                                    sizeof(long_options)/sizeof(struct option));
                   return;
            }
        }

        fprintf(stderr, "%s: service_name %s len %d ssi %s ssi_len %d \n",
            __FUNCTION__, req.service_name, req.service_name_len,
            req.service_specific_info, req.service_specific_info_len);
        nan_publish_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendDisableRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hz:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "exitflag",     1,   NULL, 'z' },
            { "transaction_id",     1,   NULL, 1005 },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
        };
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        transaction_id id = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                   id = atoi(optarg);
                   break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_DISABLE, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_disable_request(id, ifaceHandle);
    }

    void NanTestSuite::nanSendPublishCancelRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "ht:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "publish_id",   1,   NULL, 't' },
            { "transaction_id",   1,   NULL, 1005 },
            { "exitflag",     1,   NULL, 'z' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
        };
        NanPublishCancelRequest req;
        memset(&req, 0, sizeof(NanPublishCancelRequest));
        transaction_id id = 0;

        req.publish_id = 1;
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 't' :
                    req.publish_id = atoi(optarg);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_PUBLISH_CANCEL, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_publish_cancel_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendSubscribeRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hl:t:p:s:f:g:j:k:m:c:n:i:a:b:r:d:e:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",               0,   NULL, 'h' },
            { "subscribe_id",       1,   NULL, 'l' },
            { "ttl",                1,   NULL, 't' },
            { "period",             1,   NULL, 'p' },
            { "subscribe_type",     1,   NULL, 's' },
            { "srvrspfilter",       1,   NULL, 'f' },
            { "srvrspincbit",       1,   NULL, 'g' },
            { "usesrvrspfilter",    1,   NULL, 'j' },
            { "ssiinfoneeded",      1,   NULL, 'k' },
            { "subscribe_match",    1,   NULL, 'm' },
            { "subscribe_count",    1,   NULL, 'c' },
            { "service_name",       1,   NULL, 'n' },
            { "ssi",                1,   NULL, 'i' },
            { "rx_match_filter",    1,   NULL, 'a' },
            { "tx_match_filter",    1,   NULL, 'b' },
            { "rssi_threshold",     1,   NULL, 'r' },
            { "connmap",            1,   NULL, 'd' },
            { "ndp_type",           1,   NULL, 'y' },
            { "csid",               1,   NULL, 'o' },
            { "pmk",                1,   NULL, 'q' },
            { "enable_security",    1,   NULL, 'u' },
            { "enable_ranging",     1,   NULL, 'v' },
            { "interface_addr_list",1,   NULL, 'e' },
            { "exitflag",           1,   NULL, 'z' },
            { "transaction_id",     1,   NULL, 1005 },
            { "indication_cfg",     1,   NULL, 1006 },
            { "ranging_interval_msec",  1,   NULL, 1007 },
            { "cfg_ranging_ind",        1,   NULL, 1008 },
            { "cfg_dist_ingress_mm",    1,   NULL, 1009 },
            { "cfg_dist_egress_mm",     1,   NULL, 1010 },
            { "ranging_auto_resp_cfg",  1,   NULL, 1011 },
            { "cfg_range_result",       1,   NULL, 1012 },
            { "range_to_peer",          1,   NULL, 1013 },
            { "range_response",         1,   NULL, 1014 },
            { "requestor_instance_id",  1,   NULL, 1015 },
            { "sdea_service_specific_info",     1,   NULL, 1016 },
            { "passphrase",             1,   NULL, 1017},
            { "qos_required",           1,   NULL, 1018},
            { "ipv6_address",           1,   NULL, 1019},
            { "trans_port",             1,   NULL, 1020},
            { "trans_proto",            1,   NULL, 1021},
            { NULL,                 0,   NULL,  0  }   /* Required at end of array.  */
        };

        NanSubscribeRequest req;
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
        NdpIpTransParams ndpIpTransParam;
#endif
        transaction_id id = 0;
        const char* def_svc = DEFAULT_SVC;
        int size = 0;
        int period;
        memset(&req, 0, sizeof(NanSubscribeRequest));
        req.ttl = 0;
        req.period =  1;
        req.subscribe_type = NAN_SUBSCRIBE_TYPE_ACTIVE;
        req.serviceResponseFilter = NAN_SRF_ATTR_BLOOM_FILTER;
        req.serviceResponseInclude = NAN_SRF_INCLUDE_RESPOND;
        req.ssiRequiredForMatchIndication = NAN_SSI_NOT_REQUIRED_IN_MATCH_IND;
        req.subscribe_match_indicator = NAN_MATCH_ALG_MATCH_CONTINUOUS;
        req.subscribe_count = 0;
        strlcpy((char*)req.service_name, def_svc, NAN_MAX_SERVICE_NAME_LEN);
        req.service_name_len = strlen((char*)req.service_name);

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
               case 'l' :
                   req.subscribe_id = atoi(optarg);
                   break;
               case 't' :
                   req.ttl = atoi(optarg);
                   break;
               case 'p' :
                   /* period: Awake DW Interval for subscribe
                      Indicates the interval between two Discovery Windows in which
                      the device supporting the service is awake to transmit or
                      receive corresponding the Service Discovery frames.
                      The unit is 512TU.
                      Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16.
                   */
                   period = atoi(optarg);
                   if (period && !(period & (period - 1)) && period <= 16) {
                       req.period = period;
                   } else {
                       fprintf(stderr, "%s: Invalid period value %d.\n"
                               "Valid values of Awake DW Interval are: 1, 2, 4, 8 and 16.\n",
                               __FUNCTION__, period);
                       return;
                   }
                   break;
               case 's' :
                   req.subscribe_type  = (NanSubscribeType)atoi(optarg);
                   break;
               case 'f' :
                   req.serviceResponseFilter  = (NanSRFType)atoi(optarg);
                   break;
               case 'g' :
                   req.serviceResponseInclude  = (NanSRFIncludeType)atoi(optarg);
                   break;
               case 'j' :
                   req.useServiceResponseFilter  = (NanSRFState)atoi(optarg);
                   break;
               case 'k' :
                   req.ssiRequiredForMatchIndication  = (NanSsiInMatchInd)atoi(optarg);
                   break;
               case 'm' :
                   req.subscribe_match_indicator = (NanMatchAlg)atoi(optarg);
                   break;
               case 'c' :
                   req.subscribe_count = atoi(optarg);
                   break;
               case 'n' :
                   strlcpy((char*)req.service_name, optarg, NAN_MAX_SERVICE_NAME_LEN);
                   req.service_name_len = strlen((char*)req.service_name);
                   break;
               case 'i' :
                   strlcpy((char*)req.service_specific_info, optarg,
                            NAN_MAX_SERVICE_SPECIFIC_INFO_LEN);
                   req.service_specific_info_len = strlen((char*)req.service_specific_info);
                   break;
               case 'a' :
                   size = NAN_MAX_MATCH_FILTER_LEN;
                   nanParseHexString(optarg, &req.rx_match_filter[0],
                                     (int*)&size);
                   req.rx_match_filter_len = size;
                   break;
               case 'b' :
                   size = NAN_MAX_MATCH_FILTER_LEN;
                   nanParseHexString(optarg, &req.tx_match_filter[0],
                                     (int*)&size);
                   req.tx_match_filter_len = size;
                   break;
               case 'r':
                   req.rssi_threshold_flag = atoi(optarg);
                   break;
               case 'd':
                   size = sizeof(req.connmap);
                   nanParseHexString(optarg, &req.connmap, &size);
                   break;
               case 'e':
                   req.num_intf_addr_present = \
                       nanParseMacAddresslist(optarg, &req.intf_addr[0][0],
                                              NAN_MAX_SUBSCRIBE_MAX_ADDRESS);
                   break;
               case 'z':
                   gExitflag = atoi(optarg);
                   break;
               case 1005:
                    id = atoi(optarg);
                    break;
               case 1006:
                    req.recv_indication_cfg = atoi(optarg);
                    break;
               case 'y':
                   req.sdea_params.config_nan_data_path = 1;
                   req.sdea_params.ndp_type = (NdpType)atoi(optarg);
                   break;
               case 'o':
                   req.cipher_type = atoi(optarg);
                   break;
               case 'q':
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PMK;
                   size = (int)strlen(optarg) >> 1;
                   if(size > NAN_PMK_INFO_LEN) {

                       fprintf(stderr, "%s:Failed:PMK Length exceeded = %d \n",
                               __FUNCTION__, size);
                       return;
                   }
                   nanParseHexString(optarg, &req.key_info.body.pmk_info.pmk[0],
                                     (int*)&size);
                   req.key_info.body.pmk_info.pmk_len = size;
                   break;
               case 'u':
                   req.sdea_params.security_cfg =
                                    (NanDataPathSecurityCfgStatus)atoi(optarg);
                   break;
               case 'v':
                   req.sdea_params.ranging_state = (NanRangingState)atoi(optarg);
                   break;
               case 1007:
                   req.ranging_cfg.ranging_interval_msec = atoi(optarg);
                   break;
               case 1008:
                   req.ranging_cfg.config_ranging_indications = atoi(optarg);
                   break;
               case 1009:
                   req.ranging_cfg.distance_ingress_mm = atoi(optarg);
                   break;
               case 1010:
                   req.ranging_cfg.distance_egress_mm = atoi(optarg);
                   break;
               case 1011:
                   req.ranging_auto_response = (NanRangingAutoResponse)atoi(optarg);
                   break;
               case 1012:
                   req.sdea_params.range_report = (NanRangeReport)atoi(optarg);
                   break;
               case 1013:
                   nanParseMacAddress(optarg, req.range_response_cfg.peer_addr);
                   break;
               case 1014:
                   req.range_response_cfg.ranging_response = (NanRangeResponse)atoi(optarg);
                   break;
               case 1015 :
               {
                    u32 requestor_instance_id = 0;
                    size_t size = sizeof(requestor_instance_id);
                    nanParseHexString(optarg, (u8*)&requestor_instance_id,
                                     (int*)&size);
                    req.range_response_cfg.requestor_instance_id = ntohl(requestor_instance_id);
                    break;
               }
               case 1016:
                   strlcpy((char*)req.sdea_service_specific_info, optarg,
                            NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN);
                   req.sdea_service_specific_info_len = strlen((char*)req.sdea_service_specific_info);
                   break;
               case 1017:
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PASSPHRASE;
                   req.key_info.body.passphrase_info.passphrase_len = strlen((char*)optarg);
                   if (req.key_info.body.passphrase_info.passphrase_len > NAN_SECURITY_MAX_PASSPHRASE_LEN)
                   {
                       fprintf(stderr, "%s:Failed:passphrase length exceeded = %d \n",
                               __FUNCTION__, req.key_info.body.passphrase_info.passphrase_len);
                       return;
                   }
                   memcpy((char*)req.key_info.body.passphrase_info.passphrase,
                           optarg, req.key_info.body.passphrase_info.passphrase_len);
                   fprintf(stderr, "%s: passphrase = %s and passphrase_len = %d.\n",
                           __FUNCTION__, (char*)optarg,
                           req.key_info.body.passphrase_info.passphrase_len);
                   break;
               case 1018:
                   req.sdea_params.qos_cfg = (NanQosCfgStatus)atoi(optarg);
                   break;
                case 1019:
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
                   nanParseIPV6Address(optarg, ndpIpTransParam.ipv6_intf_addr);
                   ndpIpTransParam.ipv6_addr_present = 1;
#else
                   fprintf(stderr, "%s: IPV6 is not supported in this version of HAL\n"
                           "current HAL Version is %d.%d, required version is 2.1 or above\n",
                           __FUNCTION__, NAN_MAJOR_VERSION, NAN_MINOR_VERSION);
#endif
                   break;
                case 1020:
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
                   ndpIpTransParam.transport_port = atoi(optarg);
                   ndpIpTransParam.trans_port_present = 1;
#else
                   fprintf(stderr, "%s: transport port is not supported in this version of HAL\n"
                           "current HAL Version is %d.%d, required version is 2.1 or above\n",
                           __FUNCTION__, NAN_MAJOR_VERSION, NAN_MINOR_VERSION);
#endif
                   break;
                case 1021:
#if ((NAN_MAJOR_VERSION > 2) || \
        (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)) && \
        NAN_CERT_VERSION >= 5
                   ndpIpTransParam.transport_protocol = atoi(optarg);
                   ndpIpTransParam.trans_proto_present = 1;
#else
                   fprintf(stderr, "%s: transport protocol is not supported in this version of HAL\n"
                           "current HAL Version is %d.%d, required version is 2.1 or above\n",
                           __FUNCTION__, NAN_MAJOR_VERSION, NAN_MINOR_VERSION);
#endif
                   break;

               case 'h':
               default:
                   nanPrintCmdUsage(argv, NAN_CMD, NAN_SUBSCRIBE, long_options,
                                    sizeof(long_options)/sizeof(struct option));
                   return;
            }
        }
        fprintf(stderr, "%s: service_name %s len %d ssi %s ssi_len %d \n",
            __FUNCTION__, req.service_name, req.service_name_len,
            req.service_specific_info, req.service_specific_info_len);
        nan_subscribe_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendSubscribeCancelRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "ht:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "subscribe_id", 1,   NULL, 't' },
            { "exitflag",     1,   NULL, 'z' },
            { "transaction_id",         1,   NULL, 1005 },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
        };
        NanSubscribeCancelRequest req;
        memset(&req, 0, sizeof(NanSubscribeCancelRequest));
        transaction_id id = 0;

        req.subscribe_id = 128;
        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 't' :
                    req.subscribe_id = atoi(optarg);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_SUBSCRIBE_CANCEL, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_subscribe_cancel_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendConfigRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hp:a:b:c:k:m:d:n:v:w:x:y:z:u:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",                   0,   NULL, 'h' },
            { "master_pref",            1,   NULL, 'p' },
            { "2.4G_rssi_proximity",    1,   NULL, 'k' },
            { "5G_rssi_proximity",      1,   NULL, 'm' },
            { "rssi_window_val",        1,   NULL, 'n' },
            { "nan_socialchannel_param",1,   NULL, 'v' },
            { "nan_debugflags_param",   1,   NULL, 'w' },
            { "random_factor_force",    1,   NULL, 'x' },
            { "hop_count_force",        1,   NULL, 'y' },
            { "tx_conn_capability",     1,   NULL, 'z' },
            { "nan_cluster_attr_val",   1,   NULL, 'u' },
            { "sid_beacon",             1,   NULL, 1000 },
            { "tx_postdiscovery",       1,   NULL, 1001 },
            { "fam",                    1,   NULL, 1002 },
            { "exitflag",               1,   NULL, 1003 },
            { "transaction_id",         1,   NULL, 1005 },
            { "disc_mac_addr_rand_int", 1,   NULL, 1008 },
            { "cfg_discovery_ind",      1,   NULL, 1009 },
            { "subscribe_sid",          1,   NULL, 1010 },
            { "supported_bands",        1,   NULL, 'a'  },
            { "2.4G_dw_interval_val",   1,   NULL, 'b'  },
            { "5G_dw_interval_val",     1,   NULL, 'c'  },
            { "responder_mode",         1,   NULL, 'd'  },
            { "disc_bcn_int",           1,   NULL, 1011 },
            { "nss",                    1,   NULL, 1012 },
            { "enable_ranging",         1,   NULL, 1013 },
            { "enable_dw_termination",  1,   NULL, 1014 },
            { "ndpe_attr_supported",    1,   NULL, 1015 },
            { NULL,                     0,   NULL,  0   }/* Required at end of array.  */
        };

        int interval;
        NanConfigRequest req;
        memset(&req, 0, sizeof(NanConfigRequest));
        transaction_id id = 0;

        /*
           By default discovery MAC address randomization is enabled
           and default interval value is 30 minutes i.e. 1800 seconds
        */
        req.config_disc_mac_addr_randomization = 1;
        req.disc_mac_addr_rand_interval_sec = 1800;

        /* Override with command line arguements */
        int long_index = 0, opt = 0, idx = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
               case 'p' :
                   req.config_master_pref = 1;
                   req.master_pref = atoi(optarg);
                   break;
               case 'k':
                   req.config_rssi_proximity = 1;
                   req.rssi_proximity = atoi(optarg);
                   break;
               case 'm':
                   req.config_5g_rssi_close_proximity = 1;
                   req.rssi_close_proximity_5g_val = atoi(optarg);
                   break;
               case 'n':
                   req.config_rssi_window_size = 1;
                   req.rssi_window_size_val = atoi(optarg);
                   break;
               case 'v':
                   req.config_scan_params = 1;
                   nanParseSocialChannelParams(optarg, &req.scan_params_val);
                   break;
#if 0
               case 'w':
                   req.config_debug_flags = 1;
                   req.debug_flags_val = strtoull(optarg, NULL, 0);
                   break;
#endif
               case 'x':
                   req.config_random_factor_force = 1;
                   req.random_factor_force_val = atoi(optarg);
                   break;
               case 'y':
                   req.config_hop_count_force = 1;
                   req.hop_count_force_val = atoi(optarg);
                   break;
               case 'z':
                   req.config_conn_capability = 1;
                   nanParseTransmitPostConnectivityCapability(optarg,
                       &req.conn_capability_val);
                   break;
               case 'u':
                   req.config_cluster_attribute_val = atoi(optarg);
                   break;
               case 1000:
                   req.config_sid_beacon = 1;
                   req.sid_beacon = atoi(optarg);
                   break;
               case 1001:
                   idx = req.num_config_discovery_attr;
                   nanParseTransmitPostDiscovery(optarg,
                              &req.discovery_attr_val[idx]);
                   req.num_config_discovery_attr++;
                   break;
               case 1002:
                   req.config_fam = 1;
                   nanParseFurtherAvailabilityMap(optarg, &req.fam_val);
                   break;
                case 1003:
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 1008:
                  /*
                    By default discovery MAC address randomization is enabled
                    and maximum interval value is 30 minutes i.e. 1800 seconds
                    Nan enable and config can specify the interval value in seconds.
                    Value of zero disables discovery MAC address randomization.
                  */
                  req.config_disc_mac_addr_randomization = 1;
                  interval = atoi(optarg);
                  if (interval <= 1800) {
                       req.disc_mac_addr_rand_interval_sec = interval;
                  } else {
                       fprintf(stderr, "%s: Invalid discovery MAC address randomization interval %d.\n"
                               "Maximum interval value is 30 minutes i.e. 1800 seconds\n",
                               __FUNCTION__, interval);
                       return;
                  }
                  break;
               case 1009:
                  req.discovery_indication_cfg = atoi(optarg);
                  break;
               case 1010:
                   req.config_subscribe_sid_beacon = 1;
                   req.subscribe_sid_beacon_val = atoi(optarg);
                   break;
#if NAN_CERT_VERSION >= 4
               case 1011:
                   req.config_discovery_beacon_int = 1;
                   req.discovery_beacon_interval = atoi(optarg);
                  break;
               case 1012:
                   req.config_nss = 1;
                   req.nss = atoi(optarg);
                   break;
                case 1013:
                   req.config_enable_ranging = 1;
                   req.enable_ranging = atoi(optarg);
                   break;
                case 1014:
                   req.config_dw_early_termination = 1;
                   req.enable_dw_termination = atoi(optarg);
                   break;
#else
               case 1011:
               case 1012:
               case 1013:
               case 1014:
                   fprintf(stderr, "%s: disc_bcn_int, nss, ranging, dw termination not\n"
                           "supported in this version of HAL current HAL Version is %d,"
                           "required version is 4 or above\n", __func__, NAN_CERT_VERSION);
#endif
               case 1015:
#if (NAN_MAJOR_VERSION > 2) || (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)
                   req.config_ndpe_attr = 1;
                   req.use_ndpe_attr = atoi(optarg);
#else
                   fprintf(stderr, "%s: ndpe_attr_supported not supported in this version of HAL\n"
                           "current HAL Version is %d, required version is 4 or above\n",
                           __func__, NAN_CERT_VERSION);
#endif
                   break;
#ifdef WFA_CERT
               case 'a':
                   req.config_supported_bands = 1;
                   req.supported_bands_val = atoi(optarg);
                   break;
#endif
               case 'b':
                   req.config_dw.config_2dot4g_dw_band = 1;
                   req.config_dw.dw_2dot4g_interval_val = atoi(optarg);
                   break;
               case 'c':
                   req.config_dw.config_5g_dw_band = 1;
                   req.config_dw.dw_5g_interval_val = atoi(optarg);
                   break;
               case 'd':
#ifdef WFA_CERT
                   req.config_responder_mode = 1;
                   req.responder_mode_val = atoi(optarg);
                   break;
#endif
              case 'h':
               default:
                   nanPrintCmdUsage(argv, NAN_CMD, NAN_CONFIG, long_options,
                                    sizeof(long_options)/sizeof(struct option));
                   return;
            }
        }
        nan_config_request(id, ifaceHandle, &req);
    }

    int NanTestSuite::nanParseHex(unsigned char c)
    {
       if (c >= '0' && c <= '9')
          return c-'0';
       if (c >= 'a' && c <= 'f')
          return c-'a'+10;
       if (c >= 'A' && c <= 'F')
          return c-'A'+10;
       return 0;
    }

    int NanTestSuite::nanParseMacAddress(const char* arg, u8* addr)
    {
       if (strlen(arg) != 17)
       {
          fprintf(stderr, "Invalid mac address %s\n", arg);
          fprintf(stderr, "expected format xx:xx:xx:xx:xx:xx\n");
          return -1;
       }

       addr[0] = nanParseHex(arg[0]) << 4 | nanParseHex(arg[1]);
       addr[1] = nanParseHex(arg[3]) << 4 | nanParseHex(arg[4]);
       addr[2] = nanParseHex(arg[6]) << 4 | nanParseHex(arg[7]);
       addr[3] = nanParseHex(arg[9]) << 4 | nanParseHex(arg[10]);
       addr[4] = nanParseHex(arg[12]) << 4 | nanParseHex(arg[13]);
       addr[5] = nanParseHex(arg[15]) << 4 | nanParseHex(arg[16]);
       return 0;
    }

    void NanTestSuite::nanParseIPV6Address(const char* arg, u8* ipv6_addr)
    {
        int i, j;

        if (strlen(arg) != 39) {
            fprintf(stderr, "Invalid IPV6 address %s\n", arg);
            fprintf(stderr, "Expected format is fe80:0000:0000:0000:xxxx:xxxx:xxxx:xxxx\n");
            return;
        }

        i = 0;
        j = 0;
        while (i < (int)strlen(arg)) {
            ipv6_addr[j] = nanParseHex(arg[i]) << 4 | nanParseHex(arg[i+1]);
            j++;
            i+=2;
            if (arg[i] == ':')
                i++;
        }
        nanhexdump(ipv6_addr, j);
    }

    void NanTestSuite::nanSendTransmitFollowupRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "ht:a:p:d:i:m:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",                  0,   NULL, 'h' },
            { "publish_subscribe_id",  1,   NULL, 't' },
            { "addr",                  1,   NULL, 'a' },
            { "priority",              1,   NULL, 'p' },
            { "dw_or_faw",             1,   NULL, 'd' },
            { "ssi",                   1,   NULL, 'i' },
            { "requestor_instance_id", 1,   NULL, 'm' },
            { "transaction_id",        1,   NULL, 1005 },
            { "indication_cfg",        1,   NULL, 1006 },
            { "sdea_service_specific_info",     1,   NULL, 1007 },
            { "exitflag",              1,   NULL, 'z' },
            { NULL,                    0,   NULL,  0  }/* Required at end of array.  */
        };

        NanTransmitFollowupRequest req;
        memset(&req, 0, sizeof(NanTransmitFollowupRequest));
        transaction_id id = 0;
        req.addr[0] = 0xFF;
        req.addr[1] = 0xFF;
        req.addr[2] = 0xFF;
        req.addr[3] = 0xFF;
        req.addr[4] = 0xFF;
        req.addr[5] = 0xFF;
        req.priority = NAN_TX_PRIORITY_NORMAL;
        req.dw_or_faw = NAN_TRANSMIT_IN_DW;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 't' :
                    req.publish_subscribe_id = atoi(optarg);
                    break;
                case 'a' :
                    nanParseMacAddress(optarg, req.addr);
                    break;
                case 'p' :
                    req.priority = (NanTxPriority)atoi(optarg);
                    break;
                case 'd' :
                    req.dw_or_faw = (NanTransmitWindowType)atoi(optarg);
                    break;
                case 'i' :
                    strlcpy((char*)req.service_specific_info, optarg,
                            NAN_MAX_SERVICE_SPECIFIC_INFO_LEN);
                    req.service_specific_info_len = strlen((char*)req.service_specific_info);
                    break;
                case 'm' :
                {
                    u32 requestor_instance_id = 0;
                    size_t size = sizeof(requestor_instance_id);
                    nanParseHexString(optarg, (u8*)&requestor_instance_id,
                                     (int*)&size);
                    req.requestor_instance_id = ntohl(requestor_instance_id);
                    break;
                }
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 1006:
                    req.recv_indication_cfg = atoi(optarg);
                    break;
                case 1007 :
                   strlcpy((char*)req.sdea_service_specific_info, optarg,
                            NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN);
                   req.sdea_service_specific_info_len = strlen((char*)req.sdea_service_specific_info);
                   break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_FOLLOWUP, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        fprintf(stderr, "%s: ssi %s ssi_len %d " MAC_ADDR_STR "\n",
            __FUNCTION__, req.service_specific_info,
            req.service_specific_info_len, MAC_ADDR_ARRAY(req.addr));

        nan_transmit_followup_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendStatsRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:c:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "stats_id",     1,   NULL, 'i' },
            { "clear",        1,   NULL, 'c' },
            { "transaction_id",1,   NULL, 1005 },
            { "exitflag",     1,   NULL, 'z' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
        };
        NanStatsRequest req;
        memset(&req, 0, sizeof(NanStatsRequest));
        transaction_id id = 0;
        req.stats_type = NAN_STATS_ID_DE_PUBLISH;
        req.clear = 0;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'i' :
                    req.stats_type = (NanStatsType)atoi(optarg);
                    break;
                case 'c' :
                    req.clear = atoi(optarg);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_STATS, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_stats_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendTCARequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:r:f:c:t:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",         0,   NULL, 'h' },
            { "tca_id",       1,   NULL, 'i' },
            { "rising",       1,   NULL, 'r' },
            { "falling",      1,   NULL, 'f' },
            { "clear",        1,   NULL, 'c' },
            { "threshold",    1,   NULL, 't' },
            { "transaction_id",1,  NULL, 1005 },
            { "exitflag",     1,   NULL, 'z' },
            { NULL,           0,   NULL,  0  }   /* Required at end of array.  */
        };
        NanTCARequest req;
        memset(&req, 0, sizeof(NanTCARequest));
        transaction_id id = 0;
        req.tca_type = NAN_TCA_ID_CLUSTER_SIZE;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'i' :
                    req.tca_type = (NanTcaType)atoi(optarg);
                    break;
                case 'r' :
                    req.rising_direction_evt_flag = atoi(optarg);
                    break;
                case 'f' :
                    req.falling_direction_evt_flag = atoi(optarg);
                    break;
                case 'c' :
                    req.clear = atoi(optarg);
                    break;
                case 't' :
                    req.threshold = atoi(optarg);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_TCA, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_tca_request(id, ifaceHandle, &req);
    }

    void NanTestSuite::nanSendBeaconSdfRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "ht:p:f:o:v:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",           0,   NULL, 'h' },
            { "payload_tx_flag",1,   NULL, 'p' },
            { "tx_in_flag",     1,   NULL, 'f' },
            { "vendor_oui",     1,   NULL, 'o' },
            { "vsa",            1,   NULL, 'v' },
            { "transaction_id", 1,   NULL, 1005 },
            { "exitflag",       1,   NULL, 'z' },
            { NULL,             0,   NULL,  0  }   /* Required at end of array.  */
        };

        NanBeaconSdfPayloadRequest req;
        memset(&req, 0, sizeof(NanBeaconSdfPayloadRequest));
        transaction_id id = 0;
        req.vsa.payload_transmit_flag = 1;
        req.vsa.tx_in_discovery_beacon = 1;
        req.vsa.tx_in_service_discovery = 1;
        req.vsa.tx_in_sync_beacon = 1;
        req.vsa.vendor_oui = 0x001374;
        strlcpy((char*)req.vsa.vsa, "Qualcomm", NAN_MAX_VSA_DATA_LEN);
        req.vsa.vsa_len = strlen((char*)req.vsa.vsa);

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'p' :
                    req.vsa.payload_transmit_flag = atoi(optarg);
                    break;
                case 'f' :
                    req.vsa.tx_in_discovery_beacon = (atoi(optarg) & 0x01);
                    req.vsa.tx_in_service_discovery = (atoi(optarg) & 0x02) >> 1;
                    req.vsa.tx_in_sync_beacon = (atoi(optarg) & 0x04) >> 2;
                    break;
                case 'o' :
                {
                    req.vsa.vendor_oui = atoi(optarg);
                    u32 oui = 0;
                    size_t size = sizeof(oui);
                    nanParseHexString(optarg, (u8*)&oui,
                                      (int*)&size);
                    req.vsa.vendor_oui = ntohl(oui);
                    break;
                }
                case 'v' :
                    strlcpy((char*)req.vsa.vsa, optarg, NAN_MAX_VSA_DATA_LEN);
                    req.vsa.vsa_len = strlen((char*)req.vsa.vsa);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_BEACONSDF, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        fprintf(stderr, "%s: oui 0x%08x vsa_len %d vsa %s \n",
            __FUNCTION__, req.vsa.vendor_oui,
            req.vsa.vsa_len, req.vsa.vsa);

        nan_beacon_sdf_payload_request(0, ifaceHandle, &req);
    }

    void NanTestSuite::nanGetStaParameter(int argc, char **argv)
    {
        UNUSED(argc), UNUSED(argv);
        NanStaParameter rsp;
        wifi_error ret;
        memset(&rsp, 0, sizeof(NanStaParameter));
        ret = nan_get_sta_parameter(0, ifaceHandle, &rsp);
        if(ret == WIFI_ERROR_NONE)
        {
            fprintf(stderr, "%s: NanStaparameter Master_pref:0x%02x," \
                " Random_factor:0x%02x, hop_count:0x%02x " \
                " beacon_transmit_time:0x%08x " \
                " ndp_channel_freq:%d \n",
                __FUNCTION__,
                rsp.master_pref,
                rsp.random_factor,
                rsp.hop_count,
                rsp.beacon_transmit_time,
                rsp.ndp_channel_freq);
        }
        else
        {
            fprintf(stderr, "%s: Error %d", __FUNCTION__,
                    ret);
        }
    }

    void NanTestSuite::nanGetCapabilities(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:c:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",           0,   NULL, 'h'  },
            { "transaction_id", 1,   NULL, 1005 },
            { "exitflag",       1,   NULL, 'z'  },
            { NULL,             0,   NULL,  0   }   /* Required at end of array.  */
        };
        transaction_id id = 0;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_GETCAPABILITIES, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_get_capabilities(id, ifaceHandle);
    }

    void NanTestSuite::nanSendDebugRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "ha:d:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",             0,   NULL, 'h'},
            { "debug_cmd",        1,   NULL, 'd'},
            { "exitflag",         1,   NULL, 'z'},
            { NULL,               0,   NULL,  0 }   /* Required at end of array.  */
        };
        transaction_id id = 0;
        NanDebugParams cfg_debug;
        memset(&cfg_debug, 0, sizeof(NanDebugParams));
        int size = 0;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 'd':
                    //size = NAN_MAX_DEBUG_MESSAGE_LEN;
                    size = sizeof(NanDebugParams);
                    nanParseHexString(optarg, (u8*)&cfg_debug,
                                     (int*)&size);
                    fprintf(stderr, "%s: Debug Command = 0x%x \n",__FUNCTION__,
                                                                 cfg_debug.cmd);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NAN_DEBUG, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_debug_command_config(id, ifaceHandle, cfg_debug, size);
    }

    void NanTestSuite::ndpInterfaceCreate(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:z:";
        char iface_name[IFNAMSIZ + 1];
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",           0,   NULL,  'h' },
            { "iface_name",     1,   NULL,  'i' },
            { "transaction_id", 1,   NULL,  1005},
            { "exitflag",       1,   NULL,  'z' },
            { NULL,             0,   NULL,   0  }   /* Required at end of array.  */
        };
        transaction_id id = 0;
        wifi_error rc = WIFI_SUCCESS;

        memset(iface_name, 0, sizeof(iface_name));

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 'i':
                   strlcpy((char*)iface_name, optarg, sizeof(iface_name));
                   fprintf(stderr, "%s: Interface Name %s.\n",
                           __FUNCTION__, iface_name);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NDI_CREATE,
                    long_options, sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        if (*iface_name) {
            rc = nan_data_interface_create(id, ifaceHandle, iface_name);
            if (rc != WIFI_SUCCESS) {
                fprintf(stderr, "%s: Error returned=%d, Exiting....\n",
                        __FUNCTION__, rc);
                exit(0);
            }
        } else {
            nanPrintCmdUsage(argv, NAN_CMD, NDI_CREATE,
                             long_options,
                             sizeof(long_options)/sizeof(struct option));
            return;
       }
    }

    void NanTestSuite::ndpInterfaceDelete(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:z:";
        char iface_name[IFNAMSIZ + 1];
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",           0,  NULL, 'h' },
            { "iface_name",     1,  NULL, 'i' },
            { "transaction_id", 1,  NULL, 1005},
            { "exitflag",       1,  NULL, 'z' },
            { NULL,             0,  NULL,  0  }   /* Required at end of array.  */
        };
        transaction_id id = 0;
        wifi_error rc = WIFI_SUCCESS;

        memset(iface_name, 0, sizeof(iface_name));

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 'i':
                   strlcpy((char*)iface_name, optarg, sizeof(iface_name));
                   fprintf(stderr, "%s: Interface Name %s.\n",
                           __FUNCTION__, iface_name);
                   break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NDI_DELETE, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        if (*iface_name) {
            rc = nan_data_interface_delete(id, ifaceHandle, iface_name);
            if (rc != WIFI_SUCCESS) {
                fprintf(stderr, "%s: Error returned=%d, Exiting....\n",
                        __FUNCTION__, rc);
                exit(0);
            }
        } else {
            nanPrintCmdUsage(argv, NAN_CMD, NDI_DELETE,
                             long_options,
                             sizeof(long_options)/sizeof(struct option));
            return;
       }
    }

    void NanTestSuite::ndpInitiatorRequest(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:a:c:d:j:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",               0,   NULL, 'h' },
            { "peer_addr",          1,   NULL, 'a' },
            { "ndp_channel_cfg",    1,   NULL, 'c' },
            { "ndp_channel",        1,   NULL, 'l' },
            { "service_instance_id",1,   NULL, 'd' },
            { "ndp_iface",          1,   NULL, 'i' },
            { "app_info",           1,   NULL, 'j' },
            { "transaction_id",     1,   NULL, 1005},
            { "csid",               1,   NULL, 'k' },
            { "pmk",                1,   NULL, 'm' },
            { "passphrase",         1,   NULL, 1001},
            { "service_name",       1,   NULL, 1002},
            { "exitflag",           1,   NULL, 'z' },
            { NULL,                 0,   NULL,  0  }   /* Required at end of array.  */
        };
        transaction_id id = 0;
        NanDataPathInitiatorRequest req;
        int size = 0;

        memset(&req, 0, sizeof(NanDataPathInitiatorRequest));
        req.peer_disc_mac_addr[0] = 0xFF;
        req.peer_disc_mac_addr[1] = 0xFF;
        req.peer_disc_mac_addr[2] = 0xFF;
        req.peer_disc_mac_addr[3] = 0xFF;
        req.peer_disc_mac_addr[4] = 0xFF;
        req.peer_disc_mac_addr[5] = 0xFF;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'a' :
                    nanParseMacAddress(optarg, req.peer_disc_mac_addr);
                    break;
                case 'd':
                    req.requestor_instance_id = atoi(optarg);
                    break;
                case 'c':
                    req.channel_request_type = (NanDataPathChannelCfg)atoi(optarg);
                    break;
                case 'i':
                   strlcpy((char*)req.ndp_iface, optarg, (IFNAMSIZ + 1));
                   fprintf(stderr, "%s: Interface Name %s.\n",
                           __FUNCTION__, req.ndp_iface);
                   break;
                case 'j':
                   strlcpy((char*)req.app_info.ndp_app_info, optarg,
                           NAN_DP_MAX_APP_INFO_LEN);
                   req.app_info.ndp_app_info_len = strlen((char*)req.app_info.ndp_app_info);
                   fprintf(stderr, "%s: App Info %s.\n", __FUNCTION__, req.app_info.ndp_app_info);
                   break;
                case 'l':
                    req.channel = atoi(optarg);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'k':
                    req.cipher_type = atoi(optarg);
                    break;
                case 'm':
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PMK;
                   size = (int)strlen(optarg) >> 1;
                   if(size > NAN_PMK_INFO_LEN) {

                       fprintf(stderr, "%s:Failed:PMK Length exceeded = %d \n",
                               __FUNCTION__, size);
                       return;
                   }
                   nanParseHexString(optarg, &req.key_info.body.pmk_info.pmk[0],
                                     (int*)&size);
                   req.key_info.body.pmk_info.pmk_len = size;
                    break;
               case 1001:
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PASSPHRASE;
                   req.key_info.body.passphrase_info.passphrase_len = strlen((char*)optarg);
                   if (req.key_info.body.passphrase_info.passphrase_len > NAN_SECURITY_MAX_PASSPHRASE_LEN)
                   {
                       fprintf(stderr, "%s:Failed:passphrase length exceeded = %d \n",
                               __FUNCTION__, req.key_info.body.passphrase_info.passphrase_len);
                       return;
                   }
                   memcpy((char*)req.key_info.body.passphrase_info.passphrase,
                           optarg, req.key_info.body.passphrase_info.passphrase_len);
                   fprintf(stderr, "%s: passphrase = %s and passphrase_len = %d.\n",
                           __FUNCTION__, (char*)optarg,
                           req.key_info.body.passphrase_info.passphrase_len);
                   break;
                case 1002:
                   strlcpy((char*)req.service_name, optarg, NAN_MAX_SERVICE_NAME_LEN);
                   req.service_name_len = strlen((char*)req.service_name);
                   fprintf(stderr, "%s: service_name = %s and service_name_len = %d.\n",
                           __FUNCTION__, req.service_name, req.service_name_len);
                   break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NDP_REQUEST, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_data_request_initiator(id, ifaceHandle, &req);
    }

    void NanTestSuite::ndpIndicationResponse(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:d:j:k:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",               0,   NULL, 'h' },
            { "ndp_instance_id",    1,   NULL, 'd' },
            { "ndp_iface",          1,   NULL, 'i' },
            { "app_info",           1,   NULL, 'j' },
            { "ndp_rsp_code",       1,   NULL, 'k' },
            { "transaction_id",     1,   NULL, 1005},
            { "csid",               1,   NULL, 'c' },
            { "pmk",                1,   NULL, 'm' },
            { "passphrase",         1,   NULL, 1001},
            { "service_name",       1,   NULL, 1002},
            { "exitflag",           1,   NULL, 'z' },
            { NULL,                 0,   NULL,  0  }   /* Required at end of array.  */
        };
        transaction_id id = 0;
        NanDataPathIndicationResponse req;
        int size = 0;

        memset(&req, 0, sizeof(NanDataPathIndicationResponse));

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'd':
                    req.ndp_instance_id = atoi(optarg);
                    break;
                case 'i':
                    strlcpy((char*)req.ndp_iface, optarg, (IFNAMSIZ+ 1));
                    fprintf(stderr, "%s: Interface Name %s.\n",
                           __FUNCTION__, req.ndp_iface);
                    break;
                case 'j':
                    strlcpy((char*)req.app_info.ndp_app_info, optarg,
                           NAN_DP_MAX_APP_INFO_LEN);
                    req.app_info.ndp_app_info_len = strlen((char*)req.app_info.ndp_app_info);
                   fprintf(stderr, "%s: App Info %s.\n", __FUNCTION__, req.app_info.ndp_app_info);
                    break;
                case 'k':
                    req.rsp_code = (NanDataPathResponseCode)atoi(optarg);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'c':
                    req.cipher_type = atoi(optarg);
                    break;
                case 'm':
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PMK;
                   size = (int)strlen(optarg) >> 1;
                   if(size > NAN_PMK_INFO_LEN) {

                       fprintf(stderr, "%s:Failed:PMK Length exceeded = %d \n",
                               __FUNCTION__, size);
                       return;
                   }
                   nanParseHexString(optarg, &req.key_info.body.pmk_info.pmk[0],
                                     (int*)&size);
                   req.key_info.body.pmk_info.pmk_len = size;
                    break;
               case 1001:
                   req.key_info.key_type = NAN_SECURITY_KEY_INPUT_PASSPHRASE;
                   req.key_info.body.passphrase_info.passphrase_len = strlen((char*)optarg);
                   if (req.key_info.body.passphrase_info.passphrase_len > NAN_SECURITY_MAX_PASSPHRASE_LEN)
                   {
                       fprintf(stderr, "%s:Failed:passphrase length exceeded = %d \n",
                               __FUNCTION__, req.key_info.body.passphrase_info.passphrase_len);
                       return;
                   }
                   memcpy((char*)req.key_info.body.passphrase_info.passphrase,
                           optarg, req.key_info.body.passphrase_info.passphrase_len);
                   fprintf(stderr, "%s: passphrase = %s and passphrase_len = %d.\n",
                           __FUNCTION__, (char*)optarg,
                           req.key_info.body.passphrase_info.passphrase_len);
                   break;
                case 1002:
                   strlcpy((char*)req.service_name, optarg, NAN_MAX_SERVICE_NAME_LEN);
                   req.service_name_len = strlen((char*)req.service_name);
                   fprintf(stderr, "%s: service_name = %s and service_name_len = %d.\n",
                           __FUNCTION__, req.service_name, req.service_name_len);
                   break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NDP_RESPONSE, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_data_indication_response(id, ifaceHandle, &req);
    }

    void NanTestSuite::ndpEnd(int argc, char **argv)
    {
        /* A string listing valid short options letters.  */
        const char* const short_options = "hi:z:";
        /* An array describing valid long options.  */
        const struct option long_options[] = {
            { "help",           0,      NULL, 'h' },
            { "instance_ids",   1,      NULL, 'i' },
            { "transaction_id", 1,      NULL, 1005},
            { "exitflag",       1,      NULL, 'z' },
            { NULL,             0,      NULL,  0  }   /* Required at end of array.  */
        };
        transaction_id id = 0;
        NanDataPathEndRequest *ndpEndReq = NULL;

        /* Override with command line arguements */
        int long_index = 0, opt = 0;
        while ((opt = getopt_long(argc, argv, short_options,
                long_options, &long_index )) != -1)
        {
            switch (opt)
            {
                case 'i':
                    if (ndpEndReq) {
                        fprintf(stderr, "option '-i' has been processed already"
                                " and supported only once\n");
                        break;
                    }
                    ndpEndReq =
                        (NanDataPathEndRequest *)malloc(sizeof(NanDataPathEndRequest) + NDP_MAX_INSTANCE_IDS * sizeof(u32));
                    if (!ndpEndReq) {
                        fprintf(stderr, "%s: Memory allocation failed.\n", __FUNCTION__);
                        return;
                    }
                    memset(ndpEndReq, 0, sizeof(NanDataPathEndRequest) + (NDP_MAX_INSTANCE_IDS * sizeof(u32)));
                    ndpParseInstanceIds(optarg, ndpEndReq);
                    break;
                case 'z':
                    gExitflag = atoi(optarg);
                    break;
                case 1005:
                    id = atoi(optarg);
                    break;
                case 'h':
                default:
                    nanPrintCmdUsage(argv, NAN_CMD, NDP_END, long_options,
                       sizeof(long_options)/sizeof(struct option));
                    return;
            }
        }
        nan_data_end(id, ifaceHandle, ndpEndReq);
        if (!ndpEndReq)
            free(ndpEndReq);
    }

    void
    NanTestSuite::nanhexdump(
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

    void NanTestSuite::nanPrintFurtherAvailabilityChan(u8 num_chans,
        NanFurtherAvailabilityChannel* fachan)
    {
        fprintf(stderr, "*********Printing FurtherAvailabilityChan Info*******\n");
        fprintf(stderr, "Numchans:%d\n", num_chans);
        for(int idx = 0; idx < num_chans; idx++)
        {
            fprintf(stderr, "[%d]: NanAvailDuration:%d"
                            " class_val:%02x channel:%d\n",
                    idx, fachan->entry_control,
                    fachan->class_val, fachan->channel);
            fprintf(stderr, "[%d]: mapid:%d Availability bitmap:%08x\n",
                    idx, fachan->mapid,
                    fachan->avail_interval_bitmap);
        }
        fprintf(stderr, "*********************Done**********************\n");
    }

    int NanTestSuite::nanParseSocialChannelParams(
        const char* arg,
        NanSocialChannelScanParams* scan_params)
    {
        /*
          Social Channel param should have 3 consecutive dwell_time
          and 3 consecutive scan period seperated by "," as the
          delimiter
        */
        char* saveptr;
        char* token;
        int i = 0;
        char* input = (char*)arg;
        for(i = 0; i < NAN_MAX_SOCIAL_CHANNELS; i++,input=NULL)
        {
            token = strtok_r(input, ",", &saveptr);
            if (token == NULL)
            {
                fprintf(stderr,"Invalid Argument for SocialChannelParam\n");
                fprintf(stderr,"Expecting 6 integers seperated by \",\"\n");
                return -1;
            }
            scan_params->dwell_time[i] = (u8)atoi(token);
        }
        for(i = 0; i < NAN_MAX_SOCIAL_CHANNELS; i++,input=NULL)
        {
            token = strtok_r(NULL, ",", &saveptr);
            if (token == NULL)
            {
                fprintf(stderr,"Invalid Argument for SocialChannelParam\n");
                fprintf(stderr,"Expecting 6 integers seperated by \",\"\n");
                return -1;
            }
            scan_params->scan_period[i] = (u16)atoi(token);
        }
        fprintf(stderr,"********Parsed SocialChannelParam*********\n");
        fprintf(stderr,"dwell_time: %d %d %d scan_period: %d %d %d\n",
                scan_params->dwell_time[0], scan_params->dwell_time[1],
                scan_params->dwell_time[2], scan_params->scan_period[0],
                scan_params->scan_period[1], scan_params->scan_period[2]);
        fprintf(stderr,"********Parsed SocialChannelParam*********\n");
        return 0;
    }

    int NanTestSuite::nanParseTransmitPostConnectivityCapability(
        const char* arg,
        NanTransmitPostConnectivityCapability* conn_capability)
    {
        /*
          NanTransmitPostConnectivityCapability is expected as per
          how it is present in the Nan document
        */
        u32 conn_capability_value = 0;
        size_t size = sizeof(conn_capability_value);
        nanParseHexString(arg, (u8*)&conn_capability_value,
                          (int*)&size);
        conn_capability_value = ntohl(conn_capability_value);

        conn_capability->payload_transmit_flag = ((conn_capability_value & (0x01<<16))? 1:0);
        conn_capability->is_mesh_supported = ((conn_capability_value & (0x01<<5))? 1:0);
        conn_capability->is_ibss_supported = ((conn_capability_value & (0x01<<4))? 1:0);
        conn_capability->wlan_infra_field = ((conn_capability_value & (0x01<<3))? 1:0);
        conn_capability->is_tdls_supported = ((conn_capability_value & (0x01<<2))? 1:0);
        conn_capability->is_wfds_supported = ((conn_capability_value & (0x01<<1))? 1:0);
        conn_capability->is_wfd_supported = ((conn_capability_value & (0x01))? 1:0);
        fprintf(stderr,"********Parsed TxConnCapability*********\n");
        fprintf(stderr,"Input param:0x%08x\n",conn_capability_value);
        fprintf(stderr, "Wfd supported:%s\n",
                (conn_capability->is_wfd_supported? "yes": "no"));
        fprintf(stderr, "Wfds supported:%s\n",
                (conn_capability->is_wfds_supported? "yes": "no"));
        fprintf(stderr, "TDLS supported:%s\n",
                (conn_capability->is_tdls_supported? "yes": "no"));
        fprintf(stderr, "IBSS supported:%s\n",
                (conn_capability->is_ibss_supported? "yes": "no"));
        fprintf(stderr, "Mesh supported:%s\n",
                (conn_capability->is_mesh_supported? "yes": "no"));
        fprintf(stderr, "Infra Field:%d\n",
                conn_capability->wlan_infra_field);
        fprintf(stderr, "Payload Transmit Flag:%d\n",
                conn_capability->payload_transmit_flag);
        fprintf(stderr,"********Parsed TxConnCapability*********\n");
        return 0;
    }

    int NanTestSuite::nanParseTransmitPostDiscovery(
        const char* arg,
        NanTransmitPostDiscovery* discovery_attr)
    {
        /*
          1) Post Discovery should have 4 integers representing
            a) type
            b) role
            c) transmit_freq_flag
            d) duration
          2) FurtherAvailabilityMap hex string
          3) Mac address
          4) 32 octet of meshid which is
          required in case of conn type being WLAN_MESH or
          32 octet of ssid which is required in case of
          conn type being WLAN_INFRA
          All of these seperated by "," as the
          delimiter
        */
        char* saveptr;
        char* token;
        char* input = (char*)arg;

        token = strtok_r(input, ",", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
            fprintf(stderr,"Expecting 7 values seperated by \",\"\n");
            return -1;
        }
        discovery_attr->type = (NanConnectionType)atoi(token);
        token = strtok_r(NULL, ",", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
            fprintf(stderr,"Expecting 7 values seperated by \",\"\n");
            return -1;
        }
        discovery_attr->role =  (NanDeviceRole)atoi(token);
        token = strtok_r(NULL, ",", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
            fprintf(stderr,"Expecting 7 values seperated by \",\"\n");
            return -1;
        }
        discovery_attr->transmit_freq =  (u8)atoi(token);
        token = strtok_r(NULL, ",", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
            fprintf(stderr,"Expecting 7 values seperated by \",\"\n");
            return -1;
        }
        discovery_attr->duration =  (NanAvailDuration)atoi(token);
        token = strtok_r(NULL, ",", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
            fprintf(stderr,"Expecting 7 values seperated by \",\"\n");
            return -1;
        }
        int fam_bitmap = sizeof(discovery_attr->avail_interval_bitmap);
        nanParseHexString(token, (u8*)&discovery_attr->avail_interval_bitmap,
                          &fam_bitmap);
        discovery_attr->avail_interval_bitmap = \
            ntohl(discovery_attr->avail_interval_bitmap);
        token = strtok_r(NULL, ",", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
            fprintf(stderr,"Expecting 7 values seperated by \",\"\n");
            return -1;
        }
        nanParseMacAddress(token, discovery_attr->addr);
        if (discovery_attr->type == NAN_CONN_WLAN_MESH)
        {
            /* Parse the MESH ID */
            token = strtok_r(NULL, ",", &saveptr);
            if(token == NULL)
            {
                fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
                fprintf(stderr,"Expecting MESH ID\n");
                return -1;
            }
            int mesh_id_len =  NAN_MAX_MESH_DATA_LEN;
            nanParseHexString(token,
                              discovery_attr->mesh_id,
                              &mesh_id_len);
            discovery_attr->mesh_id_len = mesh_id_len;
        }
        if (discovery_attr->type == NAN_CONN_WLAN_INFRA)
        {
            /* Parse the INFRA SSID */
            token = strtok_r(NULL, ",", &saveptr);
            if(token == NULL)
            {
                fprintf(stderr,"Invalid Argument for TxPostDiscovery\n");
                fprintf(stderr,"Expecting Infrastructure Id\n");
                return -1;
            }
            int ssid_len =  NAN_MAX_MESH_DATA_LEN;
            nanParseHexString(token,
                              discovery_attr->infrastructure_ssid_val,
                              &ssid_len);
            discovery_attr->infrastructure_ssid_len = ssid_len;
        }
        /* print the parsed values here */
        fprintf(stderr,"********Parsed TxPostDiscovery*********\n");
        fprintf(stderr,"Connection Type:%d Role:%d transmit freq flag :%d\n",
                discovery_attr->type, discovery_attr->role,
                discovery_attr->transmit_freq);
        fprintf(stderr,"Duration:%d Availability_interval_bitmap:%04x\n",
                discovery_attr->duration, discovery_attr->avail_interval_bitmap);
        fprintf(stderr,"Mac address: " MAC_ADDR_STR "\n",
                MAC_ADDR_ARRAY(discovery_attr->addr));
        if (discovery_attr->type == NAN_CONN_WLAN_MESH)
        {
            fprintf(stderr,"MeshId:");
            nanhexdump(discovery_attr->mesh_id,
                       discovery_attr->mesh_id_len);
        }
        if (discovery_attr->type == NAN_CONN_WLAN_INFRA)
        {
            fprintf(stderr,"Infrastructure ssid:");
            nanhexdump(discovery_attr->infrastructure_ssid_val,
                       discovery_attr->infrastructure_ssid_len);
        }
        fprintf(stderr,"********Parsed TxPostDiscovery*********\n");
        return 0;
    }

    int NanTestSuite::nanParseFurtherAvailabilityMap(
        const char* arg,
        NanFurtherAvailabilityMap* fam)
    {
        /*
          Further availability map contains
          a) AvailDuration - integer
          b) class_val - byte value
          c) channel - byte value
          d) mapid - byte value
          e) Availability bitmap - 32bit value in hex
          All of these seperated by ":" as the delimiter
        */
        /* supporting only one numchans parsing */
        char* saveptr;
        char* token;
        char* input = (char*)arg;

        fam->numchans = 1;
        token = strtok_r(input, ":", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for FurtherAvailabilityMap\n");
            fprintf(stderr,"Expecting 5 values seperated by \":\"\n");
            return -1;
        }
        fam->famchan[0].entry_control = (NanAvailDuration)atoi(token);
        token = strtok_r(NULL, ":", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for FurtherAvailabilityMap\n");
            fprintf(stderr,"Expecting 5 values seperated by \":\"\n");
            return -1;
        }
        fam->famchan[0].class_val =  (u8)atoi(token);
        token = strtok_r(NULL, ":", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for FurtherAvailabilityMap\n");
            fprintf(stderr,"Expecting 5 values seperated by \":\"\n");
            return -1;
        }
        fam->famchan[0].channel =  (u8)atoi(token);
        token = strtok_r(NULL, ":", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for FurtherAvailabilityMap\n");
            fprintf(stderr,"Expecting 5 values seperated by \":\"\n");
            return -1;
        }
        fam->famchan[0].mapid =  (u8)atoi(token);
        token = strtok_r(NULL, ":", &saveptr);
        if(token == NULL)
        {
            fprintf(stderr,"Invalid Argument for FurtherAvailabilityMap\n");
            fprintf(stderr,"Expecting 5 values seperated by \":\"\n");
            return -1;
        }
        int fam_bitmap = sizeof(fam->famchan[0].avail_interval_bitmap);
        nanParseHexString(token, (u8*)&fam->famchan[0].avail_interval_bitmap,
                          &fam_bitmap);
        fam->famchan[0].avail_interval_bitmap = \
            ntohl(fam->famchan[0].avail_interval_bitmap);
        nanPrintFurtherAvailabilityChan(fam->numchans, &fam->famchan[0]);
        return 0;
    }

    int NanTestSuite::nanParseHexString(
        const char* input,
        u8* output,
        int* outputlen)
    {
        int i = 0;
        int j = 0;
        for(i = 0; ((i < (int)strlen(input)) && (j < *(outputlen))); i+= 2)
        {
            output[j] = nanParseHex(input[i]);
            if((i + 1) < (int)strlen(input))
            {
                output[j] = ((output[j] << 4) | nanParseHex(input[i + 1]));
            }
            j++;
        }
        *outputlen = j;
        fprintf(stderr, "Input:%s inputlen:%d outputlen:%d\n",
                input, (int)strlen(input), (int)*outputlen);
        return 0;
    }

    int NanTestSuite::nanParseMacAddresslist(
        const char* input,
        u8* output,
        u16 max_addr_allowed)
    {
        /*
         Reads a list of mac address seperated
         by ','. Each mac address should have
         the format of aa:bb:cc:dd:ee:ff
        */
        char* saveptr;
        char* token;
        int i = 0;
        for(i = 0; i < max_addr_allowed; i++)
        {
            token = strtok_r((i ==0)? (char*)input: NULL,
                             ",", &saveptr);
            if (token)
            {
                nanParseMacAddress(token, output);
                output += NAN_MAC_ADDR_LEN;
            }
            else
                break;
        }
        fprintf(stderr, "Num MacAddress:%d\n",i);
        return i;
    }

    void NanTestSuite::nanPrintStatsResponse(
        NanStatsResponse* statsrsp)
    {
        if(statsrsp->stats_type == NAN_STATS_ID_DE_PUBLISH)
        {
            fprintf(stderr, "******Printing Publish Stats******\n");
            NanPublishStats* pubstats = (NanPublishStats*)&statsrsp->data;
            fprintf(stderr, "validPublishServiceReqMsgs:%u\n"
                    "validPublishServiceRspMsgs:%u\n"
                    "validPublishServiceCancelReqMsgs:%u\n"
                    "validPublishServiceCancelRspMsgs:%u\n"
                    "validPublishRepliedIndMsgs:%u\n"
                    "validPublishTerminatedIndMsgs:%u\n"
                    "validActiveSubscribes:%u\n"
                    "validMatches:%u\n"
                    "validFollowups:%u\n"
                    "invalidPublishServiceReqMsgs:%u\n"
                    "invalidPublishServiceCancelReqMsgs:%u\n"
                    "invalidActiveSubscribes:%u\n"
                    "invalidMatches:%u\n"
                    "invalidFollowups:%u\n"
                    "publishCount:%u\n"
                    "publishNewMatchCount:%u\n"
                    "pubsubGlobalNewMatchCount:%u\n",
                    pubstats->validPublishServiceReqMsgs,
                    pubstats->validPublishServiceRspMsgs,
                    pubstats->validPublishServiceCancelReqMsgs,
                    pubstats->validPublishServiceCancelRspMsgs,
                    pubstats->validPublishRepliedIndMsgs,
                    pubstats->validPublishTerminatedIndMsgs,
                    pubstats->validActiveSubscribes,
                    pubstats->validMatches,
                    pubstats->validFollowups,
                    pubstats->invalidPublishServiceReqMsgs,
                    pubstats->invalidPublishServiceCancelReqMsgs,
                    pubstats->invalidActiveSubscribes,
                    pubstats->invalidMatches,
                    pubstats->invalidFollowups,
                    pubstats->publishCount,
                    pubstats->publishNewMatchCount,
                    pubstats->pubsubGlobalNewMatchCount);
            fprintf(stderr, "***************Done***************\n");
        }
        else if(statsrsp->stats_type == NAN_STATS_ID_DE_SUBSCRIBE)
        {
            fprintf(stderr, "*****Printing Subscribe Stats*****\n");
            NanSubscribeStats* substats = (NanSubscribeStats*)&statsrsp->data;
            fprintf(stderr, "validSubscribeServiceReqMsgs:%u\n"
                    "validSubscribeServiceRspMsgs:%u\n"
                    "validSubscribeServiceCancelReqMsgs:%u\n"
                    "validSubscribeServiceCancelRspMsgs:%u\n"
                    "validSubscribeTerminatedIndMsgs:%u\n"
                    "validSubscribeMatchIndMsgs:%u\n"
                    "validSubscribeUnmatchIndMsgs:%u\n"
                    "validSolicitedPublishes:%u\n"
                    "validMatches:%u\n"
                    "validFollowups:%u\n"
                    "invalidSubscribeServiceReqMsgs:%u\n"
                    "invalidSubscribeServiceCancelReqMsgs:%u\n"
                    "invalidSubscribeFollowupReqMsgs:%u\n"
                    "invalidSolicitedPublishes:%u\n"
                    "invalidMatches:%u\n"
                    "invalidFollowups:%u\n"
                    "subscribeCount:%u\n"
                    "bloomFilterIndex:%u\n"
                    "subscribeNewMatchCount:%u\n"
                    "pubsubGlobalNewMatchCount:%u\n",
                    substats->validSubscribeServiceReqMsgs,
                    substats->validSubscribeServiceRspMsgs,
                    substats->validSubscribeServiceCancelReqMsgs,
                    substats->validSubscribeServiceCancelRspMsgs,
                    substats->validSubscribeTerminatedIndMsgs,
                    substats->validSubscribeMatchIndMsgs,
                    substats->validSubscribeUnmatchIndMsgs,
                    substats->validSolicitedPublishes,
                    substats->validMatches,
                    substats->validFollowups,
                    substats->invalidSubscribeServiceReqMsgs,
                    substats->invalidSubscribeServiceCancelReqMsgs,
                    substats->invalidSubscribeFollowupReqMsgs,
                    substats->invalidSolicitedPublishes,
                    substats->invalidMatches,
                    substats->invalidFollowups,
                    substats->subscribeCount,
                    substats->bloomFilterIndex,
                    substats->subscribeNewMatchCount,
                    substats->pubsubGlobalNewMatchCount);
            fprintf(stderr, "***************Done***************\n");
        }
        else if(statsrsp->stats_type == NAN_STATS_ID_DE_MAC)
        {
            fprintf(stderr, "********Printing Mac Stats********\n");
            NanMacStats* macstats = (NanMacStats*)&statsrsp->data;
            fprintf(stderr, "****RX Stats****\n"
                    "validFrames:%u\n"
                    "validActionFrames:%u\n"
                    "validBeaconFrames:%u\n"
                    "ignoredActionFrames:%u\n"
                    "ignoredBeaconFrames:%u\n"
                    "invalidFrames:%u\n"
                    "invalidActionFrames:%u\n"
                    "invalidBeaconFrames:%u\n"
                    "invalidMacHeaders:%u\n"
                    "invalidPafHeaders:%u\n"
                    "nonNanBeaconFrames:%u\n"
                    "earlyActionFrames:%u\n"
                    "inDwActionFrames:%u\n"
                    "lateActionFrames:%u\n"
                    "****TX stats****\n"
                    "framesQueued:%u\n"
                    "totalTRSpUpdates:%u\n"
                    "completeByTRSp:%u\n"
                    "completeByTp75DW:%u\n"
                    "completeByTendDW:%u\n"
                    "lateActionFramesTx:%u\n"
                    "****Misc stats - ignored for DW****\n"
                    "twIncreases:%u\n"
                    "twDecreases:%u\n"
                    "twChanges:%u\n"
                    "twHighwater:%u\n"
                    "bloomFilterIndex:%u\n",
                    macstats->validFrames,
                    macstats->validActionFrames,
                    macstats->validBeaconFrames,
                    macstats->ignoredActionFrames,
                    macstats->ignoredBeaconFrames,
                    macstats->invalidFrames,
                    macstats->invalidActionFrames,
                    macstats->invalidBeaconFrames,
                    macstats->invalidMacHeaders,
                    macstats->invalidPafHeaders,
                    macstats->nonNanBeaconFrames,
                    macstats->earlyActionFrames,
                    macstats->inDwActionFrames,
                    macstats->lateActionFrames,
                    macstats->framesQueued,
                    macstats->totalTRSpUpdates,
                    macstats->completeByTRSp,
                    macstats->completeByTp75DW,
                    macstats->completeByTendDW,
                    macstats->lateActionFramesTx,
                    macstats->twIncreases,
                    macstats->twDecreases,
                    macstats->twChanges,
                    macstats->twHighwater,
                    macstats->bloomFilterIndex);
            fprintf(stderr, "***************Done***************\n");
        }
        else if(statsrsp->stats_type == NAN_STATS_ID_DE_TIMING_SYNC)
        {
            fprintf(stderr, "*******Printing Sync Stats*******\n");
            NanSyncStats* syncstats = (NanSyncStats*)&statsrsp->data;
            fprintf(stderr, "currTsf:%" PRIu64 "\n"
                    "myRank:%" PRIu64 "\n"
                    "currAmRank:%" PRIu64 "\n"
                    "lastAmRank:%" PRIu64 "\n"
                    "currAmBTT:%u\n"
                    "lastAmBTT:%u\n"
                    "currAmHopCount:%u\n"
                    "currRole:%u\n"
                    "currClusterId:%u\n"
                    "timeSpentInCurrRole:%" PRIu64 "\n"
                    "totalTimeSpentAsMaster:%" PRIu64 "\n"
                    "totalTimeSpentAsNonMasterSync:%" PRIu64 "\n"
                    "totalTimeSpentAsNonMasterNonSync:%" PRIu64 "\n"
                    "transitionsToAnchorMaster:%u\n"
                    "transitionsToMaster:%u\n"
                    "transitionsToNonMasterSync:%u\n"
                    "transitionsToNonMasterNonSync:%u\n"
                    "amrUpdateCount:%u\n"
                    "amrUpdateRankChangedCount:%u\n"
                    "amrUpdateBTTChangedCount:%u\n"
                    "amrUpdateHcChangedCount:%u\n"
                    "amrUpdateNewDeviceCount:%u\n"
                    "amrExpireCount:%u\n"
                    "mergeCount:%u\n"
                    "beaconsAboveHcLimit:%u\n"
                    "beaconsBelowRssiThresh:%u\n"
                    "beaconsIgnoredNoSpace:%u\n"
                    "beaconsForOurCluster:%u\n"
                    "beaconsForOtherCluster:%u\n"
                    "beaconCancelRequests:%u\n"
                    "beaconCancelFailures:%u\n"
                    "beaconUpdateRequests:%u\n"
                    "beaconUpdateFailures:%u\n"
                    "syncBeaconTxAttempts:%u\n"
                    "syncBeaconTxFailures:%u\n"
                    "discBeaconTxAttempts:%u\n"
                    "discBeaconTxFailures:%u\n"
                    "amHopCountExpireCount:%u\n"
                    "ndpChannelFreq:%d\n",
                    syncstats->currTsf,
                    syncstats->myRank,
                    syncstats->currAmRank,
                    syncstats->lastAmRank,
                    syncstats->currAmBTT,
                    syncstats->lastAmBTT,
                    syncstats->currAmHopCount,
                    syncstats->currRole,
                    syncstats->currClusterId,
                    syncstats->timeSpentInCurrRole,
                    syncstats->totalTimeSpentAsMaster,
                    syncstats->totalTimeSpentAsNonMasterSync,
                    syncstats->totalTimeSpentAsNonMasterNonSync,
                    syncstats->transitionsToAnchorMaster,
                    syncstats->transitionsToMaster,
                    syncstats->transitionsToNonMasterSync,
                    syncstats->transitionsToNonMasterNonSync,
                    syncstats->amrUpdateCount,
                    syncstats->amrUpdateRankChangedCount,
                    syncstats->amrUpdateBTTChangedCount,
                    syncstats->amrUpdateHcChangedCount,
                    syncstats->amrUpdateNewDeviceCount,
                    syncstats->amrExpireCount,
                    syncstats->mergeCount,
                    syncstats->beaconsAboveHcLimit,
                    syncstats->beaconsBelowRssiThresh,
                    syncstats->beaconsIgnoredNoSpace,
                    syncstats->beaconsForOurCluster,
                    syncstats->beaconsForOtherCluster,
                    syncstats->beaconCancelRequests,
                    syncstats->beaconCancelFailures,
                    syncstats->beaconUpdateRequests,
                    syncstats->beaconUpdateFailures,
                    syncstats->syncBeaconTxAttempts,
                    syncstats->syncBeaconTxFailures,
                    syncstats->discBeaconTxAttempts,
                    syncstats->discBeaconTxFailures,
                    syncstats->amHopCountExpireCount,
                    syncstats->ndpChannelFreq);
            fprintf(stderr, "***************Done***************\n");
        }
        else if(statsrsp->stats_type == NAN_STATS_ID_DE)
        {
            fprintf(stderr, "**Printing Discovery Engine Stats**\n");
            NanDeStats* destats = (NanDeStats*)&statsrsp->data;
            fprintf(stderr, "validErrorRspMsgs:%u\n"
                    "validTransmitFollowupReqMsgs:%u\n"
                    "validTransmitFollowupRspMsgs:%u\n"
                    "validFollowupIndMsgs:%u\n"
                    "validConfigurationReqMsgs:%u\n"
                    "validConfigurationRspMsgs:%u\n"
                    "validStatsReqMsgs:%u\n"
                    "validStatsRspMsgs:%u\n"
                    "validEnableReqMsgs:%u\n"
                    "validEnableRspMsgs:%u\n"
                    "validDisableReqMsgs:%u\n"
                    "validDisableRspMsgs:%u\n"
                    "validDisableIndMsgs:%u\n"
                    "validEventIndMsgs:%u\n"
                    "validTcaReqMsgs:%u\n"
                    "validTcaRspMsgs:%u\n"
                    "validTcaIndMsgs:%u\n"
                    "invalidTransmitFollowupReqMsgs:%u\n"
                    "invalidConfigurationReqMsgs:%u\n"
                    "invalidStatsReqMsgs:%u\n"
                    "invalidEnableReqMsgs:%u\n"
                    "invalidDisableReqMsgs:%u\n"
                    "invalidTcaReqMsgs:%u\n",
                    destats->validErrorRspMsgs,
                    destats->validTransmitFollowupReqMsgs,
                    destats->validTransmitFollowupRspMsgs,
                    destats->validFollowupIndMsgs,
                    destats->validConfigurationReqMsgs,
                    destats->validConfigurationRspMsgs,
                    destats->validStatsReqMsgs,
                    destats->validStatsRspMsgs,
                    destats->validEnableReqMsgs,
                    destats->validEnableRspMsgs,
                    destats->validDisableReqMsgs,
                    destats->validDisableRspMsgs,
                    destats->validDisableIndMsgs,
                    destats->validEventIndMsgs,
                    destats->validTcaReqMsgs,
                    destats->validTcaRspMsgs,
                    destats->validTcaIndMsgs,
                    destats->invalidTransmitFollowupReqMsgs,
                    destats->invalidConfigurationReqMsgs,
                    destats->invalidStatsReqMsgs,
                    destats->invalidEnableReqMsgs,
                    destats->invalidDisableReqMsgs,
                    destats->invalidTcaReqMsgs);
            fprintf(stderr, "***************Done***************\n");
        }
        else if(statsrsp->stats_type == NAN_STATS_ID_DE_DW)
        {
            fprintf(stderr, "********Printing DW Stats********\n");
            NanDWStats* dwstats = (NanDWStats*)&statsrsp->data;
            fprintf(stderr, "****RX Stats****\n"
                    "validFrames:%u\n"
                    "validActionFrames:%u\n"
                    "validBeaconFrames:%u\n"
                    "ignoredActionFrames:%u\n"
                    "ignoredBeaconFrames:%u\n"
                    "invalidFrames:%u\n"
                    "invalidActionFrames:%u\n"
                    "invalidBeaconFrames:%u\n"
                    "invalidMacHeaders:%u\n"
                    "invalidPafHeaders:%u\n"
                    "nonNanBeaconFrames:%u\n"
                    "earlyActionFrames:%u\n"
                    "inDwActionFrames:%u\n"
                    "lateActionFrames:%u\n"
                    "****TX stats****\n"
                    "framesQueued:%u\n"
                    "totalTRSpUpdates:%u\n"
                    "completeByTRSp:%u\n"
                    "completeByTp75DW:%u\n"
                    "completeByTendDW:%u\n"
                    "lateActionFramesTx:%u\n",
                    dwstats->validFrames,
                    dwstats->validActionFrames,
                    dwstats->validBeaconFrames,
                    dwstats->ignoredActionFrames,
                    dwstats->ignoredBeaconFrames,
                    dwstats->invalidFrames,
                    dwstats->invalidActionFrames,
                    dwstats->invalidBeaconFrames,
                    dwstats->invalidMacHeaders,
                    dwstats->invalidPafHeaders,
                    dwstats->nonNanBeaconFrames,
                    dwstats->earlyActionFrames,
                    dwstats->inDwActionFrames,
                    dwstats->lateActionFrames,
                    dwstats->framesQueued,
                    dwstats->totalTRSpUpdates,
                    dwstats->completeByTRSp,
                    dwstats->completeByTp75DW,
                    dwstats->completeByTendDW,
                    dwstats->lateActionFramesTx);
            fprintf(stderr, "****************Done***************\n");
        }
        else
        {
            fprintf(stderr,"Unknown stats_type:%d\n",
                    statsrsp->stats_type);
        }
    }
    void NanTestSuite::nanPrintCapabilities(NanCapabilities* rsp)
    {
        fprintf(stderr, "%s: ********NanCapabilities**********\n"
                "max_concurrent_nan_clusters:0x%x\n"
                "max_publishes:0x%x\n"
                "max_subscribes:0x%x\n"
                "max_service_name_len:0x%x\n"
                "max_match_filter_len:0x%x\n"
                "max_total_match_filter_len:0x%x\n"
                "max_service_specific_info_len:0x%x\n"
                "max_vsa_data_len:0x%x\n"
                "max_mesh_data_len:0x%x\n"
                "max_ndi_interfaces:0x%x\n"
                "max_ndp_sessions:0x%x\n"
                "max_app_info_len:0x%x\n"
                "max_queued_transmit_followup_msgs:0x%x\n"
                "ndp_supported_bands:0x%x\n"
                "cipher_suites_supported:0x%x\n"
                "max_scid_len:0x%x\n"
                "is_ndp_security_supported:0x%x\n"
                "max_sdea_service_specific_info_len:0x%x\n"
                "max_subscribe_address:0x%x\n"
                "ndpe_attr_supported:0x%x\n",
                __FUNCTION__,
                rsp->max_concurrent_nan_clusters,
                rsp->max_publishes,
                rsp->max_subscribes,
                rsp->max_service_name_len,
                rsp->max_match_filter_len,
                rsp->max_total_match_filter_len,
                rsp->max_service_specific_info_len,
                rsp->max_vsa_data_len,
                rsp->max_mesh_data_len,
                rsp->max_ndi_interfaces,
                rsp->max_ndp_sessions,
                rsp->max_app_info_len,
                rsp->max_queued_transmit_followup_msgs,
                rsp->ndp_supported_bands,
                rsp->cipher_suites_supported,
                rsp->max_scid_len,
                rsp->is_ndp_security_supported,
                rsp->max_sdea_service_specific_info_len,
                rsp->max_subscribe_address,
#if (NAN_MAJOR_VERSION > 2) || (NAN_MAJOR_VERSION == 2 && NAN_MINOR_VERSION >= 1)
                rsp->ndpe_attr_supported);
#else
                0);
#endif
    }

    int NanTestSuite::ndpParseInstanceIds(
        const char* arg,
        NanDataPathEndRequest* end_req)
    {
        /*
          Pass upto 20 ndp instance Ids seperated by ",".
        */
        char* saveptr;
        char* token;
        int i = 0;
        char* input = (char*)arg;
        for(i = 0; i < NDP_MAX_INSTANCE_IDS; i++,input=NULL)
        {
            token = strtok_r(input, ",", &saveptr);
            if (token == NULL)
            {
                break;
            }
            end_req->num_ndp_instances++;
            end_req->ndp_instance_id[i] = (u32)atoi(token);
        }
        fprintf(stderr,"********Parsed NDP Ids*********\n");
        for(i = 0; i < end_req->num_ndp_instances; i++)
            fprintf(stderr,"%d, ", end_req->ndp_instance_id[i]);
        fprintf(stderr,"\n");
        return 0;
    }
}

