/*
* Copyright (c) 2014 Qualcomm Technologies, Inc.  All Rights Reserved.
* Qualcomm Technologies Proprietary and Confidential.
*
*/

#ifndef __WIFI_HAL_NAN_TEST_HPP__
#define __WIFI_HAL_NAN_TEST_HPP__

#include "wifi_hal.h"
#include "nan_cert.h"
#include <getopt.h>
#include "common.hpp"

namespace NAN_TEST
{
    class NanTestSuite
    {
    public:

        /* CLI cmd strings */
        static const char *NAN_CMD;
        static const char *NAN_PUBLISH;
        static const char *NAN_SUBSCRIBE;
        static const char *NAN_ENABLE;
        static const char *NAN_DISABLE;
        static const char *NAN_PUBLISH_CANCEL;
        static const char *NAN_SUBSCRIBE_CANCEL;
        static const char *NAN_CONFIG;
        static const char *NAN_STATS;
        static const char *NAN_TCA;
        static const char *NAN_FOLLOWUP;
        static const char *NAN_BEACONSDF;
        static const char *NAN_GETSTAPARAMETER;
        static const char *NAN_GETCAPABILITIES;
        static const char *NDI_CREATE;
        static const char *NDI_DELETE;
        static const char *NDP_REQUEST;
        static const char *NDP_RESPONSE;
        static const char *NDP_END;
        static const char *NDP_SCHEDULE_UPDATE;
        static const char *NAN_DEBUG;
        static const char *NAN_AWARE_IFACE;

        /* Default service name */
        static const char *DEFAULT_SVC;

        /* Default service name */
        static const char *DEFAULT_SVC_INFO;

        //NanTestSuite(wifi_request_id id, wifi_handle handle);
        NanTestSuite(wifi_handle handle);

        /* process the command line args */
        void processCmd(int argc, char **argv);

        /* Various Events/response Callbacks */
        static void nanNotifyResponse(transaction_id id, NanResponseMsg *rsp_data);
#if QTI_BSP
        static void nanEventPublishReplied(NanPublishRepliedInd *event);
#endif
        static void nanEventPublishTerminated(NanPublishTerminatedInd *event);
        static void nanEventMatch(NanMatchInd *event);
        static void nanEventMatchExpired(NanMatchExpiredInd *event);
        static void nanEventSubscribeTerminated(NanSubscribeTerminatedInd *event);
        static void nanEventFollowup(NanFollowupInd *event);
        static void nanEventDiscEngEvent(NanDiscEngEventInd *event);
        static void nanEventDisabled(NanDisabledInd *event);
        static void nanEventTca(NanTCAInd *event);
        static void nanEventBeaconSdfPayload(NanBeaconSdfPayloadInd* event);
        static void ndpEventRequest(NanDataPathRequestInd* event);
        static void ndpEventConfirm(NanDataPathConfirmInd* event);
        static void ndpEventEnd(NanDataPathEndInd* event);
        static void nanEventSelfTransmitFollowup(NanTransmitFollowupInd *event);
        static void nanRangeRequestReceivedInd(NanRangeRequestInd *event);
        static void nanRangeReportInd(NanRangeReportInd *event);
        static void ndpEventScheduleUpdate(NanDataPathScheduleUpdateInd *event);
    private:
        wifi_handle wifiHandle_;
        //wifi_request_id id;
        wifi_interface_handle ifaceHandle;

        /* Send a NAN command to Android HAL */
        void nanSendEnableRequest(int argc, char **argv);
        void nanSendDisableRequest(int argc, char **argv);
        void nanSendPublishRequest(int argc, char **argv);
        void nanSendPublishCancelRequest(int argc, char **argv);
        void nanSendSubscribeRequest(int argc, char **argv);
        void nanSendSubscribeCancelRequest(int argc, char **argv);
        void nanSendConfigRequest(int argc, char **argv);
        void nanSendTransmitFollowupRequest(int argc, char **argv);
        void nanSendStatsRequest(int argc, char **argv);
        void nanSendTCARequest(int argc, char **argv);
        void nanSendBeaconSdfRequest(int argc, char **argv);
        void nanGetStaParameter(int argc, char **argv);
        void nanGetCapabilities(int argc, char **argv);
        void nanSendDebugRequest(int argc, char **argv);
        void ndpInterfaceCreate(int argc, char **argv);
        void ndpInterfaceDelete(int argc, char **argv);
        void ndpInitiatorRequest(int argc, char **argv);
        void ndpIndicationResponse(int argc, char **argv);
        void ndpEnd(int argc, char **argv);
        void ndpScheduleUpdate(int argc, char **argv);
        int nanParseHex(unsigned char c);
        int nanParseMacAddress(const char* arg, u8* addr);
        void nanPrintCmdUsage(
            char **argv,
            const char *cmd,
            const char *sub_cmd,
            const struct option long_options[],
            int size);
        static void nanhexdump(
        uint8_t* data,
        size_t len);
        static void nanPrintFurtherAvailabilityChan(u8 num_chans,
            NanFurtherAvailabilityChannel* fachan);
        int nanParseSocialChannelParams(
            const char* arg,
            NanSocialChannelScanParams* scan_params);
        int nanParseTransmitPostConnectivityCapability(
            const char* arg,
            NanTransmitPostConnectivityCapability* conn_capability);
        int nanParseTransmitPostDiscovery(
            const char* arg,
            NanTransmitPostDiscovery* discovery_attr);
        int nanParseFurtherAvailabilityMap(
            const char* arg,
            NanFurtherAvailabilityMap* fam);
        int nanParseHexString(
            const char* input,
            u8* output,
            int* outputlen);
        int nanParseMacAddresslist(
            const char* input,
            u8* output,
            u16 max_addr_allowed);
        static void nanPrintStatsResponse(
            NanStatsResponse* statsrsp);
        static void nanPrintCapabilities(NanCapabilities* rsp);
        int ndpParseInstanceIds(const char* arg, NanDataPathEndRequest* end_req);
        void nanParseIPV6Address(const char* arg, u8* ipv6_addr);
        char cur_nan_iface[IFNAMSIZ+1];
    };
}

#endif
