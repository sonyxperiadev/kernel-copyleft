/*
 * Copyright (c) 2016 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#include <stdio.h>
#include <inttypes.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x\n"

/* WiFi Common definitions */
/* channel operating width */
typedef enum {
    WIFI_CHAN_WIDTH_20    = 0,
    WIFI_CHAN_WIDTH_40    = 1,
    WIFI_CHAN_WIDTH_80    = 2,
    WIFI_CHAN_WIDTH_160   = 3,
    WIFI_CHAN_WIDTH_80P80 = 4,
    WIFI_CHAN_WIDTH_5     = 5,
    WIFI_CHAN_WIDTH_10    = 6,
    WIFI_CHAN_WIDTH_INVALID = -1
} wifi_channel_width;

typedef int wifi_radio;
typedef int wifi_channel;

typedef struct {
    wifi_channel_width width;
    int center_frequency0;
    int center_frequency1;
    int primary_frequency;
} wifi_channel_spec;

typedef struct {
    int status;   // 0 taken, 1 released
    int reason;   // reason why this wake lock is taken
    char name[0]; // null terminated
} __attribute__((packed)) wake_lock_event;

typedef struct bt_coex_common_data {
    u8 link_id;
    u8 link_state;
    u8 link_role;
} __attribute__((packed)) bt_coex_common_data_t;

typedef struct bt_coex_vendor_data {
    u8 link_type;
    u16 Tsco;
    u8 Rsco;
} __attribute__((packed)) bt_coex_vendor_data_t;

typedef struct bt_coex_hid_vendor_data {
    u8 Tsniff;
    u8 attempts;
} bt_coex_hid_vendor_data_t;

typedef struct ext_scan_cycle_vendor_data {
    u32 timer_tick;
    u32 scheduled_bucket_mask;
    u32 scan_cycle_count;
} __attribute__((packed)) ext_scan_cycle_vendor_data_t;

typedef struct ext_scan_results_available_vendor_data {
    u32 table_type;
    u32 entries_in_use;
    u32 maximum_entries;
    u32 scan_count_after_getResults;
    u8 threshold_num_scans;
} __attribute__((packed)) ext_scan_results_available_vendor_data;

typedef struct {
    u32 roam_scan_flags;
    u32 cur_rssi;
    u16 scan_params[18];
    u16 scan_channels[40]; // first 40 channels only
} __attribute__((packed)) roam_scan_started_vendor_data_t;

typedef struct {
    u32 completion_flags;
    u32 num_candidate;
    u32 flags;
} __attribute__((packed)) roam_scan_complete_vendor_data_t;

typedef struct {
    u8 ssid[33];
    u8 auth_mode;
    u8 ucast_cipher;
    u8 mcast_cipher;
} __attribute__((packed)) roam_candidate_found_vendor_data_t;

typedef struct {
    u8 scan_type;
    u8 scan_bitmap;
} __attribute__((packed)) bt_coex_bt_scan_start_vendor_data_t;

typedef struct {
    u8 scan_type;
    u8 scan_bitmap;
} __attribute__((packed)) bt_coex_bt_scan_stop_vendor_data_t;

typedef struct {
    u16 sme_state;
    u16 mlm_state;
} __attribute__((packed)) pe_event_vendor_data_t;

typedef struct {
    u8 Tsniff;
    u8 attempts;
} __attribute__((packed)) bt_coex_bt_hid_vendor_data_t;

typedef struct {
    u8 ucBaPeerMac[6];
    u8 ucBaTid;
    u8 ucBaBufferSize;
    u16 ucBaSSN;
    u8 fInitiator;
} __attribute__((packed)) wlan_add_block_ack_success_payload_type;

/* EVENT_WLAN_ADD_BLOCK_ACK_FAILED */
typedef struct {
    u8 ucBaPeerMac[6];
    u8 ucBaTid;
    u8 ucReasonCode;
    u8 fInitiator;
} __attribute__((packed)) wlan_add_block_ack_failed_payload_type;

typedef struct {
    u32 flags;
    u32 roam_scan_config[8];
} __attribute__((packed)) wlan_roam_scan_config_payload_type;

typedef struct {
    u8 bssid[6];
    u32 rssi;
} __attribute__((packed)) beacon_event_data_t;

typedef enum
{
    WIFI_EVENT_MEMORY_FAILURE,
} resource_failure_type;

typedef struct wlan_low_resource_failure_event
{
    resource_failure_type event_sub_type;
} __attribute__((packed)) wlan_low_resource_failure_event_t;

// Below events refer to the wifi_connectivity_event ring and shall be supported

char *events[] = {
    "WIFI_EVENT_ASSOCIATION_REQUESTED", // driver receive association command from kernel
    "WIFI_EVENT_AUTH_COMPLETE",
    "WIFI_EVENT_ASSOC_COMPLETE",
    "WIFI_EVENT_FW_AUTH_STARTED", // received firmware event indicating auth frames are sent
    "WIFI_EVENT_FW_ASSOC_STARTED", // received firmware event indicating assoc frames are sent
    "WIFI_EVENT_FW_RE_ASSOC_STARTED", // received firmware event indicating reassoc frames are sent
    "WIFI_EVENT_DRIVER_SCAN_REQUESTED",
    "WIFI_EVENT_DRIVER_SCAN_RESULT_FOUND",
    "WIFI_EVENT_DRIVER_SCAN_COMPLETE",
    "WIFI_EVENT_G_SCAN_STARTED",
    "WIFI_EVENT_G_SCAN_COMPLETE",
    "WIFI_EVENT_DISASSOCIATION_REQUESTED",
    "WIFI_EVENT_RE_ASSOCIATION_REQUESTED",
    "WIFI_EVENT_ROAM_REQUESTED",
    "WIFI_EVENT_BEACON_RECEIVED",  // received beacon from AP (event enabled only in verbose mode)
    "WIFI_EVENT_ROAM_SCAN_STARTED",  // firmware has triggered a roam scan (not g-scan)
    "WIFI_EVENT_ROAM_SCAN_COMPLETE",  // firmware has completed a roam scan (not g-scan)
    "WIFI_EVENT_ROAM_SEARCH_STARTED",  // firmware has started searching for roam candidates (with reason =xx)
    "WIFI_EVENT_ROAM_SEARCH_STOPPED",  // firmware has stopped searching for roam candidates (with reason =xx)
    "NULL",
    "WIFI_EVENT_CHANNEL_SWITCH_ANOUNCEMENT",  // received channel switch anouncement from AP
    "WIFI_EVENT_FW_EAPOL_FRAME_TRANSMIT_START",  // fw start transmit eapol frame, with EAPOL index 1-4
    "WIFI_EVENT_FW_EAPOL_FRAME_TRANSMIT_STOP",  // fw gives up eapol frame, with rate, success/failure and number retries
    "WIFI_EVENT_DRIVER_EAPOL_FRAME_TRANSMIT_REQUESTED",  // kernel queue EAPOL for transmission in tdriver
    // with EAPOL index 1-4
    "WIFI_EVENT_FW_EAPOL_FRAME_RECEIVED",  // with rate, regardless of the fact that EAPOL frame
    // is accepted or rejected by firmware
    "NULL",
    "WIFI_EVENT_DRIVER_EAPOL_FRAME_RECEIVED",  // with rate, and eapol index, driver has received
    // EAPOL frame and will queue it up to wpa_supplicant
    "WIFI_EVENT_BLOCK_ACK_NEGOTIATION_COMPLETE",  // with success/failure, parameters
    "WIFI_EVENT_BT_COEX_BT_SCO_START",
    "WIFI_EVENT_BT_COEX_BT_SCO_STOP",
    "WIFI_EVENT_BT_COEX_BT_SCAN_START",  // for paging/scan etc..., when BT starts transmiting
    //    twice per BT slot
    "WIFI_EVENT_BT_COEX_BT_SCAN_STOP",
    "WIFI_EVENT_BT_COEX_BT_HID_START",
    "WIFI_EVENT_BT_COEX_BT_HID_STOP",
    "WIFI_EVENT_ROAM_AUTH_STARTED",   // firmware sends auth frame in roaming to next candidate
    "WIFI_EVENT_ROAM_AUTH_COMPLETE",  // firmware receive auth confirm from ap
    "WIFI_EVENT_ROAM_ASSOC_STARTED",  // firmware sends assoc/reassoc frame in
    //     roaming to next candidate
    "WIFI_EVENT_ROAM_ASSOC_COMPLETE",  // firmware receive assoc/reassoc confirm from ap
    "WIFI_EVENT_G_SCAN_STOP",  // firmware sends stop G_SCAN
    "WIFI_EVENT_G_SCAN_CYCLE_STARTED",  // firmware indicates G_SCAN scan cycle started
    "WIFI_EVENT_G_SCAN_CYCLE_COMPLETED",  // firmware indicates G_SCAN scan cycle completed
    "WIFI_EVENT_G_SCAN_BUCKET_STARTED",  // firmware indicates G_SCAN scan start
    // for a particular bucket
    "WIFI_EVENT_G_SCAN_BUCKET_COMPLETED",  // firmware indicates G_SCAN scan completed for
    // for a particular bucket
    "WIFI_EVENT_G_SCAN_RESULTS_AVAILABLE",  // Event received from firmware about G_SCAN scan
    // results being available
    "WIFI_EVENT_G_SCAN_CAPABILITIES",  // Event received from firmware with G_SCAN
    // capabilities
    "WIFI_EVENT_ROAM_CANDIDATE_FOUND",  // Event received from firmware when eligible
        // candidate is found
    "WIFI_EVENT_ROAM_SCAN_CONFIG",
    "WIFI_EVENT_ASSOC_TIMEOUT",
    "WIFI_EVENT_AUTH_TIMEOUT",
    "WIFI_EVENT_RESOURCE_FAILURE",
};

// Below events refer to the wifi_connectivity_event ring and shall be supported

#define WIFI_EVENT_ASSOCIATION_REQUESTED 0 // driver receive association command from kernel
#define WIFI_EVENT_AUTH_COMPLETE 1
#define WIFI_EVENT_ASSOC_COMPLETE 2
#define WIFI_EVENT_FW_AUTH_STARTED 3 // received firmware event indicating auth frames are sent
#define WIFI_EVENT_FW_ASSOC_STARTED 4 // received firmware event indicating assoc frames are sent
#define WIFI_EVENT_FW_RE_ASSOC_STARTED 5 // received firmware event indicating reassoc frames are sent
#define WIFI_EVENT_DRIVER_SCAN_REQUESTED 6
#define WIFI_EVENT_DRIVER_SCAN_RESULT_FOUND 7
#define WIFI_EVENT_DRIVER_SCAN_COMPLETE 8
#define WIFI_EVENT_G_SCAN_STARTED 9
#define WIFI_EVENT_G_SCAN_COMPLETE 10
#define WIFI_EVENT_DISASSOCIATION_REQUESTED 11
#define WIFI_EVENT_RE_ASSOCIATION_REQUESTED 12
#define WIFI_EVENT_ROAM_REQUESTED 13
#define WIFI_EVENT_BEACON_RECEIVED 14 // received beacon from AP (event enabled only in verbose mode)
#define WIFI_EVENT_ROAM_SCAN_STARTED 15 // firmware has triggered a roam scan (not g-scan)
#define WIFI_EVENT_ROAM_SCAN_COMPLETE 16 // firmware has completed a roam scan (not g-scan)
#define WIFI_EVENT_ROAM_SEARCH_STARTED 17 // firmware has started searching for roam candidates (with reason =xx)
#define WIFI_EVENT_ROAM_SEARCH_STOPPED 18 // firmware has stopped searching for roam candidates (with reason =xx)
#define WIFI_EVENT_CHANNEL_SWITCH_ANOUNCEMENT 20 // received channel switch anouncement from AP
#define WIFI_EVENT_FW_EAPOL_FRAME_TRANSMIT_START 21 // fw start transmit eapol frame, with EAPOL index 1-4
#define WIFI_EVENT_FW_EAPOL_FRAME_TRANSMIT_STOP 22 // fw gives up eapol frame, with rate, success/failure and number retries
#define WIFI_EVENT_DRIVER_EAPOL_FRAME_TRANSMIT_REQUESTED 23 // kernel queue EAPOL for transmission in tdriver
                                                            // with EAPOL index 1-4
#define WIFI_EVENT_FW_EAPOL_FRAME_RECEIVED 24 // with rate, regardless of the fact that EAPOL frame
                                           // is accepted or rejected by firmware
#define WIFI_EVENT_DRIVER_EAPOL_FRAME_RECEIVED 26 // with rate, and eapol index, driver has received
                                                  // EAPOL frame and will queue it up to wpa_supplicant
#define WIFI_EVENT_BLOCK_ACK_NEGOTIATION_COMPLETE 27 // with success/failure, parameters
#define WIFI_EVENT_BT_COEX_BT_SCO_START 28
#define WIFI_EVENT_BT_COEX_BT_SCO_STOP 29
#define WIFI_EVENT_BT_COEX_BT_SCAN_START 30 // for paging/scan etc..., when BT starts transmiting
                                            //    twice per BT slot
#define WIFI_EVENT_BT_COEX_BT_SCAN_STOP 31
#define WIFI_EVENT_BT_COEX_BT_HID_START 32
#define WIFI_EVENT_BT_COEX_BT_HID_STOP 33
#define WIFI_EVENT_ROAM_AUTH_STARTED 34 // firmware sends auth frame in roaming to next candidate
#define WIFI_EVENT_ROAM_AUTH_COMPLETE 35 // firmware receive auth confirm from ap
#define WIFI_EVENT_ROAM_ASSOC_STARTED 36 // firmware sends assoc/reassoc frame in
                                         //     roaming to next candidate
#define WIFI_EVENT_ROAM_ASSOC_COMPLETE 37 // firmware receive assoc/reassoc confirm from ap
#define WIFI_EVENT_G_SCAN_STOP 38 // firmware sends stop G_SCAN
#define WIFI_EVENT_G_SCAN_CYCLE_STARTED 39 // firmware indicates G_SCAN scan cycle started
#define WIFI_EVENT_G_SCAN_CYCLE_COMPLETED 40 // firmware indicates G_SCAN scan cycle completed
#define WIFI_EVENT_G_SCAN_BUCKET_STARTED 41 // firmware indicates G_SCAN scan start
                                            // for a particular bucket
#define WIFI_EVENT_G_SCAN_BUCKET_COMPLETED 42 // firmware indicates G_SCAN scan completed for
                                             // for a particular bucket
#define WIFI_EVENT_G_SCAN_RESULTS_AVAILABLE 43 // Event received from firmware about G_SCAN scan
                                               // results being available
#define WIFI_EVENT_G_SCAN_CAPABILITIES 44 // Event received from firmware with G_SCAN
                                           // capabilities
#define WIFI_EVENT_ROAM_CANDIDATE_FOUND 45 // Event received from firmware when eligible
                                           // candidate is found
#define WIFI_EVENT_ROAM_SCAN_CONFIG 46
#define WIFI_EVENT_ASSOC_TIMEOUT 47
#define WIFI_EVENT_AUTH_TIMEOUT 48
#define WIFI_EVENT_RESOURCE_FAILURE 49

// Parameters of wifi logger events are TLVs
// Event parameters tags are defined as:
#define WIFI_TAG_VENDOR_SPECIFIC 0 // take a byte stream as parameter
#define WIFI_TAG_BSSID 1 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_ADDR 2 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_SSID 3 // takes a 32 bytes SSID address as parameter
#define WIFI_TAG_STATUS 4 // takes an integer as parameter
#define WIFI_TAG_CHANNEL_SPEC 5 // takes one or more wifi_channel_spec as parameter
#define WIFI_TAG_WAKE_LOCK_EVENT 6 // takes a wake_lock_event struct as parameter
#define WIFI_TAG_ADDR1 7 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_ADDR2 8 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_ADDR3 9 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_ADDR4 10 // takes a 6 bytes MAC address as parameter
#define WIFI_TAG_TSF 11 // take a 64 bits TSF value as parameter
#define WIFI_TAG_IE 12  // take one or more specific 802.11 IEs parameter, IEs are in turn indicated
                        // in TLV format as per 802.11 spec
#define WIFI_TAG_INTERFACE 13 // take interface name as parameter
#define WIFI_TAG_REASON_CODE 14 // take a reason code as per 802.11 as parameter
#define WIFI_TAG_RATE_MBPS 15 // take a wifi rate in 0.5 mbps

#define WIFI_TAG_REQUEST_ID 16 // take an integer as parameter
#define WIFI_TAG_BUCKET_ID 17 // take an integer as parameter
#define WIFI_TAG_GSCAN_PARAMS 18 // takes a wifi_scan_cmd_params struct as parameter
#define WIFI_TAG_GSCAN_CAPABILITIES 19 // takes a wifi_gscan_capabilities struct as parameter
#define WIFI_TAG_SCAN_ID 20 // take an integer as parameter
#define WIFI_TAG_RSSI 21 // take an integer as parameter
#define WIFI_TAG_CHANNEL 22 // take an integer as parameter
#define WIFI_TAG_LINK_ID 23 // take an integer as parameter
#define WIFI_TAG_LINK_ROLE 24 // take an integer as parameter
#define WIFI_TAG_LINK_STATE 25 // take an integer as parameter
#define WIFI_TAG_LINK_TYPE 26 // take an integer as parameter
#define WIFI_TAG_TSCO 27 // take an integer as parameter
#define WIFI_TAG_RSCO 28 // take an integer as parameter
#define WIFI_TAG_EAPOL_MESSAGE_TYPE 29 // take an integer as parameter
                                       // M1-1, M2-2, M3-3, M4-4
typedef struct {
    u16 tag;
    u16 length; // length of value
    u8 value[0];
} tlv_log;

typedef struct {
    u16 event;
    tlv_log tlvs[0];    // separate parameter structure per event to be provided and optional data
                        // the event_data is expected to include an official android part, with some
                        // parameter as transmit rate, num retries, num scan result found etc...
                        // as well, event_data can include a vendor proprietary part which is
                        // understood by the developer only.
} __attribute__((packed)) wifi_ring_buffer_driver_connectivity_event;

typedef struct {
    uint16_t entry_size; // the size of payload excluding the header.
    uint8_t flags;
    uint8_t type; // Entry type
    uint64_t timestamp; //present if has_timestamp bit is set.
} __attribute__((packed)) wifi_ring_buffer_entry;

typedef struct {
    wifi_ring_buffer_entry rb_head;
    wifi_ring_buffer_driver_connectivity_event *conn_entry;
}rb_entry;

void parse_connectivity_event(FILE *ofile, wifi_ring_buffer_driver_connectivity_event *conn_entry, u16 buflen)
{
    u16 len = 0, i, j = 0;
    tlv_log *tlvs = conn_entry->tlvs;
    wifi_channel_spec *chan;
    wake_lock_event *wle;
    int addba_status;

    fprintf(ofile, "Event: %s: %d\n", events[conn_entry->event], conn_entry->event);
    /*sizeof(conn_entry->event) ==2 */
    while(len < buflen - 2)
    {
        fprintf(ofile, "\nTLV-%d: length: %d ", j++, tlvs->length);
        switch(tlvs->tag)
        {
            case WIFI_TAG_VENDOR_SPECIFIC:
            {
                fprintf(ofile, "WIFI_TAG_VENDOR_SPECIFIC\n");
#if 0
                for(i=0; i<tlvs->length; i++)
                    fprintf(ofile, "%02x ", *(u8 *)&tlvs->value[i]);
#endif
                switch(conn_entry->event)
                {
                    case WIFI_EVENT_BT_COEX_BT_SCAN_START:
                    {
                        bt_coex_bt_scan_start_vendor_data_t *bt_coex_strt_vd
                             = (bt_coex_bt_scan_start_vendor_data_t *)tlvs->value;
                        fprintf(ofile, "       scan_type             : %u\n", bt_coex_strt_vd->scan_type);
                        fprintf(ofile, "       scan_bitmap           : 0x%x\n", bt_coex_strt_vd->scan_bitmap);
                    }
                    break;
                    case WIFI_EVENT_BT_COEX_BT_SCAN_STOP:
                    {
                        bt_coex_bt_scan_stop_vendor_data_t *bt_coex_stop_vd
                             = (bt_coex_bt_scan_stop_vendor_data_t *)tlvs->value;
                        fprintf(ofile, "       scan_type             : %u\n", bt_coex_stop_vd->scan_type);
                        fprintf(ofile, "       scan_bitmap           : 0x%x\n", bt_coex_stop_vd->scan_bitmap);
;
                    }
                    break;
                    case WIFI_EVENT_G_SCAN_CYCLE_STARTED:
                    case WIFI_EVENT_G_SCAN_CYCLE_COMPLETED:
                    {
                        ext_scan_cycle_vendor_data_t *ext_cycle_vd = (ext_scan_cycle_vendor_data_t *)tlvs->value;
                        fprintf(ofile, "       timer_tick            : %u\n", ext_cycle_vd->timer_tick);
                        fprintf(ofile, "       scheduled_bucket_mask : %u\n", ext_cycle_vd->scheduled_bucket_mask);
                        fprintf(ofile, "       scan_cycle_count      : %u\n", ext_cycle_vd->scan_cycle_count);
                    }
                    break;
                    case WIFI_EVENT_G_SCAN_RESULTS_AVAILABLE:
                    {
                        ext_scan_results_available_vendor_data *ext_scan_res_vd
                         = (ext_scan_results_available_vendor_data *)tlvs->value;
                        fprintf(ofile, "       table_type                   : %u\n", ext_scan_res_vd->table_type);
                        fprintf(ofile, "       entries_in_use               : %u\n", ext_scan_res_vd->entries_in_use);
                        fprintf(ofile, "       maximum_entries              : %u\n", ext_scan_res_vd->maximum_entries);
                        fprintf(ofile, "       scan_count_after_getResults  : %u\n", ext_scan_res_vd->scan_count_after_getResults);
                        fprintf(ofile, "       threshold_num_scans          : %u\n", ext_scan_res_vd->threshold_num_scans);
                    }
                    break;
                    case WIFI_EVENT_BLOCK_ACK_NEGOTIATION_COMPLETE:
                    {
                        //TODO: Need to fix/complete the impl in wifihal; then add decoding
                        fprintf(ofile, "WIFI_EVENT_BLOCK_ACK_NEGOTIATION_COMPLETE\n");

                        if (addba_status == 0) {
                            fprintf(ofile, "Addba success\n");
                            wlan_add_block_ack_success_payload_type *ba_success_vd
                                = (wlan_add_block_ack_success_payload_type *)tlvs->value;
                            fprintf(ofile, "       ucBaPeerMac     : ");
                            fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)ba_success_vd->ucBaPeerMac));
                            fprintf(ofile, "       ucBaTid         : %u\n", ba_success_vd->ucBaTid);
                            fprintf(ofile, "       ucBaBufferSize  : %u\n", ba_success_vd->ucBaBufferSize);
                            fprintf(ofile, "       ucBaSSN         : %u\n", ba_success_vd->ucBaSSN);
                            fprintf(ofile, "       fInitiator      : %u\n", ba_success_vd->fInitiator);
                        } else {
                            wlan_add_block_ack_failed_payload_type *ba_failure_vd
                                = (wlan_add_block_ack_failed_payload_type *)tlvs->value;
                            fprintf(ofile, "Addba failure\n");
                            fprintf(ofile, "       ucBaPeerMac     : ");
                            fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)ba_failure_vd->ucBaPeerMac));
                            fprintf(ofile, "       ucBaTid         : %u\n", ba_failure_vd->ucBaTid);
                            fprintf(ofile, "       ucReasonCode    : %u\n", ba_failure_vd->ucReasonCode);
                            fprintf(ofile, "       fInitiator      : %u\n", ba_failure_vd->fInitiator);
                        }
                    }
                    break;
                    case WIFI_EVENT_ROAM_SCAN_STARTED:
                    {
                        roam_scan_started_vendor_data_t *roam_scan_strt_vd
                         = (roam_scan_started_vendor_data_t *)tlvs->value;
                        fprintf(ofile, "       roam_scan_flags  : 0x%x\n", roam_scan_strt_vd->roam_scan_flags);
                        fprintf(ofile, "       cur_rssi         : %d\n", roam_scan_strt_vd->cur_rssi);
                        fprintf(ofile, "       scan params : ");
                        for(i=0; i<18; i++)
                            fprintf(ofile, "0x%x ", roam_scan_strt_vd->scan_params[i]);
                        fprintf(ofile, "\n       scan channels: ");
                        for(i=0; i<40; i++)
                            fprintf(ofile, "0x%x ", roam_scan_strt_vd->scan_channels[i]);
                    }
                    break;
                    case WIFI_EVENT_ROAM_SCAN_COMPLETE:
                    {
                        roam_scan_complete_vendor_data_t *roam_scan_cmpl_vd
                         = (roam_scan_complete_vendor_data_t *)tlvs->value;
                        fprintf(ofile, "       completion_flags : 0x%x\n", roam_scan_cmpl_vd->completion_flags);
                        fprintf(ofile, "       num_candidate    : %u\n",   roam_scan_cmpl_vd->num_candidate);
                        fprintf(ofile, "       flags            : 0x%x\n", roam_scan_cmpl_vd->flags);
                    }
                    break;
                    case WIFI_EVENT_ROAM_CANDIDATE_FOUND:
                    {
                        roam_candidate_found_vendor_data_t *roam_cd_found_vd
                         = (roam_candidate_found_vendor_data_t *)tlvs->value;
                        fprintf(ofile, "       ssid         : %s\n", roam_cd_found_vd->ssid);
                        fprintf(ofile, "       auth_mode    : %u\n", roam_cd_found_vd->auth_mode);
                        fprintf(ofile, "       ucast_cipher : %u\n", roam_cd_found_vd->ucast_cipher);
                        fprintf(ofile, "       mcast_cipher : %u\n", roam_cd_found_vd->mcast_cipher);
                    }
                    break;
                    case WIFI_EVENT_ROAM_SCAN_CONFIG:
                    {
                        wlan_roam_scan_config_payload_type *roam_payload_vd
                         = (wlan_roam_scan_config_payload_type *)&tlvs->value[0];
                        fprintf(ofile, "       flags: 0x%x\n", roam_payload_vd->flags);
                        fprintf(ofile, "       scan config: ");
                        for(i=0; i<8; i++)
                            fprintf(ofile, "0x%x ", roam_payload_vd->roam_scan_config[i]);
                    }
                    break;
                    case WIFI_EVENT_ASSOCIATION_REQUESTED:
                    case WIFI_EVENT_AUTH_COMPLETE:
                    case WIFI_EVENT_ASSOC_COMPLETE:
                    case WIFI_EVENT_FW_AUTH_STARTED:
                    case WIFI_EVENT_FW_ASSOC_STARTED:
                    case WIFI_EVENT_FW_RE_ASSOC_STARTED:
                    case WIFI_EVENT_DRIVER_SCAN_REQUESTED:
                    case WIFI_EVENT_DRIVER_SCAN_RESULT_FOUND:
                    case WIFI_EVENT_DRIVER_SCAN_COMPLETE:
                    case WIFI_EVENT_DISASSOCIATION_REQUESTED:
                    case WIFI_EVENT_RE_ASSOCIATION_REQUESTED:
                    case WIFI_EVENT_ROAM_AUTH_STARTED:
                    case WIFI_EVENT_ROAM_AUTH_COMPLETE:
                    case WIFI_EVENT_ROAM_ASSOC_STARTED:
                    case WIFI_EVENT_ROAM_ASSOC_COMPLETE:
                    case WIFI_EVENT_ASSOC_TIMEOUT:
                    case WIFI_EVENT_AUTH_TIMEOUT:
                    {
                        pe_event_vendor_data_t *pe_event_vd
                         = (pe_event_vendor_data_t *)&tlvs->value[0];
                        fprintf(ofile, "       sme_state  : %u\n", pe_event_vd->sme_state);
                        fprintf(ofile, "       mlm_state  : %u\n", pe_event_vd->mlm_state);
                    }
                    break;
                    case WIFI_EVENT_BEACON_RECEIVED:
                    {
                        beacon_event_data_t *pe_event_vd
                         = (beacon_event_data_t *)&tlvs->value[0];
                        fprintf(ofile, "       BSSID: ");
                        fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&pe_event_vd->bssid[0]));
                        fprintf(ofile, "       RSSI: %u\n", pe_event_vd->rssi);
                    }
                    break;
                    case WIFI_EVENT_RESOURCE_FAILURE:
                    {
                        wlan_low_resource_failure_event_t *pe_event_vd
                         = (wlan_low_resource_failure_event_t *)&tlvs->value[0];
                        fprintf(ofile, "       Failure Type: %u\n", pe_event_vd->event_sub_type);
                    }
                    break;
                        //TODO: add other vendor spevific event parsing similar to this
                    default:
                        fprintf(ofile, "Unknown event: %d", conn_entry->event);
                }
            }
            break;
            case WIFI_TAG_BSSID:
                fprintf(ofile, "WIFI_TAG_BSSID: ");
#if 0
                fprintf(ofile, "\n");
                for(i=0; i<tlvs->length; i++)
                    fprintf(ofile, "%02x ", *(u8 *)&tlvs->value[i]);
                fprintf(ofile, "\n");
#endif
                fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&tlvs->value[0]));
            break;
            case WIFI_TAG_ADDR:
                fprintf(ofile, "WIFI_TAG_ADDR: ");
                fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&tlvs->value[0]));
            break;
            case WIFI_TAG_ADDR1:
                fprintf(ofile, "WIFI_TAG_ADDR1: ");
                fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&tlvs->value[0]));
            break;
            case WIFI_TAG_ADDR2:
                fprintf(ofile, "WIFI_TAG_ADDR2: ");
                fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&tlvs->value[0]));
            break;
            case WIFI_TAG_ADDR3:
                fprintf(ofile, "WIFI_TAG_ADDR3: ");
                fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&tlvs->value[0]));
            break;
            case WIFI_TAG_ADDR4:
                fprintf(ofile, "WIFI_TAG_ADDR4: ");
                fprintf(ofile, MAC_ADDR_STR, MAC_ADDR_ARRAY((u8 *)&tlvs->value[0]));
            break;
            case WIFI_TAG_SSID:
                fprintf(ofile, "WIFI_TAG_SSID: ");
                fprintf(ofile, "%s", (char *)&tlvs->value[0]); //null terminated? otherwise copy to a str
            break;
            case WIFI_TAG_STATUS:
                fprintf(ofile, "WIFI_TAG_STATUS: ");
                fprintf(ofile, "%d", *(u16 *)&tlvs->value[0]);
                addba_status = *(u16 *)&tlvs->value[0];
            break;
            case WIFI_TAG_CHANNEL_SPEC:
                fprintf(ofile, "WIFI_TAG_CHANNEL_SPEC\n");
                chan = (wifi_channel_spec *)&tlvs->value[0];
                fprintf(ofile, "width             : %d\n", chan->width);
                fprintf(ofile, "center_frequency0 : %d\n", chan->center_frequency0);
                fprintf(ofile, "center_frequency1 : %d\n", chan->center_frequency1);
                fprintf(ofile, "primary_frequency : %d\n", chan->primary_frequency);
            break;
            case WIFI_TAG_WAKE_LOCK_EVENT:
                fprintf(ofile, "WIFI_TAG_WAKE_LOCK_EVENT\n");
                wle = (wake_lock_event *)&tlvs->value[0];
                fprintf(ofile, "status            : %d\n", wle->status);
                fprintf(ofile, "reason            : %d\n", wle->reason);
                fprintf(ofile, "name              : %s\n", wle->name);
            break;
            case WIFI_TAG_TSF:
                fprintf(ofile, "WIFI_TAG_TSF: ");
                fprintf(ofile, "%" PRIu64, *(uint64_t *)&tlvs->value[0]);
            break;
            case WIFI_TAG_IE:
                fprintf(ofile, "WIFI_TAG_IE: ");
                //TODO:
            break;
            case WIFI_TAG_INTERFACE:
                fprintf(ofile, "WIFI_TAG_INTERFACE: ");
                //TODO:
            break;
            case WIFI_TAG_REASON_CODE:
                fprintf(ofile, "WIFI_TAG_REASON_CODE :");
                fprintf(ofile, "%d", *(u16 *)&tlvs->value[0]);
            break;
            case WIFI_TAG_RATE_MBPS:
                fprintf(ofile, "WIFI_TAG_RATE_MBPS : ");
                fprintf(ofile, "%d", *(u16 *)&tlvs->value[0]);
            break;
            case WIFI_TAG_REQUEST_ID:
                fprintf(ofile, "WIFI_TAG_REQUEST_ID :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_BUCKET_ID:
                fprintf(ofile, "WIFI_TAG_BUCKET_ID :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_GSCAN_PARAMS:
                fprintf(ofile, "WIFI_TAG_GSCAN_PARAMS: ");
                //TODO:
            break;
            case WIFI_TAG_GSCAN_CAPABILITIES:
                fprintf(ofile, "WIFI_TAG_GSCAN_CAPABILITIES: ");
            break;
            case WIFI_TAG_SCAN_ID:
                fprintf(ofile, "WIFI_TAG_SCAN_ID: ");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_RSSI:
                fprintf(ofile, "WIFI_TAG_RSSI: ");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_CHANNEL:
                fprintf(ofile, "WIFI_TAG_CHANNEL :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_LINK_ID:
                fprintf(ofile, "WIFI_TAG_LINK_ID :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_LINK_ROLE:
                fprintf(ofile, "WIFI_TAG_LINK_ROLE :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_LINK_STATE:
                fprintf(ofile, "WIFI_TAG_LINK_STATE :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_LINK_TYPE:
                fprintf(ofile, "WIFI_TAG_LINK_TYPE :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_TSCO:
                fprintf(ofile, "WIFI_TAG_TSCO :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_RSCO:
                fprintf(ofile, "WIFI_TAG_RSCO :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
            case WIFI_TAG_EAPOL_MESSAGE_TYPE:
                fprintf(ofile, "WIFI_TAG_EAPOL_MESSAGE_TYPE :");
                fprintf(ofile, "%d", *(int *)&tlvs->value[0]);
            break;
        }
        len += sizeof(tlv_log) + tlvs->length;
        tlvs = (tlv_log *)((u8 *)conn_entry->tlvs + len);
    }
}

int main (int argc, char *argv[])
{
    FILE *ifile, *ofile;
    rb_entry rb_node;
    unsigned char stats[512] = {0};
    int idx = 0;

    if(argc<2) {
       printf("provide file name as argument\n");
       printf("Ex: Usage\n./conn_stats_parser file.bin\n");
       return -1;
    }

    ifile = fopen(argv[1], "rb+");
    if (ifile == NULL) {
        printf("not able to open file");
        return -1;
    }

    if(argc >= 3) {
        ofile = fopen(argv[2], "w");
    } else {
        ofile = fopen("connectivity_events_rb.txt", "w");
    }

    if (ofile == NULL) {
        fclose(ifile);
        printf("not able to open file connectivity_events_rb.txt");
        return -1;
    }

    while (!feof(ifile)) {
        if (fread(&rb_node.rb_head, sizeof(wifi_ring_buffer_entry), 1, ifile) != 1) {
            printf("\nParsing done; connectivity stats @ connectivity_events_rb.txt\n");
            break;
        }

        if (fread(stats, rb_node.rb_head.entry_size, 1, ifile) != 1) {
            printf("\nfread failed/done\n");
            break;
        }

        rb_node.conn_entry = (wifi_ring_buffer_driver_connectivity_event *)stats;
        fprintf(ofile, "======================================\n");
        fprintf(ofile, "=== Connectivity event entry : %d ===\n", idx);
        fprintf(ofile, "======================================\n");
        uint64_t sec = rb_node.rb_head.timestamp/1000000;
        uint64_t min = sec/60;
        uint64_t hours = min/60;
        fprintf(ofile, "Timestamp : %" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 "\n", hours % 60, min % 60, sec % 60, rb_node.rb_head.timestamp - (sec*1000000));

        parse_connectivity_event(ofile, rb_node.conn_entry, rb_node.rb_head.entry_size);
        fprintf(ofile, "\n======================================\n\n\n");

        idx++;
    }
    return 0;
}
