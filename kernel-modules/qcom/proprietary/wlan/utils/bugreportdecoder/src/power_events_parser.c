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
    "WIFI_EVENT_ROAM_SCAN_CONFIG"
};

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

char *wl_reason_code[] = {
       "WIFI_POWER_EVENT_WAKELOCK_DRIVER_INIT",
       "WIFI_POWER_EVENT_WAKELOCK_DRIVER_REINIT",
       "WIFI_POWER_EVENT_WAKELOCK_SCAN",
       "WIFI_POWER_EVENT_WAKELOCK_RESUME_WLAN",
       "WIFI_POWER_EVENT_WAKELOCK_ROC",
       "WIFI_POWER_EVENT_WAKELOCK_HOLD_RX",
       "WIFI_POWER_EVENT_WAKELOCK_SAP",
};

char *wl_status_code[] = {
	"WIFI_POWER_EVENT_WAKELOCK_TAKEN",
	"WIFI_POWER_EVENT_WAKELOCK_RELEASED"
};

typedef struct {
    u16 tag;
    u16 length; // length of value
    u8 value[0];
} tlv_log;

typedef struct {
    uint16_t entry_size; // the size of payload excluding the header.
    uint8_t flags;
    uint8_t type; // Entry type
    uint64_t timestamp; //present if has_timestamp bit is set.
} __attribute__((packed)) wifi_ring_buffer_entry;

/**
 * Ring buffer name for power events ring. note that power event are extremely frequents
 * and thus should be stored in their own ring/file so as not to clobber connectivity events.
 */
typedef struct {
    int status;      // 0 taken, 1 released
    int reason;      // reason why this wake lock is taken
    char name[0];    // null terminated
} __attribute__((packed)) wake_lock_event;

typedef struct {
    u16 event;
    tlv_log tlvs[0];
} __attribute__((packed)) wifi_power_event;

typedef struct {
    wifi_ring_buffer_entry rb_head;
    wifi_power_event *power_entry;
}rb_entry;

void parse_power_events_stats(FILE *ofile, wifi_power_event *power_entry, u16 buflen)
{
    wake_lock_event *wle;
    tlv_log *tlvs = power_entry->tlvs;

	switch (power_entry->event)
	{
		case WIFI_TAG_WAKE_LOCK_EVENT:
		{
			wle = (wake_lock_event *)&tlvs->value[0];
			fprintf(ofile, "TAG: %d, length :%d\n", tlvs->tag, tlvs->length);
			fprintf(ofile, "status: %s, reason: %s, Name: %s\n", wl_status_code[wle->status], wl_reason_code[wle->reason], wle->name);
		}
		break;
		default:
			fprintf(ofile, "Unknown event\n");
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
        ofile = fopen("power_events_rb.txt", "w");
    }

    if (ofile == NULL) {
        fclose(ifile);
        printf("not able to open file power_events_rb.txt");
        return -1;
    }

    while (!feof(ifile)) {
        if (fread(&rb_node.rb_head, sizeof(wifi_ring_buffer_entry), 1, ifile) != 1) {
            printf("\nParsing done; connectivity stats @ power_events_stats.txt\n");
            break;
        }

        if (fread(stats, rb_node.rb_head.entry_size, 1, ifile) != 1) {
            printf("\nfread failed/done\n");
            break;
        }

        rb_node.power_entry = (wifi_power_event *)stats;
        fprintf(ofile, "======================================\n");
        fprintf(ofile, "=== Power event entry : %d ===\n", idx);
        fprintf(ofile, "======================================\n");
        uint64_t sec = rb_node.rb_head.timestamp/1000000;
        uint64_t min = sec/60;
        uint64_t hours = min/60;
        fprintf(ofile, "Timestamp : %" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 "\n", hours % 60, min % 60, sec % 60, rb_node.rb_head.timestamp - (sec*1000000));

        parse_power_events_stats(ofile, rb_node.power_entry, rb_node.rb_head.entry_size);
        fprintf(ofile, "\n======================================\n\n\n");

        idx++;
    }
    return 0;
}
