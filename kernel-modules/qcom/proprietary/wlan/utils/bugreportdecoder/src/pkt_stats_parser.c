/*
 * Copyright (c) 2016 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define INVALID_RATE_CODE 0XFF

typedef struct pps_entry_s {
     uint8_t flags;
     uint8_t tid; // transmit or received tid
     uint16_t MCS; // modulation and bandwidth
     uint8_t rssi; // TX: RSSI of ACK for that packet
              // RX: RSSI of packet
     uint8_t num_retries; // number of attempted retries
     uint16_t last_transmit_rate; // last transmit rate in .5 mbps
     uint16_t link_layer_transmit_sequence; // transmit/reeive sequence for that MPDU packet
     uint64_t firmware_entry_timestamp;  // TX: firmware timestamp (us) when packet is queued within firmware buffer
                                    // for SDIO/HSIC or into PCIe buffer
                                    // RX : firmware receive timestamp
     uint64_t start_contention_timestamp; // firmware timestamp (us) when packet start contending for the
                                     // medium for the first time, at head of its AC queue,
                                     // or as part of an MPDU or A-MPDU. This timestamp is not updated
                                     // for each retry, only the first transmit attempt.
     uint64_t transmit_success_timestamp; // fimrware timestamp (us) when packet is successfully transmitted
                                     // or aborted because it has exhausted its maximum number of retries
     uint8_t data[0]; // packet data. The length of packet data is determined by the entry_size field of
                 // the wifi_ring_buffer_entry structure. It is expected that first bytes of the
                 // packet, or packet headers only (up to TCP or RTP/UDP headers) will be copied into the ring
 } __attribute__((packed)) wifi_ring_per_packet_status_entry;

typedef struct {
    uint16_t entry_size; // the size of payload excluding the header.
    uint8_t flags;
    uint8_t type; // Entry type
    uint64_t timestamp; //present if has_timestamp bit is set.
} __attribute__((packed)) wifi_ring_buffer_entry;

typedef struct {
    wifi_ring_buffer_entry rb_head;
    wifi_ring_per_packet_status_entry *pps_entry;
}rb_entry;

const char *get_bw(unsigned char bw)
{
    char *bw_s;
    switch (bw)
    {
        case 0:
            bw_s = "20MHz";
        break;
        case 1:
            bw_s = "40MHz";
        break;
        case 2:
            bw_s = "80MHz";
        break;
        case 3:
            bw_s = "160MHz";
        break;
    }

    return bw_s;
}

const char* get_mcsindex(unsigned short MCS, char *rate)
{
    if ((((MCS >> 6) & 0x3) == 0) || (((MCS >> 6) & 0x3) == 1))
    {
        rate[0] = 'N';
        rate[1] = 'A';
        rate[2] = '\0';
    } else {
        snprintf(rate, 2, "%d", (MCS & 0xF));
    }

    return rate;
}

int main (int argc, char *argv[])
{
    FILE *ifile, *ofile, *payload_file;
    rb_entry rb_node;
    unsigned char pkt_stats[1024] = {0};
    int idx = 0, i;
    bool invalid_rate_code;

    if(argc<2) {
       printf("provide file name as argument\n");
       printf("Ex: Usage\n./pkt_stats_parser pkt_stats_rb.bin\n");
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
        ofile = fopen("pkt_stats_rb.txt", "w");
    }

    if (ofile == NULL) {
        fclose(ifile);
        printf("not able to open file pkt_stats_rb.txt");
        return -1;
    }

    payload_file = fopen("pkt_stats_payload.txt", "w");

    if (payload_file == NULL) {
        fclose(ifile);
        fclose(ofile);
        printf("not able to open file pkt_stats_payload.txt");
        return -1;
    }

    while (!feof(ifile)) {
        if (fread(&rb_node.rb_head, sizeof(wifi_ring_buffer_entry), 1, ifile) != 1) {
            printf("\nParsing done; packet stats @ pkt_stats_rb.txt\n");
            break;
        }

        if (fread(pkt_stats, rb_node.rb_head.entry_size, 1, ifile) != 1) {
            printf("\nfread failed/done failed for payload size : %d\n", rb_node.rb_head.entry_size);
            break;
        }

        rb_node.pps_entry = (wifi_ring_per_packet_status_entry *)pkt_stats;
        invalid_rate_code = ((rb_node.pps_entry->MCS & INVALID_RATE_CODE) == INVALID_RATE_CODE);
        char rate_index[3];
        if ((idx % 16) == 0) {
            fprintf(ofile, "\n\n    idx       Timestamp(TOD)   Flags           Tx/Rx     Success/Failure    Header     Protected   TID     MCS (rate nss preamble BW)    RSSI   retries rate    seq_no firmware_timestamp contention_timestamp success_timestamp              Payload ");
        }

        fprintf(ofile, "\n%8d     %"PRIu64"       %02x               %s        %s                %s         %s        %2d     %04x (%s  %s  %2x  %s)    %d %s %4d   %4d      %4d  %"PRIu64"  %"PRIu64"  %"PRIu64"               ",
                idx,
		rb_node.rb_head.timestamp,
                rb_node.pps_entry->flags,
                (rb_node.pps_entry->flags & 1)?"RX":"TX",
                (rb_node.pps_entry->flags & 2)?"S":"F",
                (rb_node.pps_entry->flags & 4)?"802.11":"802.3 ",
                (rb_node.pps_entry->flags & 8)?"Y":"N",
                rb_node.pps_entry->tid,
                invalid_rate_code ? 0xff : rb_node.pps_entry->MCS,
                invalid_rate_code ? "NA" : get_mcsindex(rb_node.pps_entry->MCS, rate_index),
                invalid_rate_code ? "NA" : ((rb_node.pps_entry->MCS >> 4) & 0x3)?"2x2":"1x1",
                invalid_rate_code ? 0x0  : (rb_node.pps_entry->MCS >> 6) & 0x3,
                invalid_rate_code ? "NA" : get_bw((rb_node.pps_entry->MCS >> 8 ) & 0x3),
                rb_node.pps_entry->rssi,
                (rb_node.pps_entry->rssi == 255)?"NA":"   ",
                rb_node.pps_entry->num_retries,
                rb_node.pps_entry->last_transmit_rate,
                rb_node.pps_entry->link_layer_transmit_sequence,
                rb_node.pps_entry->firmware_entry_timestamp,
                rb_node.pps_entry->start_contention_timestamp,
                rb_node.pps_entry->transmit_success_timestamp
                );
        for (i=0; i<rb_node.rb_head.entry_size - sizeof(wifi_ring_per_packet_status_entry); i++) {
            fprintf(ofile, "%02x ", rb_node.pps_entry->data[i]);
            if (!(i&0xF))
                fprintf(payload_file, "\n%06x", i);
            fprintf(payload_file, " %02x", rb_node.pps_entry->data[i]);
        }

        if (rb_node.rb_head.entry_size - sizeof(wifi_ring_per_packet_status_entry)) {
            fprintf(payload_file, "\n");
        }
        idx++;
    }
    fclose(ifile);
    fclose(ofile);
    printf("\nHex dump of the payload @ pkt_stats_payload.txt\n");
    fclose(payload_file);

    return 0;
}
