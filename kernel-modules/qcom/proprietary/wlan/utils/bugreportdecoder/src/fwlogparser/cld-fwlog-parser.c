/*
 * Copyright (c) 2016-17 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <athdefs.h>
#include <a_types.h>
#include <ctype.h>
#include "dbglog.h"
#include "dbglog_id.h"
#include "a_debug.h"
#include <linux/netlink.h>
#include "cld-diag-parser.h"

static FILE *log_in;
FILE *log_out = NULL;

unsigned char buf[RECLEN];
uint8_t read_buff[READ_LEN];

#define TRUE 1
#define FALSE 0
#define MAX_DBG_MSGS 256

typedef struct {
    uint16_t entry_size; // the size of payload excluding the header.
    uint8_t flags;
    uint8_t type;        // entry type
    uint64_t timestamp;  // present if has_timestamp bit is set.
} __attribute__((packed)) wifi_ring_buffer_entry;
typedef unsigned long addr_t;
module_dbg_print mod_print[WLAN_MODULE_ID_MAX];
void hexdump(const void *ptr, size_t len)
{
	addr_t address = (addr_t)ptr;
	size_t count;
	int i;

	for (count = 0 ; count < len; count += 16) {
		printf("0x%08lx: ", address);
		printf("%08x %08x %08x %08x |", *(const uint32_t *)address, *(const uint32_t *)(address + 4), *(const uint32_t *)(address + 8), *(const uint32_t *)(address + 12));
		for (i=0; i < 16; i++) {
			char c = *(const char *)(address + i);
			if (isalpha(c)) {
				printf("%c", c);
			} else {
				printf(".");
			}
		}
		printf("|\n");
		address += 16;
	}
}

int main(int argc, char *argv[])
{
    int res,ret;
    wifi_ring_buffer_entry *slot;
    unsigned int length = 0;
    uint16_t parse_flag = 0;

    while ((ret = getopt(argc, argv, "p:")) != -1) {
	switch (ret) {
		case 'p':
			parse_flag |= WITHOUT_RING_FLAG;
	}
    }

    if (optind != 3 && argc != 2) {
        fprintf(stderr, "usage:\n"
            "%s -p <path to encrpyt log file> \n"
            "%s <path to log file>\n"
            "for example:\n"
            "ath6kl-fwlog-parser /tmp/cld-fwlog\n",
            argv[0],argv[0]);
        return -1;
    }
    if (parse_flag & WITHOUT_RING_FLAG)
	argv += 1;
    parser_init();

    log_in = fopen(argv[1], "r");
    if (log_in == NULL) {
        perror("Failed to open input file");
        return -1;
    }
    fseek(log_in, 0, SEEK_END);
    length = ftell(log_in);
    fseek(log_in, 0, SEEK_SET);
    if (!(parse_flag & WITHOUT_RING_FLAG)) {
	while ((res = fread(buf, sizeof(wifi_ring_buffer_entry), 1, log_in)) == 1) {
	    length = 0;
	    slot =(wifi_ring_buffer_entry *)buf;
	    length = slot->entry_size;
	    //fprintf(stderr, "Length %d \n", length);
	    memset(buf, 0, sizeof(wifi_ring_buffer_entry));
	    fread(buf, length, 1, log_in);
	    //hexdump(buf, length);
	    parse_diagfw_msg((uint8_t *)buf, length, log_out, parse_flag);
	    memset(buf, 0, length);
	}
    } else {
	 if (length >= READ_LEN) {
	     printf("Failed to parse  %ld size of file\n"
		    "Parse upto %dMB size of file\n",
		    length,(READ_LEN/(1024*1024)));
	     return -1;
	 }
	 while ((res = fread(read_buff, length, 1, log_in)) == 1) {
	     parse_diagfw_msg((uint8_t *)read_buff, length, log_out,
			      parse_flag);
	     memset(read_buff, 0, length);
	 }
    }
    fclose(log_in);
    return 0;
}
