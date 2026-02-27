/*
 * Copyright (c) 2016 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

typedef struct {
    uint16_t entry_size; // the size of payload excluding the header.
    uint8_t flags;
    uint8_t type; // Entry type
    uint64_t timestamp; //present if has_timestamp bit is set.
} __attribute__((packed)) wifi_ring_buffer_entry;

typedef struct {
    wifi_ring_buffer_entry rb_head;
    void *drv_prints;
}rb_entry;

int main (int argc, char *argv[])
{
    FILE *ifile, *ofile;
    rb_entry rb_node;
    unsigned char drv_prints[8192] = {0};
    int bytes_read = 0;

    if(argc<2) {
       printf("provide file name as argument\n");
       printf("Ex: Usage\n./driver_prints_parser driver_prints_rb.bin\n");
       return -1;
    }

    ifile = fopen(argv[1], "rb+");
    if (ifile == NULL) {
        printf("not able to open file input file");
        return -1;
    }

    if(argc >= 3) {
        ofile = fopen(argv[2], "w");
    } else {
        ofile = fopen("driver_prints_rb.txt", "w");
    }

    if (ofile == NULL) {
        fclose(ifile);
        printf("not able to open file driver_prints_rb.txt");
        return -1;
    }

    while (!feof(ifile)) {
        if (fread(&rb_node.rb_head, sizeof(wifi_ring_buffer_entry), 1, ifile) != 1) {
            printf("\nParsing done; driver prints @ driver_prints_rb.txt\n");
            break;
        }

        bytes_read += sizeof(wifi_ring_buffer_entry);
//        printf("\nBytes read : %d payload : %d\n", bytes_read, rb_node.rb_head.entry_size);
        if (fread(drv_prints, rb_node.rb_head.entry_size, 1, ifile) != 1) {
            printf("\nfread failed/done failed for payload size : %d\n", rb_node.rb_head.entry_size);
            break;
        }
        bytes_read += rb_node.rb_head.entry_size;

        fwrite (drv_prints, rb_node.rb_head.entry_size, 1, ofile);
    }
    fclose(ifile);
    fclose(ofile);

    return 0;
}
