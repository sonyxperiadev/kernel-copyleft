#!/bin/sh
#
#  Copyright (c) 2016 Qualcomm Atheros, Inc.
#
#  All Rights Reserved.
#  Qualcomm Atheros Confidential and Proprietary.
echo 'Compiling...'
date
mkdir -p bin
gcc -Wall src/conn_stats_parser.c -o bin/conn_stats_parser
gcc -Wall src/power_events_parser.c -o bin/power_events_parser
gcc -Wall src/pkt_stats_parser.c -o bin/pkt_stats_parser
gcc -Wall src/driver_prints_parser.c -o bin/driver_prints_parser
cd src/fwlogparser/ && make clean; make; cp fwlogparser ../../bin/.; cd -;
echo 'done'
