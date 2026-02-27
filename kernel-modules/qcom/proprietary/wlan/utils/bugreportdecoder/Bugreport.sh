#!/bin/sh
#
#  Copyright (c) 2016 Qualcomm Atheros, Inc.
#
#  All Rights Reserved.
#  Qualcomm Atheros Confidential and Proprietary.
date
if [ "$1" == "" ]
then
    echo "Please provide bugreport file as argument"
    exit
else
    mkdir -p bin
    if [ ! -f bin/conn_stats_parser ]
    then
        gcc src/conn_stats_parser.c -o bin/conn_stats_parser
    fi
    if [ ! -f bin/driver_prints_parser.c ]
    then
        gcc src/driver_prints_parser.c -o bin/driver_prints_parser
    fi
    if [ ! -f bin/pkt_stats_parser.c ]
    then
        gcc src/pkt_stats_parser.c -o bin/pkt_stats_parser
    fi
    if [ ! -f bin/power_events_parser.c ]
    then
        gcc src/power_events_parser.c -o bin/power_events_parser
    fi
    if [ ! -f WifiBugReportParser.class ]
    then
        javac src/WifiBugReportParser.java
        cp src/WifiBugReportParser.class .
    fi
    java WifiBugReportParser $1
    if [ ! -d BugReport_* ]
    then
        echo "No bugreports are present"
        exit
    fi
    newest=$(ls -d BugReport*| sort -t _ -k 2,2 | tail -n 1)
fi

for d in $newest/*; do
    echo $d
if [ -f "$d/driver_prints_rb.bin" ]
then
    bin/driver_prints_parser $d/driver_prints_rb.bin $d/driver_prints_rb.txt
fi
if [ -f "$d/pkt_stats_rb.bin" ]
then
    bin/pkt_stats_parser $d/pkt_stats_rb.bin $d/pkt_stats_rb.txt
fi
if [ -f "$d/connectivity_events_rb.bin" ]
then
    bin/conn_stats_parser $d/connectivity_events_rb.bin $d/connectivity_events_rb.txt
fi
if [ -f "$d/power_events_rb.bin" ]
then
    bin/power_events_parser $d/power_events_rb.bin $d/power_events_rb.txt
fi
if [ -f "$d/firmware_prints.bin" ]
then
    chmod 777 $d/firmware_prints.bin
    bin/fwlogparser $d/firmware_prints.bin > $d/firmware_prints.txt
fi
done
if [ "$2" != "" ]
then
    sort -m $2 $newest/WifiQualifiedNetworkSelector.txt -o $newest/logcat_ext.txt
fi

rm tempBugRepRBs_
date
