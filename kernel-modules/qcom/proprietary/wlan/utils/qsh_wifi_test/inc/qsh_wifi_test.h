/*
 * Copyright (c) 2020,2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef QSH_WIFI_TEST_H
#define QSH_WIFI_TEST_H

#include "qmi_client.h"

#include "sns_std_type.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_client.pb.h"
#include "sns_suid.pb.h"
#include "qsh_wifi.pb.h"
#include <libxml/parser.h>
#include <libxml/tree.h>

/*=============================================================================
  Macro Definitions
  ===========================================================================*/
#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "QSH_WIFI_TEST"

#define SENSOR_NAME "wifi_svc"

#ifndef UNUSED_VAR
#define UNUSED_VAR(var) ((void)(var));
#endif


#define MAC_ADDR_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

#define MACADDR(mac_addr) \
  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]

#define TAG "QSH_WIFI_TEST"
/* Timeout for CMD response */
#define DEF_CMD_TIMEOUT_MS 1000 // 10 seconds
//Already defined at => system/logging/liblog/include_vndk/log/log_time.h
//#define NS_PER_SEC 1000000000 //nano seconds in per second
//#define MS_PER_SEC 1000       //mili seconds in per second
#define NS_PER_MS (NS_PER_SEC/MS_PER_SEC)
#define CMD_TIMEOUT_IN_MS 5000

#define MAX_AP_IN_SCAN_EVT    15
#define MAX_FREQ_IN_SCAN_EVT 128
#define MAX_SCAN_EVENTS       20
#define MAX_SSID_CNT          10
#define MAX_SSID_SIZE         32
#define MAX_REQ_RANGING_BSS   20
#define MAX_FRAG_RANGING_BSS  10
#define MAX_RTT_EVENTS        2

#define SCAN_FILE_PATH "/data/scan.xml"
const xmlChar XML_NODE_SCAN[]            = "scan";
const xmlChar XML_NODE_SCAN_TYPE[]       = "scan_type";
const xmlChar XML_NODE_SCAN_AGE_MS[]     = "age_in_ms";
const xmlChar XML_NODE_BAND[]            = "band";
const xmlChar XML_NODE_RADIO_CHAIN[]     = "radio_chain";
const xmlChar XML_NODE_FREQS[]           = "freqs";
const xmlChar XML_NODE_SSIDS[]           = "ssids";

#define RTT_FILE_PATH "/data/rtt.xml"
const xmlChar XML_NODE_RTT[]            = "rtt";
const xmlChar XML_NODE_AP[]             = "ap";
const xmlChar XML_NODE_MAC[]            = "mac";
const xmlChar XML_NODE_FREQ[]           = "freq";
const xmlChar XML_NODE_CF0[]            = "cent_freq0";
const xmlChar XML_NODE_CF1[]            = "cent_freq1";
const xmlChar XML_NODE_CW[]             = "ch_width";
const xmlChar XML_NODE_LCI[]            = "lci";
const xmlChar XML_NODE_NUM_FRAMES[]     = "num_frames";

/*=============================================================================
  Data Type Definitions
  ===========================================================================*/

enum CMDS
{
  OPEN                           = 1,
  MONITOR_MODE_ON                = 2,
  MONITOR_MODE_OFF               = 3,
  ON_DEMAND_SCAN                 = 4,
  ON_DEMAND_SCAN_USER_PARAMS     = 5,
  RTT_SCAN                       = 6,

  DUMP_EVENT                     = 8,
  EXIT                           = 9,
};

enum init_state
{
  INIT   = 0,
  DEINIT = 1,
  OPENED = 2,
};

int parse_rtt_xml(const char * const rtt_xml,
    qsh_wifi_cmd_ranging_req *cmd );
#endif //QSH_WIFI_TEST_H
