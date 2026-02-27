/*=============================================================================
  @file qsh_wifi_test.cpp

  QSH WIFI TEST application.  This application will open a QMI connection to
  the Sensors Service (which resides on the SSC).  It will send a SUID Lookup
  Request for the data type "wifi_svc". For each found SUID, this application
  will send an Attributes Request, and subsequently an sends request with
  to Sensor.

  Copyright (c) 2023 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
  ===========================================================================*/

/*=============================================================================
  Include Files
  ===========================================================================*/

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <pthread.h>
#include <android/log.h>
#include <qsh_wifi_test.h>
#include "qsh_wifi_suid_util.h"

using namespace std;

/*=============================================================================
  Global Data
  ===========================================================================*/
// Scan Results place holder for Request Based Scan
qsh_wifi_evt_scan_result g_scan_evt_list[MAX_SCAN_EVENTS];
uint32_t g_scan_evt_list_count = 0;
bool is_scan_requested = 0;
pthread_mutex_t scan_mutex;

// Scan Results place holder for Monitor Scan
qsh_wifi_evt_scan_result g_mon_evt_list[MAX_SCAN_EVENTS];
uint32_t g_mon_evt_list_count = 0;
bool is_monitor_on = false;

// Results place holder for Ranging Request
qsh_wifi_evt_ranging_result g_rtt_evt_list[MAX_RTT_EVENTS];
uint32_t g_rtt_evt_list_count = 0;
bool is_rtt_requested = false;
pthread_mutex_t rtt_mutex;

enum init_state state = DEINIT;
pthread_mutex_t mutex_m;
pthread_cond_t  cond;
pid_t pid;

/* KPI DUMP MASK on console -kdm */
#define DUMP_KPI_EXT  0x0001 // Mask for VENDORS ( default value)
#define DUMP_KPI_INT  0x0010 // Mask for dumping complete kpi

#define CAN_DUMP_INT_KPI_STATS(mask) (mask & DUMP_KPI_INT)
#define CAN_DUMP_EXT_KPI_STATS(mask) (mask & DUMP_KPI_EXT)

/* KPI FS Log MASK -kfm */
enum qsh_wifi_log_mask
{
  M_HIGH = 0x0001,
  M_MED  = 0x0002,
  M_LOW  = 0x0004
};

/*=============================================================================
  Other Static Function Definitions
  ===========================================================================*/
static inline void qsh_wifi_unblock_th()
{
  pthread_mutex_lock(&mutex_m);
  pthread_cond_signal(&cond);
  pthread_mutex_unlock(&mutex_m);
}

/*=============================================================================
 * qsh_wifi_wait_for_signal_or_timeout
 *
 * Description:
 *   Wait for a signal or timeout
 *
 * Return value:
 *   1 if signaled
 *   0 if timed out
 *===========================================================================*/
static int qsh_wifi_timeout_block(
              uint32_t timeout, pthread_cond_t* cmd_cond)
{
  struct timespec present_time;
  int ret_val = -1;

  if (0 == clock_gettime(CLOCK_REALTIME, &present_time)) {
    present_time.tv_sec += timeout / MS_PER_SEC;
    present_time.tv_nsec += ((timeout % MS_PER_SEC) * NS_PER_MS);
    if ((unsigned long long)present_time.tv_nsec > NS_PER_SEC) {
      present_time.tv_sec++;
      present_time.tv_nsec -= (NS_PER_SEC);
    }

    pthread_mutex_lock(&mutex_m);
    ret_val = pthread_cond_timedwait(cmd_cond,
                                     &mutex_m,
                                     &present_time);
    pthread_mutex_unlock(&mutex_m);
  }
  return ret_val;
}

uint32_t g_log_mask = 7;

static void
qsh_wifi_printk(enum qsh_wifi_log_mask mask, const char *fmt, ...)
{
  if (g_log_mask & mask) {
    va_list args;
    va_start (args, fmt);
    __android_log_print(ANDROID_LOG_WARN, TAG, fmt, args);
    va_end (args);
  }
}

#define __qsh_wifi_pr qsh_wifi_printk
#define qsh_wifi_pr(log_level, format, args...) \
        __qsh_wifi_pr(log_level, format, ## args)

#define qsh_wifi_pr_high(params...) qsh_wifi_pr(M_HIGH, ## params)
#define qsh_wifi_pr_med(params...)  qsh_wifi_pr(M_MED, ## params)
#define qsh_wifi_pr_low(params...)  qsh_wifi_pr(M_LOW, ## params)

//fwd declarations
int qsh_wifi_init ();

/*=============================================================================
  Static Variable Definitions
  ===========================================================================*/

/* SSC interface used to send a request to the Sensor service */
static qsh_interface *connection = nullptr;

/* SUID for the wifi Sensor, as received from the SUID Lookup Sensor */
static sensor_uid wifi_suid;

/*=============================================================================
  Static Function Definitions
  ===========================================================================*/

char const* get_cmd_name(qsh_wifi_msgid a)
{
  switch ( a )
  {
    case QSH_WIFI_MSGID_QSH_WIFI_CMD_OPEN:
      return "CMD_OPEN";
    case QSH_WIFI_MSGID_QSH_WIFI_CMD_SCAN_REQ:
      return "CMD_SCAN_REQ";
    case QSH_WIFI_MSGID_QSH_WIFI_CMD_MONITOR_REQ:
      return "CMD_MONITOR_REQ";
    case QSH_WIFI_MSGID_QSH_WIFI_CMD_RANGING_REQ:
      return "CMD_RANGING_REQ";
    default:
      return "INVALID_WIFI_CMD";
  }
}

char const* get_event_name(uint32_t a)
{
  switch ( a )
  {
    case SNS_STD_MSGID_SNS_STD_ATTR_EVENT:
      return "SNS_STD_ATTR_EVENT";
    case SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT:
      return "SNS_STD_SENSOR_EVENT";
    case SNS_STD_MSGID_SNS_STD_ERROR_EVENT:
      return "SNS_STD_ERROR_EVENT";
    case QSH_WIFI_MSGID_QSH_WIFI_CMD_ACK:
      return "QSH_WIFI_CMD_ACK";
    case QSH_WIFI_MSGID_QSH_WIFI_EVT_SCAN_RESULTS:
      return "QSH_WIFI_EVT_SCAN_RESULTS";
    case QSH_WIFI_MSGID_QSH_WIFI_EVT_RANGING_RESULTS:
      return "QSH_WIFI_EVT_RANGING_RESULTS";
    default:
      return "INVALID_EVENT";
  }
}

char const* get_error_name(uint32_t a)
{
  switch ( a )
  {
    case QSH_WIFI_ERROR_NONE:
      return "QSH_WIFI_ERROR_NONE";
    case QSH_WIFI_ERROR:
      return "QSH_WIFI_ERROR";
    case QSH_WIFI_ERROR_INVALID_ARGS:
      return "QSH_WIFI_ERROR_INVALID_ARGS";
    case QSH_WIFI_ERROR_BUSY:
      return "QSH_WIFI_ERROR_BUSY";
    case QSH_WIFI_ERROR_NO_MEMORY:
      return "QSH_WIFI_ERROR_NO_MEMORY";
    case QSH_WIFI_ERROR_NOT_SUPPORTED:
      return "QSH_WIFI_ERROR_NOT_SUPPORTED";
    case QSH_WIFI_ERROR_TIMEOUT:
      return "QSH_WIFI_ERROR_TIMEOUT";
    case QSH_WIFI_ERROR_INTF_DISABLED:
      return "QSH_WIFI_ERROR_INTF_DISABLED";
    default:
      return "INVALID_ERROR";
  }
}

/**
 * Send a request to the specified service.  Do not specify any batching
 * options.
 *
 * @param[i] req_payload Encoded Sensor-specific request message
 * @param[i] suid Destination SUID
 * @param[i] msgid Sensor-specific message ID
 */
static int send_basic_req(qsh_interface *conn, sensor_uid &suid, qsh_wifi_msgid msgid, std::string req_payload)
{
  std::string pb_req_msg_encoded;
  sns_client_request_msg pb_req_msg;

  pb_req_msg.set_msg_id(msgid);
  sns_std_request* pStdReq = pb_req_msg.mutable_request();
  if (pStdReq) {
    pStdReq->set_payload(req_payload);
  } else {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "mutable_request failed!");
    return -1;
  }
  sns_std_suid* pStdSuid = pb_req_msg.mutable_suid();
  if (pStdSuid) {
    pStdSuid->set_suid_high(suid.high);
    pStdSuid->set_suid_low(suid.low);
  } else {
    pb_req_msg.clear_request();
    __android_log_print(ANDROID_LOG_ERROR, TAG, "mutable_suid failed!");
    return -1;
  }
  //The suspend config can be set according to the msgid, if reqd
  sns_client_request_msg_suspend_config* pSuspendCfg = pb_req_msg.mutable_susp_config();
  if (pSuspendCfg) {
    pSuspendCfg->set_delivery_type(SNS_CLIENT_DELIVERY_WAKEUP);
    pSuspendCfg->set_client_proc_type(SNS_STD_CLIENT_PROCESSOR_APSS);
  } else {
    pb_req_msg.clear_request();
    pb_req_msg.clear_suid();
    __android_log_print(ANDROID_LOG_ERROR, TAG, "mutable_susp_config failed!");
    return -1;
  }

  pb_req_msg.SerializeToString(&pb_req_msg_encoded);
  conn->send_request(suid, true, pb_req_msg_encoded);
  pb_req_msg.clear_request();
  pb_req_msg.clear_suid();
  pb_req_msg.clear_susp_config();

  return 0;
}

static int qsh_wifi_send_monitor_mode_req(
    qsh_interface *conn, sensor_uid &suid, bool action)
{
  std::string mon_cmd_encoded;
  qsh_wifi_cmd_monitor_req mon_req;
  qsh_wifi_msgid mon_cmd_msgid = QSH_WIFI_MSGID_QSH_WIFI_CMD_MONITOR_REQ;
  int ret_from_send_req = 0;
  int ret_for_ack = 0;

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Send QSH WIFI open request");

  // State should be initialize
  if (state == DEINIT) {
    std::cout << "\nInitialize the QMI Session first";
    return -1;
  }

  // Sanity for action
  if ((action == true) && (is_monitor_on == true)) {
    qsh_wifi_pr_high("Client is already requested monitor on, Ignore it");
    return 0;
  }

  if ((action == false) && (is_monitor_on == false)) {
    qsh_wifi_pr_high("Client have not requested monitor mode, Ignore it");
    return 0;
  }

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Send Monitor request %s", action ? "ON" : "OFF");

  //Prepare Monitor Mode Request
  qsh_wifi_monitor_action action_value = action ? QSH_WIFI_MONTIOR_ACTION_ON: QSH_WIFI_MONTIOR_ACTION_OFF;
  mon_req.set_action(action_value);
  mon_req.SerializeToString(&mon_cmd_encoded);

  //Send Monitor Mode Request
  ret_from_send_req = send_basic_req(conn, suid, mon_cmd_msgid, mon_cmd_encoded);
  if(ret_from_send_req != 0) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Send Basic Request Failed, ret = %d", ret_from_send_req);
    return -2;
  }

  qsh_wifi_pr_high("Waiting for Monitor mode Ack");
  ret_for_ack = qsh_wifi_timeout_block(CMD_TIMEOUT_IN_MS, &cond);
  if (ret_for_ack != 0) {
    std::cout << "\nError occured while waiting for Monitor mode ACK, ret: " << ret_for_ack;
    return -2;
  }
  if (action)
    is_monitor_on = true;
  else
    is_monitor_on = false;
  return 0;
}

static int
qsh_wifi_send_on_demand_scan(qsh_interface *conn, sensor_uid &suid,
    qsh_wifi_cmd_scan_req &scan_req)
{
  std::string scan_cmd_encoded;
  qsh_wifi_msgid scan_cmd_msgid = QSH_WIFI_MSGID_QSH_WIFI_CMD_SCAN_REQ;
  int ret_from_send_req = 0;
  int ret_for_ack = 0;

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Send QSH WIFI ON DEMAND SCAN request");

  // State should be initialize
  if (state == DEINIT) {
    std::cout << "\nInitialize the QMI Session first";
    return -1;
  }

  qsh_wifi_pr_high("Send on demand scan request");

  //Prepare the Request
  scan_req.SerializeToString(&scan_cmd_encoded);

  //Send the Request
  ret_from_send_req = send_basic_req(conn, suid, scan_cmd_msgid, scan_cmd_encoded);
  if(ret_from_send_req != 0) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Send Basic Request Failed, ret = %d", ret_from_send_req);
    return -2;
  }

  pthread_mutex_lock(&scan_mutex);
  // Reset the scan event result data base after sending the new scan request
  for(int i = 0; i < MAX_SCAN_EVENTS ; i++)
    g_scan_evt_list[i].Clear();
  g_scan_evt_list_count = 0;
  is_scan_requested = true;
  pthread_mutex_unlock(&scan_mutex);

  qsh_wifi_pr_high("Waiting for on demand scan ACK");
  ret_for_ack = qsh_wifi_timeout_block(CMD_TIMEOUT_IN_MS * 3, &cond);
  if (ret_for_ack != 0) {
    pthread_mutex_lock(&scan_mutex);
    is_scan_requested = false;
    pthread_mutex_unlock(&scan_mutex);
    std::cout << "\nError occured while waiting for ACK ON DEMAND RSP: " << ret_for_ack;
    return -2;
  }
  return 0;
}

static int
qsh_wifi_send_rtt_req(qsh_interface *conn, sensor_uid &suid, qsh_wifi_cmd_ranging_req &rtt_req)
{
  std::string rtt_cmd_encoded;
  qsh_wifi_msgid rtt_cmd_msgid = QSH_WIFI_MSGID_QSH_WIFI_CMD_RANGING_REQ;
  int ret_from_send_req = 0;
  int ret_for_ack = 0;

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Send QSH WIFI RTT SCAN request");

  // State should be initialize
  if (state == DEINIT) {
    std::cout << "\nInitialize the QMI Session first";
    return -1;
  }

  qsh_wifi_pr_high("Send Ranging request");

  //Prepare the request
  rtt_req.SerializeToString(&rtt_cmd_encoded);

  //Send the request
  ret_from_send_req = send_basic_req(conn, suid, rtt_cmd_msgid, rtt_cmd_encoded);
  if(ret_from_send_req != 0) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Send Basic Request Failed, ret = %d", ret_from_send_req);
    return -2;
  }

  pthread_mutex_lock(&rtt_mutex);
  // Reset the scan event result data base after sending the new scan request
  for(int i = 0; i < MAX_RTT_EVENTS; i++)
    g_rtt_evt_list[i].Clear();
  g_rtt_evt_list_count = 0;
  is_rtt_requested = true;
  pthread_mutex_unlock(&rtt_mutex);

  qsh_wifi_pr_high("Waiting for on RTT request Ack");
  ret_for_ack = qsh_wifi_timeout_block(CMD_TIMEOUT_IN_MS * 2, &cond);
  if (ret_for_ack != 0) {
    pthread_mutex_lock(&rtt_mutex);
    is_rtt_requested = false;
    pthread_mutex_unlock(&rtt_mutex);
    std::cout << "\nError occured while waiting for ACK ON RTT RSP: " << ret_for_ack;
    return -2;
  }
  return 0;
}

/**
* Send an QSH WIFI open request for the specified SUID.
*/
static int qsh_wifi_send_open_req(qsh_interface *conn, sensor_uid &suid) {
  std::string open_cmd_encoded;
  qsh_wifi_cmd_open open_cmd;
  qsh_wifi_msgid open_cmd_msgid = QSH_WIFI_MSGID_QSH_WIFI_CMD_OPEN;
  int ret_from_send_req = 0;
  int ret_for_ack = 0;

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Send QSH WIFI open request");

  if (nullptr == conn) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "conn is nullptr!");
    return -1;
  }

  //Prepare the request, The response to open command will be ACK
  // with capabilties and version embedded in it
  open_cmd.set_get_capability(true);   //Ask the QSH WIFI sensor to send capabilities
  open_cmd.set_get_version(true);      //Ask the QSH WIFI sensor to send version

  //Encode the request
  open_cmd.SerializeToString(&open_cmd_encoded);

  //Send to Open Request to QMI
  ret_from_send_req = send_basic_req(conn, suid, open_cmd_msgid, open_cmd_encoded);
  if(ret_from_send_req != 0) {
    __android_log_print(ANDROID_LOG_ERROR, TAG,"Send Basic Request Failed, ret = %d", ret_from_send_req);
    return -2;
  }

  qsh_wifi_pr_high("Waiting for Open Ack");
  ret_for_ack = qsh_wifi_timeout_block(CMD_TIMEOUT_IN_MS, &cond);
  if (ret_for_ack != 0) {
    std::cout << "\nError occured while waiting for Open Ready ret: " << ret_for_ack;
    return -2;
  }

  return 0;
}

static int qsh_wifi_send_dump_event(qsh_interface *conn, sensor_uid &suid)
{
  std::string dump_cmd_encoded;
  qsh_wifi_cmd_open dump_cmd;
  qsh_wifi_msgid dump_cmd_msgid = static_cast<qsh_wifi_msgid>(QSH_WIFI_MSGID_QSH_WIFI_CMD_OPEN-1);
  int ret_from_send_req = 0;

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Send QSH WIFI DUMP EVENT");

  // Before open intilization is essential.
  if (state != OPENED) {
    qsh_wifi_pr_high("QMI is not OPENED yet");
    return -1;
  }

  qsh_wifi_pr_high("Send dump event request");

  //Encode the request
  dump_cmd.SerializeToString(&dump_cmd_encoded);

  //Send the request
  ret_from_send_req = send_basic_req(conn, suid, dump_cmd_msgid, dump_cmd_encoded);
  if(ret_from_send_req != 0) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Send Basic Request Failed, ret = %d", ret_from_send_req);
    return -2;
  }

  return 0;
}

static void print_scan_events(qsh_wifi_evt_scan_result evt_list[MAX_SCAN_EVENTS],
    uint32_t list_count, bool on_console)
{
  uint32_t evt_cnt = 0;
  uint32_t total_ap = 0;

  if (on_console) {
    std::cout << "\n\n" << std::setw(4) << "SNO";
    std::cout << std::setw(15) << "MAC" << std::setw(15) << "freq";
    std::cout << std::setw(8) << "RSSI" << std::setw(8) << "SSID";
    std::cout << "\n----------------------------------------------------------";
  }
  for(evt_cnt = 0; evt_cnt < list_count; evt_cnt++)
  {
    uint32_t ap_cnt = 0;
    const qsh_wifi_evt_scan_result &msg = evt_list[evt_cnt];
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "status:%u last_in_series:%i result total:%i count:%i scan_type:(%i:%i) radio_chain:(%i :%i)",
      msg.status(), msg.last_in_series(), msg.result_total(), msg.result_count(),
      msg.has_scan_type(), msg.scan_type(),
      msg.has_radio_chain_pref(), msg.radio_chain_pref());
    for (ap_cnt = 0; ap_cnt < msg.result_count(); ap_cnt++)
    {
      total_ap++;
      const qsh_wifi_bss_info &ap = msg.ap_list(ap_cnt);
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "ap_no_in_evt:%i band:%i rssi:%i freq:%i width:%i cap_info %u CF0(%s %i) CF1(%s %i)",
          total_ap, ap.band(), ap.rssi(), ap.freq(), ap.channel_width(), ap.capability_info(),
          (ap.has_cent_freq0() == 1) ? "T" : "F", ap.cent_freq0(),
          (ap.has_cent_freq1() == 1) ? "T" : "F", ap.cent_freq1());
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "AP pairs age(%s, %u) flags(%s, %u) sec(%s %u) radio_chain(%s %d) rssi0 (%s %i) rssi1(%s %i)",
          (ap.has_age_ms() == 1) ? "T" : "F", ap.age_ms(),
          (ap.has_flags() == 1) ? "T" : "F", ap.flags(),
          (ap.has_security_mode() == 1) ? "T" : "F", ap.security_mode(),
          (ap.has_radio_chain() == 1) ? "T" : "F", ap.radio_chain(),
          (ap.has_rssi_chain0() == 1) ? "T" : "F", ap.rssi_chain0(),
          (ap.has_rssi_chain1() == 1) ? "T" : "F", ap.rssi_chain1());
      if (on_console) {
        std::cout << std::endl << std::right << std::setw(3) << total_ap << ")";
        std::cout << std::setw(4) << "" << std::setfill('0');
        for(int i = 0; i < 5 ; i++)
          std::cout << std::hex << std::setw(2) << static_cast<int>(ap.bssid()[i]) << ":" ;
        std::cout << std::hex << std::setw(2) << static_cast<int>(ap.bssid()[5]);
        std::cout << std::setfill(' ') << std::dec;
        std::cout << std::setw(4) << "" << std::setw(4) << ap.freq();
        std::cout << std::setw(3) << "" << std::setw(4) << ap.rssi();
        std::cout << std::setw(3) << "";
        std::cout << std::left << std::setw(32) << ap.ssid();
      }
    }
  }
}

void print_ranging_events(qsh_wifi_evt_ranging_result evt_list[MAX_RTT_EVENTS],
    uint32_t list_count, bool on_console)
{
  uint32_t evt_cnt = 0;
  uint32_t total_ap = 0;

  if (on_console) {
    std::cout << "\n\n" << std::setw(4) << "SNO" << std::right;
    std::cout << std::setw(15) << "MAC";
    std::cout << std::setw(15) << "RSSI";
    std::cout << std::setw(15) << "Distance(mm)";
  }
  for(evt_cnt = 0; evt_cnt < list_count; evt_cnt++)
  {
    uint32_t ap_cnt = 0;
    const qsh_wifi_evt_ranging_result &msg = evt_list[evt_cnt];
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "status:%u last_in_series:%i result total:%i count:%i",
        msg.status(), msg.last_in_series(), msg.result_total(), msg.result_count());
    for (ap_cnt = 0; ap_cnt < msg.result_count(); ap_cnt++)
    {
      total_ap++;
      const qsh_wifi_ranging_result_ap &ap = msg.bss_list(ap_cnt);
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "ap_no_in_evt:%i rtt_status:%i flag:%u  rssi:%i distance_mm:%i distance_dev:%d ",
          total_ap, ap.rtt_status(), ap.result_flag(),
          ap.rssi(), ap.distance(), ap.distance_stddev());

      if (ap.result_flag() & QSH_RTT_RESULT_HAS_LCI)
      {
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "lat(%s %lld) long (%s %lld) altitude(%s %i) alt_type(%s %d)  lat_unc(%s %u) long_unc(%s %u) alt_unc(%s %u)",
          (ap.has_latitude() == 1) ? "T" : "F", ap.latitude(),
          (ap.has_longitude() == 1) ? "T" : "F", ap.longitude(),
          (ap.has_altitude() == 1) ? "T" : "F", ap.altitude(),
          (ap.has_altitude_type() == 1) ? "T" : "F", ap.altitude_type(),
          (ap.has_latitude_uncertainty() == 1) ? "T" : "F", ap.latitude_uncertainty(),
          (ap.has_longitude_uncertainty() == 1) ? "T" : "F", ap.longitude_uncertainty(),
          (ap.has_altitude_uncertainty() == 1) ? "T" : "F", ap.altitude_uncertainty());
      }
      if (on_console) {
        std::cout << std::endl << std::right << std::setw(3) << total_ap << ")";
        std::cout << std::setw(4) << "" << std::setfill('0');
        for(int i = 0; i < 5 ; i++)
          std::cout << std::hex << std::setw(2) << static_cast<int>(ap.mac_addr()[i]) << ":" ;
        std::cout << std::hex << std::setw(2) << static_cast<int>(ap.mac_addr()[5]);
        std::cout << std::setfill(' ') << std::dec;
        std::cout << std::setw(4) << "" << std::setw(3) << ap.rssi();
        std::cout << std::setw(4) << "" << std::setw(3) << ap.distance();
      }
    }
  }
}

/**
 * QMI Error callback.  This would be our notification if the QMI connection
 * was lost.  A typical implementation would re-establish the connection, and
 * resend any active requests.
 *
 * See qmi_client_error_cb. */
static void
qsh_wifi_error_cb(qsh_interface_error error)
{
  qsh_wifi_pr_high("SSC Interface Error : %i", error);
  std::cout << "\nReceived SSC Interface error!";
}

/**
 * Event callback function, as registered with qsh_interface.
 * Handles a received event message from the Sensor Service.
 */
static void qsh_wifi_event_cb(const uint8_t *data, size_t size, uint64_t ts) {
  sns_client_event_msg pb_event_msg;
  UNUSED_VAR(ts);

  union sigval sig_value;
  bool waiting_for_scan_result = false;

  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received QMI indication with length %zu", size);

  if (nullptr == data) {
    __android_log_print(ANDROID_LOG_ERROR, TAG,"data is nullptr!");
    return;
  }

  sig_value.sival_int = 1;

  pb_event_msg.ParseFromArray(data, size);
  for (int i = 0; i < pb_event_msg.events_size(); i++) {
    const sns_client_event_msg_sns_client_event &pb_event= pb_event_msg.events(i);

    __android_log_print(ANDROID_LOG_DEBUG, TAG, "Event[%i] msg_id=%i, ts=%llu", i, pb_event.msg_id(),
             (unsigned long long)pb_event.timestamp());

    if (SNS_STD_MSGID_SNS_STD_ERROR_EVENT == pb_event.msg_id()) {
      sns_std_error_event error;
      error.ParseFromString(pb_event.payload());
      __android_log_print(ANDROID_LOG_ERROR, TAG, "Received error event %i", error.error());
    } else if (SNS_STD_MSGID_SNS_STD_ATTR_EVENT == pb_event.msg_id()) {
      sns_std_attr_event attr_event;
      int32_t attr_id;
      attr_event.ParseFromString(pb_event.payload());
      attr_id = attr_event.attributes(0).attr_id();
      __android_log_print(ANDROID_LOG_ERROR, TAG, "Received attr_event attr_id %d", attr_id);
    } else if (SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT == pb_event.msg_id()) {
      //TBD: Added from qsh-wifi-test.c, NOT present in qsh-location-test [I guess, NOT required]
      //This event was just a sample event in qsh-wifi-test, I guess
      //"Decode the sensor event (aka a sensor sample)."
      sns_std_sensor_event sensor_evt;
      sensor_evt.ParseFromString(pb_event.payload());
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received STANDARD SENSOR EVENT! NOT handled in qsh-wifi-test!");
    } else if (QSH_WIFI_MSGID_QSH_WIFI_CMD_ACK == pb_event.msg_id()) {
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received QSH_WIFI_MSGID_QSH_WIFI_CMD_ACK");
      qsh_wifi_cmd_ack ack;

      //Decode the event
      ack.ParseFromString(pb_event.payload());

      //Print the decoded object to logs and Console!
      if(ack.status() != QSH_WIFI_ERROR_NONE) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "Error received => %s", get_error_name(ack.status()));
      } else {
        if (ack.command() == QSH_WIFI_MSGID_QSH_WIFI_CMD_OPEN) {
          __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received ack for OPEN request");
          __android_log_print(ANDROID_LOG_VERBOSE, TAG, "has_version=%d version=%u", ack.has_version(), ack.version());
          __android_log_print(ANDROID_LOG_VERBOSE, TAG, "has_caps=%d caps=%u", ack.has_capabilities(), ack.capabilities());
          qsh_wifi_unblock_th();
        } else if (ack.command() == QSH_WIFI_MSGID_QSH_WIFI_CMD_MONITOR_REQ) {
          __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received Monitor Mode ACK");
          qsh_wifi_unblock_th();
        } else if (ack.command() == QSH_WIFI_MSGID_QSH_WIFI_CMD_RANGING_REQ) {
          __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received Scan Req ACK, waiting for RANGING Event from Sensor");
          // We will not unblock main thread untill we received last_in_series
          // message or time out happens unless sensor send ack as failure.
          // We won't continue the APP after successful ACK until we get the response too!
          if (ack.status() != QSH_WIFI_ERROR_NONE)  //If error occurs, only then unblock
            qsh_wifi_unblock_th();
          } else if (ack.command() == QSH_WIFI_MSGID_QSH_WIFI_CMD_SCAN_REQ) {
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received Scan Req ACK, waiting for Scan Event from Sensor");
            // We will not unblock main thread untill we received last_in_series
            // message or time out happens unless sensor send ack as failure.
            // We won't continue the APP after successful ACK until we get the response too!
            if (ack.status() != QSH_WIFI_ERROR_NONE)  //If error occurs, only then unblock
              qsh_wifi_unblock_th();
          } else {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "Recieved ACK for UNKNOWN Request");
          }
        }
    } else if (QSH_WIFI_MSGID_QSH_WIFI_EVT_SCAN_RESULTS == pb_event.msg_id()) {
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received QSH_WIFI_MSGID_QSH_WIFI_EVT_SCAN_RESULTS");

      qsh_wifi_evt_scan_result *scan_evt_ptr;
      uint32_t tmp_cnt = 0;
      bool is_monitor_results = false;

      pthread_mutex_lock(&scan_mutex);
      waiting_for_scan_result = is_scan_requested || is_monitor_on;
      pthread_mutex_unlock(&scan_mutex);
      if (waiting_for_scan_result == false)
      {
        __android_log_print(ANDROID_LOG_VERBOSE, TAG,"Ignore SCAN Event, Not waiting for monitor/demand scan result");
        continue;
      }

      // If flow reaches here it means we are waiting for scan
      // result surely (either demand scan or monitor scan).
      pthread_mutex_lock(&scan_mutex);
      if (is_scan_requested)   {
        scan_evt_ptr = &g_scan_evt_list[g_scan_evt_list_count++];
        tmp_cnt = g_scan_evt_list_count;
      } else {
        scan_evt_ptr = &g_mon_evt_list[g_mon_evt_list_count++];
        tmp_cnt = g_mon_evt_list_count;
      }
      pthread_mutex_unlock(&scan_mutex);

      if (tmp_cnt >= MAX_SCAN_EVENTS)
      {
        __android_log_print(ANDROID_LOG_ERROR, TAG,"Recevied more than 20 max Scan events ie more than 200 AP's");
        continue;    //skip the next steps for this case and go to next iteration
      }

      //Decode scan event
      scan_evt_ptr->ParseFromString(pb_event.payload());

      //Print the scan results, if the last scan event is received
      if(scan_evt_ptr->last_in_series())
      {
        // Transfer data to tmp list so printing can be mutex free
        qsh_wifi_evt_scan_result tmp_list [MAX_SCAN_EVENTS];
        uint32_t evt_cnt = 0;
        for(int i = 0; i < MAX_SCAN_EVENTS; i++)
          tmp_list[i].Clear();

        pthread_mutex_lock(&scan_mutex);
        if (is_scan_requested)   {
          for(int i = 0; i < MAX_SCAN_EVENTS; i++) {
            tmp_list[i] = g_scan_evt_list[i];
            g_scan_evt_list[i].Clear();
          }
          evt_cnt = g_scan_evt_list_count;
          g_scan_evt_list_count = 0;
          is_monitor_results = false;
        } else {
          for(int i = 0 ; i < MAX_SCAN_EVENTS; i++) {
            tmp_list[i] = g_mon_evt_list[i];
            g_mon_evt_list[i].Clear();
          }
          evt_cnt = g_mon_evt_list_count;
          g_mon_evt_list_count = 0;
          is_monitor_results = true;
        }
        pthread_mutex_unlock(&scan_mutex);

        // Dump result on console for non monitor mode
        print_scan_events(tmp_list, evt_cnt,!is_monitor_results);
        // Received last scan result message from Sesnor
        // hence unblock the main thread of client so
        // it can issue the next request.
        qsh_wifi_unblock_th();
      }
    } else if (QSH_WIFI_MSGID_QSH_WIFI_EVT_RANGING_RESULTS == pb_event.msg_id()) {
      __android_log_print(ANDROID_LOG_DEBUG, TAG,"Received QSH_WIFI_MSGID_QSH_WIFI_EVT_RANGING_RESULTS");
      pthread_mutex_lock(&rtt_mutex);
      if (is_rtt_requested == false)
      {
        pthread_mutex_unlock(&rtt_mutex);
        __android_log_print(ANDROID_LOG_VERBOSE, TAG,"Ignore RANGING Event, Not waiting for RANGING result");
        continue;    //Skip the rest of the code to next iteration, this result is ignored
      }
      pthread_mutex_unlock(&rtt_mutex);

      qsh_wifi_evt_ranging_result *ranging_evt_ptr;
      uint32_t tmp_cnt = 0;

      // If flow reaches here it means we are waiting for RTT results
      pthread_mutex_lock(&rtt_mutex);
      if (is_rtt_requested) {
        ranging_evt_ptr = &g_rtt_evt_list[g_rtt_evt_list_count++];
        tmp_cnt = g_rtt_evt_list_count;
      }
      pthread_mutex_unlock(&rtt_mutex);

      if (tmp_cnt >= MAX_RTT_EVENTS)
      {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "Recevied more than 5 max RTT events ie more than 50 AP's");
        continue;    //Skip the rest of the handling of event, go to next iteration
      }

      //decode ranging event
      ranging_evt_ptr->ParseFromString(pb_event.payload());

      //If the event is last in RTT series, we print it on the console
      if (ranging_evt_ptr->last_in_series())
      {
        // Transfer data to tmp list so printing can be mutex free
        qsh_wifi_evt_ranging_result tmp_rtt_list [MAX_RTT_EVENTS];
        uint32_t evt_cnt = 0;

        pthread_mutex_lock(&rtt_mutex);
        if (is_rtt_requested)
        {
          for(int i = 0 ; i < MAX_RTT_EVENTS ; i++) {
            tmp_rtt_list[i] = g_rtt_evt_list[i];
            g_rtt_evt_list[i].Clear();
          }
          evt_cnt = g_rtt_evt_list_count;
          g_rtt_evt_list_count = 0;
          is_rtt_requested = false;
        }
        pthread_mutex_unlock(&rtt_mutex);

        // Dump result on console ranging results
        print_ranging_events(tmp_rtt_list, evt_cnt, true);

        // Received last RTT scan result message from Sesnor
        // hence unblock the main thread of client so
        // it can issue the next request.
        qsh_wifi_unblock_th();
      }
    } else {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "Received unknown message ID %i", pb_event.msg_id());
    }
  }
}

/**
 * SUID callback as registered with suid_lookup.
 * Working of suid_lookup:
 * 1.suid_lookup object has two callback functions(_cb and _event_cb)
 * 2.suid_lookup constructor does the following:
 *   -> Creates it's own SSC_QMI connection,i.e _conn, to the scc_interface
 *   -> _conn is connected to which specific sensor(Any general lookup sensor)
 *         *_suid.low = 12370169555311111083ull;
 *         *_suid.high = 12370169555311111083ull;
 *    -> sets handle_ssc_event callback function on that connection,_event_cb
 *    -> sets "qsh_wifi_suid_cb" as _cb
 * 3.suid_lookup::request_suid(SENSOR_NAME) :
 *    -> Prepares and Sends get SUIDs Request on _conn for SENSOR_NAME
 *    -> You don't need to do "Send a request to the SUID Lookup Sensor for the complete list of SUIDs"
 *    -> It's a request for list of SUIDs related to this SENSOR_NAME only.
 * 4.Response from QMI triggers the callback function, handle_ssc_event
 *    -> An event will have datatype(SENSOR_NAME) and list of SUIDs for that datatype
 *    -> handle_ssc_event will decode this event and call _cb(qsh_wifi_suid_cb)
 *    -> The parameter of _cb will be datatype(SENSOR_NAME) and a list of all
 *       SUIDs found for that SENSOR_NAME. Store this SUID in some global var.
 */
static void qsh_wifi_suid_cb(const std::string& datatype, const std::vector<sensor_uid>& suids) {
  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Received SUID event with length(number of SUIDs) = %zu", suids.size());

  if(datatype.compare(SENSOR_NAME) < 0)    //Only for wifi_svc is accepted
    return;

  if (suids.size() > 0) {
    sensor_uid suid = suids.at(0);

    qsh_wifi_pr_high("Creating client connection");

    qsh_conn_config conn_config;
    //QSH_QMI : for QMI as transport layer
    connection = qsh_interface::create(QSH_QMI, conn_config);
    if (nullptr != connection) {
      //Response callback is NOT registered(nullptr)
      //Only Error callback and Event callback is registered in callback table
      //TBD: What's the difference between response callback and event callback?
      connection->register_cb(suids[0], nullptr, qsh_wifi_error_cb, qsh_wifi_event_cb);

      __android_log_print(ANDROID_LOG_WARN, TAG,"Received SUID %lu %lu for '%s'",
                suid.high, suid.low, datatype.c_str());

      //store suid in a global variable
      wifi_suid.low = suid.low;
      wifi_suid.high = suid.high;

      //TBD: do you need to do send_attr_req(qsh-location-test)
      //TBD: OR, send_config_req(sns-client-example-cpp)
      //send_attr_req(connection, suid);
    } else {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "connection is nullptr");
    }
  } else {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "%s sensor is not available", SENSOR_NAME);
    std::cout << "\n" << SENSOR_NAME << "sensor NOT AVAIALBE";
    exit(-1);
  }

  qsh_wifi_unblock_th();
}

static int parse_int_element(xmlNode *pXmlNode, int *value,
                             const xmlChar * const xmlString)
{
  int retVal = -1;
  const char *payloadData = (const char *)xmlNodeGetContent(pXmlNode);
  do
  {
    if (NULL == payloadData)
    {
      __android_log_print(ANDROID_LOG_WARN, TAG, "%s:%s No Payload\n", __FUNCTION__, xmlString);
      break;
    }
    __android_log_print(ANDROID_LOG_WARN, TAG, "%s:%s Payload: %s\n", __FUNCTION__, xmlString, payloadData);
    int status = sscanf(payloadData, "%d", value);
    if (status < 1)
    {
      __android_log_print(ANDROID_LOG_WARN, TAG, "%s: Element %s not formed correctly. Value = %d\n",
                      __FUNCTION__, xmlString, *value);
      break;
    }
    retVal = 0;
  }
  while (0);
  return retVal;
}


static void parse_scan_elements(xmlNode *a_node,
    qsh_wifi_cmd_scan_req *cmd)
{
  xmlNode *cur_node = NULL;
  for (cur_node = a_node; cur_node; cur_node = cur_node->next)
  {
    if (cur_node->type == XML_ELEMENT_NODE)
    {
      __android_log_print(ANDROID_LOG_WARN, TAG, "parse_elements: Element, name: %s\n", cur_node->name);
      if (xmlStrncmp (cur_node->name, XML_NODE_SCAN_TYPE,
                               xmlStrlen(XML_NODE_SCAN_TYPE)) == 0)
      {
        int scan_type;
        int ret = parse_int_element(cur_node, &scan_type, XML_NODE_SCAN_TYPE);
        if (ret == 0)
        {
          cmd->set_scan_type(static_cast<qsh_wifi_scan_type>(scan_type));
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_SCAN_AGE_MS,
                               xmlStrlen(XML_NODE_SCAN_AGE_MS)) == 0)
      {
        int age_ms;
        int status = parse_int_element(cur_node, &age_ms, XML_NODE_SCAN_AGE_MS);
        if (status == 0)
        {
          cmd->set_max_scan_age_ms(age_ms);
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_BAND,
                               xmlStrlen(XML_NODE_BAND)) == 0)
      {
        int band;
        int status = parse_int_element(cur_node, &band, XML_NODE_BAND);
        if (status == 0)
          cmd->set_band(band);
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_RADIO_CHAIN,
                               xmlStrlen(XML_NODE_RADIO_CHAIN)) == 0)
      {
        int radio;
        int status = parse_int_element(cur_node, &radio, XML_NODE_RADIO_CHAIN);
        if (status == 0)
        {
          cmd->set_radio_chain_pref(static_cast<qsh_wifi_pref_radio_chain>(radio));
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_FREQS,
                               xmlStrlen(XML_NODE_FREQS)) == 0)
      {
        const char *payloadData = (const char *)xmlNodeGetContent(cur_node);
        uint32_t freq = 0;
        if (payloadData)
        {
          __android_log_print(ANDROID_LOG_WARN, TAG, "parse_elements: Element len: %zu value: %s\n",
              strlen(payloadData), payloadData);
          char * data = (char *)malloc(strlen(payloadData)+1);
          if (data)
          {
            strlcpy(data, payloadData, (size_t)strlen(payloadData)+1);
            char *ctx = NULL;
            char *word = strtok_r(data, ",", &ctx);
            while( word != NULL )
            {
              freq = atoi(word);
              if (freq && (cmd->freq_list_size() < MAX_FREQ_IN_SCAN_EVT))
              {
                cmd->add_freq_list(freq);
              }
              __android_log_print(ANDROID_LOG_WARN, TAG, " %s\n word len %zu freq:%u", word, strlen(word), freq);
              word = strtok_r(NULL, ",", &ctx);
            }
            free(data);
          }
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_SSIDS,
                               xmlStrlen(XML_NODE_SSIDS)) == 0)
      {
        const char *payloadData = (const char *)xmlNodeGetContent(cur_node);
        if (payloadData)
        {
          __android_log_print(ANDROID_LOG_WARN, TAG, "parse_elements: Element len: %zu value: %s\n",
              strlen(payloadData), payloadData);
          char * data = (char *)malloc(strlen(payloadData)+1);
          if (data)
          {
            strlcpy(data, (char *)payloadData, (size_t)strlen(payloadData)+1);
            __android_log_print(ANDROID_LOG_WARN, TAG, "**** Data len: %zu value: %s\n", strlen(data), data);
            char *ctx = NULL;
            char *word = strtok_r(data, ",", &ctx);
            while( word != NULL )
            {
              if ((cmd->ssid_list_size() < MAX_SSID_CNT) &&
                  strlen(word) < (MAX_SSID_SIZE+1))
              {
                cmd->add_ssid_list(word, strlen(word)+1);
              }
              __android_log_print(ANDROID_LOG_WARN, TAG, "ssid %s\n len %zu", word, strlen(word));
              word = strtok_r(NULL, ",", &ctx);
            }
            free(data);
          }
        }
      }
    }
    parse_scan_elements(cur_node->children, cmd);
  }
}

/**
 * Below is the template of the SCAN Schema please put it as /data/scan.xml
 * NOTE: Heres freq's/SSID's are comma seperated values.
 *       SPACE in SSID will be treated as part of the name hence ensure
 *       No extra space are there in name.
 *
 * <scan>
 *  <scan_type>1</scan_type>
 *  <age_in_ms>1000</age_in_ms>
 *  <band>3</band>
 *  <radio_chain_pref>3</radio_chain_pref>
 *  <freqs>2412,2417,2422,2427,2432,2437,2442,2447,
 *   2452,2457,2462,2467,2472,2484
 *  </freqs>
 *  <ssids>HYDRA,Pandora,Guest</ssids>
 * </scan>
*/
static int parse_scan_xml(const char * const scan_xml,
    qsh_wifi_cmd_scan_req *cmd)
{
  // Open the xml file
  FILE *xmlFile = fopen(scan_xml, "rb");

  if (xmlFile == NULL)
  {
    fprintf(stderr, "%s:%d: Error opening file %s: %s\n",
            __func__,__LINE__, scan_xml, strerror(errno));
    return -1;
  }
  else
  {
    // File opened. Read it
    fseek(xmlFile, 0, SEEK_END);
    int xmlSize = (int)ftell(xmlFile);
    fseek(xmlFile, 0, SEEK_SET);
    char* buffer = new char[xmlSize];
    if (NULL == buffer)
    {
      fprintf(stderr, "Unable to allocate memory for provided xml size = %x\n",
              xmlSize);
      return -2;
    }
    fread(buffer, xmlSize, 1, xmlFile);
    fclose(xmlFile);

    xmlDoc *doc = xmlParseMemory(buffer, xmlSize);
    if (!doc)
    {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "cannot parse the xml file");
      return -3;
    }
    xmlNode *root_element = xmlDocGetRootElement(doc);
    if (!root_element)
    {
      __android_log_print(ANDROID_LOG_ERROR, TAG, "cannot find the root element scan xml file");
      return -4;
    }

    //Ensure Parent node is <scan> node only
    if ((root_element->type == XML_ELEMENT_NODE) &&
         (xmlStrncmp (root_element->name, XML_NODE_SCAN,
                               xmlStrlen(XML_NODE_SCAN)) == 0))
    {
      parse_scan_elements(root_element, cmd);
    }
    /*free the document */
    xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    xmlCleanupParser();

    // Free the buffer
    if (buffer) {
      delete[] buffer;
      buffer = nullptr;
    };
  }

  return 0;
}

static void parse_rtt_elements(xmlNode *a_node,
    qsh_wifi_cmd_ranging_req *cmd)
{
  UNUSED_VAR(cmd);
  xmlNode *cur_node = NULL;
  qsh_wifi_ranging_req_ap *ap = nullptr;

  if(cmd == NULL)
  {
    std::cout << "\n\tcmd is NULL";
    return;
  }

  if (cmd->bss_list_size())
    ap = cmd->mutable_bss_list(cmd->bss_list_size() - 1);

  for (cur_node = a_node; cur_node; cur_node = cur_node->next)
  {
    //Ensure current node is <ap> node only
    if (cur_node->type == XML_ELEMENT_NODE)
    {
      __android_log_print(ANDROID_LOG_WARN, TAG, "parse_elements: Element, name: %s\n", cur_node->name);
      if(xmlStrncmp (cur_node->name, XML_NODE_AP,
                               xmlStrlen(XML_NODE_AP)) == 0)
      {
        if (cmd->bss_list_size() >= MAX_REQ_RANGING_BSS)
        {
          __android_log_print(ANDROID_LOG_WARN, TAG, "Max Allowed BSS reached");
          break;
        }
        cmd->add_bss_list();
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_FREQ,
                               xmlStrlen(XML_NODE_FREQ)) == 0)
      {
        int freq;
        int ret = parse_int_element(cur_node, &freq, XML_NODE_FREQ);
        if (ret == 0 && ap)
          ap->set_freq(freq);
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_NUM_FRAMES,
                               xmlStrlen(XML_NODE_NUM_FRAMES)) == 0)
      {
        int frames = qsh_wifi_ranging_req_ap::default_instance().num_frames_per_burst();
        int ret = parse_int_element(cur_node, &frames, XML_NODE_NUM_FRAMES);
        if (ret == 0 && ap)
          ap->set_num_frames_per_burst(frames);
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_CF0,
                               xmlStrlen(XML_NODE_CF0)) == 0)
      {
        int freq0;
        int ret = parse_int_element(cur_node, &freq0, XML_NODE_CF0);
        if (ret == 0 && ap)
        {
          ap->set_cent_freq0(freq0);
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_CF1,
                               xmlStrlen(XML_NODE_CF1)) == 0)
      {
        int freq1;
        int ret = parse_int_element(cur_node, &freq1, XML_NODE_CF1);
        if (ret == 0 && ap)
        {
          ap->set_cent_freq1(freq1);
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_CW,
                               xmlStrlen(XML_NODE_CW)) == 0)
      {
        int cw;
        int ret = parse_int_element(cur_node, &cw, XML_NODE_CW);
        if (ret == 0 && ap)
        {
          ap->set_channel_width(static_cast<qsh_wifi_channel_width>(cw));
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_LCI,
                               xmlStrlen(XML_NODE_LCI)) == 0)
      {
        int lci;
        int ret = parse_int_element(cur_node, &lci, XML_NODE_LCI);
        if (ret == 0 && ap)
        {
          ap->set_request_lci(lci ? true : false);
        }
      }
      else if (xmlStrncmp (cur_node->name, XML_NODE_MAC,
                               xmlStrlen(XML_NODE_MAC)) == 0)
      {
        const char *payloadData = (const char *)xmlNodeGetContent(cur_node);
        if (payloadData)
        {
          __android_log_print(ANDROID_LOG_WARN, TAG, "parse_elements: Element len: %zu value: %s\n",
              strlen(payloadData), payloadData);
          uint8_t bssid[6];
          memset(bssid, 0, sizeof(bssid));
          int status = sscanf(payloadData, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                              &bssid[0], &bssid[1], &bssid[2],
                              &bssid[3], &bssid[4], &bssid[5]);
          if (status < 6)
          {
            // Either the element does not contain the content or it is
            // not formatted correctly. Set all the entries to 0
            __android_log_print(ANDROID_LOG_WARN, TAG, "mac adress is not formatted correctly");
          }
          else
          {
            if (ap)
            {
              ap->set_mac_addr((const char *)bssid);
            }
          }
        }
      }
    }
    parse_rtt_elements(cur_node->children, cmd);
  }
}


/**
 * Below is the template of the RTT Schema
 * please put it as /data/rtt.xml.
 * <rtt>
 *   <ap>
 *    <mac>aa:bb:cc:dd:ee:ff</mac>
 *    <ch_width>2</ch_width>
 *    <num_frames>8</num_frames>
 *    <freq>5745</freq>
 *    <cent_freq0>5775</cent_freq0>
 *    <cent_freq1>0</cent_freq1>
 *    <lci>1</lci>
 *   </ap>
 *   <ap>
 *    <mac>11:22:33:44:55:66</mac>
 *    <ch_width>3</ch_width>
 *    <num_frames>8</num_frames>
 *    <freq>5500</freq>
 *    <cent_freq0>5530</cent_freq0>
 *    <cent_freq1>5610</cent_freq1>
 *    <lci>1</lci>
 *   </ap>
 * </rtt>
 */
int parse_rtt_xml(const char * const rtt_xml,
    qsh_wifi_cmd_ranging_req *cmd )
{
  // Open the xml file
  FILE *xmlFile = fopen(rtt_xml, "rb");

  if (xmlFile == NULL)
  {
    fprintf(stderr, "%s:%d: Error opening file %s: %s\n",
            __func__,__LINE__, rtt_xml, strerror(errno));
    std::cout << "\nError opening XML file";
    return -1;
  }
  else
  {
    // File opened. Read it
    fseek(xmlFile, 0, SEEK_END);
    int xmlSize = (int)ftell(xmlFile);
    fseek(xmlFile, 0, SEEK_SET);
    __android_log_print(ANDROID_LOG_WARN, TAG, "rtt xml file size %d", xmlSize);
    char* buffer = new char[xmlSize];
    if (NULL == buffer)
    {
      fprintf(stderr, "Unable to allocate memory for provided xml size = %x\n",
              xmlSize);
      return -2;
    }
    fread(buffer, xmlSize, 1, xmlFile);
    fclose(xmlFile);

    xmlDoc *doc = xmlParseMemory(buffer, xmlSize);
    if (!doc)
    {
      __android_log_print(ANDROID_LOG_WARN, TAG, "cannot parse the xml file");
      return -3;
    }
    xmlNode *root_element = xmlDocGetRootElement(doc);
    if (!root_element)
    {
      __android_log_print(ANDROID_LOG_WARN, TAG, "cannot find the root element rtt xml file");
      return -4;
    }

    //Ensure Parent node is <rtt> node only
    if ((root_element->type == XML_ELEMENT_NODE) &&
         (xmlStrncmp (root_element->name, XML_NODE_RTT,
                               xmlStrlen(XML_NODE_RTT)) == 0))
    {
      parse_rtt_elements(root_element, cmd);
    }
    /*free the document */
    xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    xmlCleanupParser();

    // Free the buffer
    if (buffer) {
      delete[] buffer;
      buffer  = nullptr;
    }
  }

  return 0;
}

int qsh_wifi_init ()
{
  int ret = -1;
  // Before open intilization is essential.
  if (state == INIT) {
    qsh_wifi_pr_high("Client is already connected to Sensor");
    return 0;
  }
  pthread_mutex_init(&mutex_m, NULL);
  pthread_cond_init(&cond, NULL);
  pthread_mutex_init(&scan_mutex, NULL);
  pthread_mutex_init(&rtt_mutex, NULL);

  qsh_wifi_pr_high("Starting QSH WIFI TEST");

  //TBD: what's alternative in C++ to wait-for-service
  //wait-for-service is REMOVED in qsh-location-test as well as sns-client-example-cpp
  //if(!qsh_wifi_wait_for_service(200))
  //  return -1;

  suid_lookup lookup(qsh_wifi_suid_cb);
  lookup.request_suid(SENSOR_NAME);

  std::cout << "\nWaiting for SUID callback";
  ret = qsh_wifi_timeout_block(CMD_TIMEOUT_IN_MS, &cond);
  if (ret != 0)
  {
    std::cout << "\nError occured while waiting for SUID callback, ret: "<< ret;
    return -4;
  }
  __android_log_print(ANDROID_LOG_VERBOSE, TAG, "suid lookup done => wifi_suid(in hex) = %lx %lx", wifi_suid.high, wifi_suid.low);
  std::cout << "\nSUID for " << SENSOR_NAME << " : 0x";
  std::cout << std::hex << wifi_suid.high << std::hex << wifi_suid.low;
  state = INIT;

  pthread_mutex_lock(&scan_mutex);
  for(int i = 0; i < MAX_SCAN_EVENTS; i++)
    g_scan_evt_list[i].Clear();
  g_scan_evt_list_count = 0;
  is_scan_requested = false;

  for(int i = 0; i < MAX_SCAN_EVENTS; i++)
    g_mon_evt_list[i].Clear();
  g_mon_evt_list_count = 0;
  is_monitor_on= false;
  pthread_mutex_unlock(&scan_mutex);

  pthread_mutex_lock(&rtt_mutex);
  for(int i = 0; i < MAX_RTT_EVENTS; i++)
    g_rtt_evt_list[i].Clear();
  g_rtt_evt_list_count = 0;
  is_rtt_requested= false;
  pthread_mutex_unlock(&rtt_mutex);

  return 0;
}

int qsh_wifi_deinit()
{
  std::cout << "\nExiting qsh wifi example state " << state << endl;
  if (state == DEINIT)
    return -1;

  delete connection;
  connection = nullptr;

  pthread_mutex_destroy(&mutex_m);
  pthread_cond_destroy(&cond);
  pthread_mutex_destroy(&scan_mutex);
  pthread_mutex_destroy(&rtt_mutex);

  state = DEINIT;
  return 0;
}

void qsh_wifi_signal_handler(int signal_id)
{
  std::cout << "\nReceived signal: "<< signal_id << " console interrupted";
  qsh_wifi_deinit();
  state = DEINIT;
  std::cout << "\n *** Terminated qsh_wifi_test successfully *** \n";
  exit(0);
}

static void dump_scan_req_params(const qsh_wifi_cmd_scan_req &scan_req)
{
  __android_log_print(ANDROID_LOG_DEBUG, TAG,
      "band: %u scan_type: %u has_age: %d age_ms: %u"
      " freq_cnt(list size): %u ssid_cnt(list size): %u",
      scan_req.band(), scan_req.scan_type(), scan_req.has_max_scan_age_ms(),
      scan_req.max_scan_age_ms(), scan_req.freq_list_size(), scan_req.ssid_list_size());

  for (int cnt = 0; cnt < scan_req.freq_list_size() && cnt < MAX_SCAN_EVENTS; cnt++)
  {
    __android_log_print(ANDROID_LOG_DEBUG, TAG,"\tfreq  %u", scan_req.freq_list(cnt));
  }

  for (int cnt = 0; cnt  < scan_req.ssid_list_size(); cnt++)
  {
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "ssid len %u %-32s", scan_req.ssid_list(cnt).length(),
      scan_req.ssid_list(cnt).c_str());
  }
}

static void dump_rtt_req_params(qsh_wifi_cmd_ranging_req *rtt_db)
{
  __android_log_print(ANDROID_LOG_DEBUG, TAG, "Total AP in RTT req: %u",
      rtt_db->bss_list_size());

  for (int cnt = 0; cnt < rtt_db->bss_list_size() && cnt < MAX_REQ_RANGING_BSS; cnt++)
  {
    const qsh_wifi_ranging_req_ap &ap = rtt_db->bss_list(cnt);
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "AP bssid: %02x:%02x:%02x:%02x:%02x:%02x",
         static_cast<int>(ap.mac_addr()[0]),static_cast<int>(ap.mac_addr()[1]),
         static_cast<int>(ap.mac_addr()[2]),static_cast<int>(ap.mac_addr()[3]),
         static_cast<int>(ap.mac_addr()[4]),static_cast<int>(ap.mac_addr()[5]));
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "freq:%u BW:%u CF0:%u CF1:%u lci:%d num_frames:%u",
        ap.freq(), ap.channel_width(), ap.cent_freq0(), ap.cent_freq1(),
        ap.request_lci(), ap.num_frames_per_burst());
  }
}

/**
 * Handle only cmd line interrupts
 * to send close to Sensor.
 * This helps to get Abort call stack
 *
 * Ctrl+C - SIGINT
 * Ctrl+\ - SIGQUIT
 * Ctrl+Z - SIGTSTP
 */
void register_signal()
{
  // Register signal handler
  signal(SIGINT, qsh_wifi_signal_handler);
  signal(SIGQUIT, qsh_wifi_signal_handler);
  signal(SIGTSTP, qsh_wifi_signal_handler);
}

qsh_wifi_cmd_scan_req g_scan_cmd;
qsh_wifi_cmd_ranging_req g_rtt_cmd;

int main(int argc,char *argv[])
{
  UNUSED_VAR(argc);
  UNUSED_VAR(argv);

  int ret = -1;
  int loop = 1;

  register_signal();
  pid = getpid();

  std::cout << "\n****** QSH WIFI TEST *******";
  std::cout << "\npid = " << pid << ", threadid = " << gettid();
  std::cout << "\nUsage: Wifi test";
  std::cout << "\n[1 -> Open]\n[2 -> monitor on]";
  std::cout << "\n[3 -> monitor off]\n[4 -> on demand scan]";
  std::cout << "\n[5 -> on demand with scan params]\n[6 -> rtt start]";
  std::cout << "\n[exit -> 9]";

  while(loop)
  {
    int cmd = -1;
    long delay = -1;
    long iter = -1;
    std::cout << "\n\nEnter command : ";
    std::cin >> cmd;
    switch(cmd)
    {
      case OPEN:
        std::cout << "\nOpening QSH Wifi Driver Interface";
        if (state == OPENED)
        {
          std::cout << "\nAlready opened";
        }
        else
        {
          // Initialize the QMI stuff with WIFI Sensor
          ret = qsh_wifi_init();
          if(ret != 0)
          {
            std::cout << "\nError occurred in initialization : " << ret << " Hence Stopping it";
            ret = 0;
            loop = 0;
          }
          ret = qsh_wifi_send_open_req(connection, wifi_suid);
          if (ret != 0)
          {
            std::cout << "\nOpen failed move to deinit state";
            qsh_wifi_deinit();
          }
          else
            state = OPENED;
        }
        break;

      case MONITOR_MODE_ON:
        std::cout << "\nSending Montior Mode ON request";
        ret = qsh_wifi_send_monitor_mode_req(connection,
                  wifi_suid, true);
        break;

      case MONITOR_MODE_OFF:
        std::cout << "\nSending Monitor Mode OFF request";
        ret = qsh_wifi_send_monitor_mode_req (connection,
                  wifi_suid, false);
        break;

      case ON_DEMAND_SCAN:
        delay = 1000*1000;   //one msec
        iter = 1;

        std::cout << "\nScan Start";
        std::cout << "\nEnter number of iteration: (Default is 1 iteration): ";
        std::cin >> iter;

        if(iter > 1) {
          std::cout << "\nDelay between iteration in msec: (Ex: 1000): ";
          std::cin >> delay;
          /* convert in msec  */
          delay = ((delay <= 0) ? 1000*1000 : delay * 1000);
        } else if(iter <= 0) {
          std::cout << "\nEntered zero iteration hence skipping sending ON DEMAND SCAN request";
          break;
        }

        for (int i =0; i < iter; i++)
        {
          std::cout << "\niteration number : " << (i+1);
          g_scan_cmd.Clear();
          g_scan_cmd.set_band(QSH_WIFI_BAND_MASK_2P4_GHZ |
                               QSH_WIFI_BAND_MASK_5_GHZ);
          g_scan_cmd.set_scan_type(QSH_WIFI_SCAN_TYPE_ACTIVE);
          g_scan_cmd.set_radio_chain_pref(QSH_WIFI_PREF_RADIO_CHAIN_HIGH_ACCURACY);
          g_scan_cmd.set_max_scan_age_ms(4500);
          std::cout <<"\nSend On Demand Scan Request";
          dump_scan_req_params(g_scan_cmd);
          ret = qsh_wifi_send_on_demand_scan (connection,
                   wifi_suid, g_scan_cmd);
          is_scan_requested = false;
          usleep(delay);
        }
        break;

      case ON_DEMAND_SCAN_USER_PARAMS:
        g_scan_cmd.Clear();
        ret = parse_scan_xml(SCAN_FILE_PATH, &g_scan_cmd);
        if (ret == 0)
        {
          std::cout << "\nSend On Demand Scan from " << SCAN_FILE_PATH;
          dump_scan_req_params(g_scan_cmd);
          ret = qsh_wifi_send_on_demand_scan (connection,
                    wifi_suid, g_scan_cmd);
        }
        is_scan_requested = false;
        break;

      case RTT_SCAN:
        g_rtt_cmd.Clear();
        ret = parse_rtt_xml(RTT_FILE_PATH, &g_rtt_cmd);
        if (ret != 0)
        {
          std::cout << "\nNot able to parse rtt xml file, ret : " << ret;
          break;
        }
        if (g_rtt_cmd.bss_list_size() == 0)
        {
          std::cout << "\nNo valid BSS found in input xml file";
          break;
        }

        delay = 1000*1000;   //one msec
        iter = 1;

        std::cout << "\nStart Ranging";
        std::cout << "\nEnter number of iteration: (Default is 1 iteration): ";
        std::cin >> iter;

        if(iter > 1) {
          std::cout << "\nDelay between iteration in msec: (Ex: 1000): ";
          std::cin >> delay;
          /* convert in msec  */
          delay = ((delay <= 0) ? 1000*1000 : delay * 1000);
        } else if(iter <= 0) {
          std::cout << "\nEntered zero iteration hence skipping sending RTT SCAN request";
          break;
        }

        dump_rtt_req_params(&g_rtt_cmd);
        for (int i =0; i < iter; i++)
        {
          std::cout << "\niteration number : " << (i+1);
          ret = qsh_wifi_send_rtt_req (connection,
                   wifi_suid, g_rtt_cmd);
          is_rtt_requested = false;
          usleep(delay);
        }
        break;

      case DUMP_EVENT:
        std::cout << "\nSend Dump event request";
        ret = qsh_wifi_send_dump_event(connection, wifi_suid);
        break;

      case EXIT:
        std::cout << "\nTerminate QSH-WIFI-TEST app";
        ret = 0;
        loop = 0;
        break;

      default:
        std::cout << "\nPlease enter correct CMD!";
    }
  }

  ret = qsh_wifi_deinit();
  return ret;
}
