/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#pragma once
#include "qsh_wifi_interface_reference.h"
#include "qmi_client.h"
#include "sns_client_api_v01.h"
#include <mutex>
#include <map>
#include <deque>
#include <chrono>
#include "qsh_wifi_register_cb_reference.h"
#include <unordered_map>
#include <vector>
struct cb_sensor_uid_hash
{
  std::size_t operator()(const sensor_uid& suid) const
  {
    std::string data(reinterpret_cast<const char*>(&suid), sizeof(sensor_uid));
    return std::hash<std::string>()(data);
  }
};

/* exception type defining errors related to qmi API calls */
struct qsh_qmi_error : public std::runtime_error
{
  qsh_qmi_error(int error_code, const std::string& what = "") :
        runtime_error(what + ": " + error_code_to_string(error_code)) { }

  static std::string error_code_to_string(int code)
  {
    static const std::map<int, std::string> error_map = {
      { QMI_NO_ERR, "qmi no error" },
      { QMI_INTERNAL_ERR, "qmi internal error" },
      { QMI_TIMEOUT_ERR, "qmi timeout" },
      { QMI_XPORT_BUSY_ERR, "qmi transport busy" },
      { QMI_SERVICE_ERR, "qmi service error" },
    };
    std::string msg;
    try {
      msg = error_map.at(code);
    } catch (std::out_of_range& e) {
      msg = "qmi error";
    }
    return msg + " (" + std::to_string(code) + ")";
  }
};

class qsh_qmi : public qsh_interface {
public:
  qsh_qmi();
  ~qsh_qmi();
  int register_cb(sensor_uid suid, qsh_resp_cb resp_cb, qsh_error_cb error_cb, qsh_event_cb event_cb);
  void send_request(sensor_uid suid, bool use_jumbo_request, std::string encoded_buffer);

private:
  void qmi_connect();
  void qmi_wait_for_service();
  void qmi_disconnect();
  void handle_resp_cb(sns_client_resp_msg_v01 resp);
  void handle_event_cb(unsigned int msg_id, void *ind_buf, unsigned int ind_buf_len);
  int decode_qmi_buffer(unsigned int msg_id, void *ind_buf, unsigned int ind_buf_len);
  void create_ind_memory();
  void destroy_ind_memory();
  bool is_disable_stream_request(std::string encoded_buffer);


  struct stream_req_info
  {
    sensor_uid suid;
    bool is_disable_req;
  };

  sns_client_jumbo_report_ind_msg_v01       *_ind = nullptr;
  qmi_cci_os_signal_type                    _os_params;
  sns_client_resp_msg_v01                   _resp = {};
  qmi_client_type                           _qmi_handle;

  bool                                      _service_ready;

  std::atomic<bool>                         _reconnecting = false;
  std::atomic<bool>                         _connection_closed = false;
  std::mutex                                _mutex;
  std::condition_variable                   _cv;
  std::mutex                                _cb_map_table_mutex;
  std::unordered_map<sensor_uid, qsh_register_cb, cb_sensor_uid_hash>     _callback_map_table;

  std::deque<stream_req_info>               _resp_queue;
  std::mutex                                _resp_queue_mutex;
  const int                                 _initial_discovery_tries_count = 4;
  const int                                 _post_discovery_tries_count = 2;

  /*All static methods and variables - below*/
  static void qmi_indication_cb(qmi_client_type user_handle,
                                unsigned int msg_id, void* ind_buf,
                                unsigned int ind_buf_len,
                                void* ind_cb_data);

  static void qmi_notify_cb(qmi_client_type user_handle,
                            qmi_idl_service_object_type service_obj,
                            qmi_client_notify_event_type service_event,
                            void *notify_cb_data);

  static void qmi_error_cb(qmi_client_type user_handle,
                           qmi_client_error_type error,
                           void* err_cb_data);

  static void qmi_response_cb(qmi_client_type user_handle,
                                unsigned int msg_id,
                                void* resp_cb,
                                unsigned int resp_cb_len,
                                void* resp_cb_data,
                                qmi_client_error_type qmi_error);

  static bool                               _service_accessed;
  static uint32_t                           _qmi_err_cnt;


};
