/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "qsh_wifi_qmi_reference.h"
#include "sns_client_api_v01.h"
#include "qsh_wifi_suid_util.h"
#include "sns_client.pb.h"
#include "utils/SystemClock.h"
using namespace std;

#define MAX_SVC_INFO_ARRAY_SIZE 5
static auto SENSORS_SERVICE_DISCOVERY_TIMEOUT = 2s;
static auto SENSORS_POST_DISCOVERY_TIMEOUT = 500;

bool qsh_qmi::_service_accessed = false;
uint32_t qsh_qmi::_qmi_err_cnt = 0;

qsh_qmi::qsh_qmi(){
  _qmi_handle = nullptr;
  create_ind_memory();
  qmi_connect();
  sns_logi("%s" , __func__);
}

void qsh_qmi::create_ind_memory() {
  _ind = (sns_client_jumbo_report_ind_msg_v01 *)calloc(1, sizeof(sns_client_jumbo_report_ind_msg_v01));
  if(nullptr == _ind){
    sns_loge("Error while creating memory for ind");
    return;
  }
}

void qsh_qmi::destroy_ind_memory(){
  if(nullptr != _ind) {
    free(_ind);
    _ind = nullptr;
  }
}

qsh_qmi::~qsh_qmi(){
  _connection_closed = true;
  qmi_disconnect();
  destroy_ind_memory();
  _callback_map_table.clear();
  _resp_queue.clear();
  sns_logi("%s" , __func__);
}

void qsh_qmi::qmi_connect()
{
  qmi_idl_service_object_type svc_obj =
      SNS_CLIENT_SVC_get_service_object_v01();

  qmi_client_error_type qmi_err;
  qmi_service_info svc_info_array[MAX_SVC_INFO_ARRAY_SIZE];
  uint32_t num_services, num_entries = MAX_SVC_INFO_ARRAY_SIZE;

  /*svc_info_array[5] - Initialized to 0 to avoid static analysis errors*/
  for(uint32_t i = 0 ; i < num_entries ; i++)
    memset(&svc_info_array[i], 0, sizeof(svc_info_array[i]));

  sns_logv("waiting for sensors qmi service");
  qmi_wait_for_service();
  sns_logv("connecting to qmi service");
  qmi_err = qmi_client_get_service_list(svc_obj, svc_info_array,
      &num_entries, &num_services);
  if (QMI_NO_ERR != qmi_err) {
    throw qsh_qmi_error(qmi_err, "qmi_client_get_service_list() failed");
  }

  if (num_entries == 0) {
    throw runtime_error("sensors service has no available instances");
  }

  if (_connection_closed) {
    sns_logi("connection got closed do not open qmi_channel");
    return ;
  }

  /* As only one qmi service is expected for sensors, use the 1st instance */
  qmi_service_info svc_info = svc_info_array[0];
  std::unique_lock<std::mutex> lk(_mutex);
  qmi_err = qmi_client_init(&svc_info, svc_obj, qmi_indication_cb,
      (void*)this, &_os_params, &_qmi_handle);
  if (qmi_err != QMI_IDL_LIB_NO_ERR) {
    lk.unlock();
    throw qsh_qmi_error(qmi_err, "qmi_client_init() failed");
  }

  qmi_err = qmi_client_register_error_cb(_qmi_handle, qmi_error_cb, this);
  if (QMI_NO_ERR != qmi_err) {
    lk.unlock();
    qmi_client_release(_qmi_handle);
    throw qsh_qmi_error(qmi_err, "qmi_client_register_error_cb() failed");
  }
  lk.unlock();
  sns_logv("connected to qsh for %p", (void *)this);
}

void qsh_qmi::qmi_wait_for_service()
{
  qmi_client_type notifier_handle;
  qmi_client_error_type qmi_err;
  qmi_cci_os_signal_type os_params;
  int num_tries = _initial_discovery_tries_count;
  qmi_err = qmi_client_notifier_init(SNS_CLIENT_SVC_get_service_object_v01(),
      &os_params, &notifier_handle);
  if (QMI_NO_ERR != qmi_err) {
    throw qsh_qmi_error(qmi_err, "qmi_client_notifier_init() failed");
  }

  /* register a callback and wait until service becomes available */
  _service_ready = false;
  qmi_err = qmi_client_register_notify_cb(notifier_handle, qmi_notify_cb,
      this);
  if (qmi_err != QMI_NO_ERR) {
    qmi_client_release(notifier_handle);
    throw qsh_qmi_error(qmi_err, "qmi_client_register_notify_cb() failed: %d");
  }

  if (_service_accessed) {
    num_tries = _post_discovery_tries_count;
  }

  std::unique_lock<std::mutex> lk(_mutex);
  bool timeout = false;
  while (num_tries > 0 && (_service_ready != true)) {
    num_tries--;
    if (_service_accessed)
      timeout = !_cv.wait_for(lk, std::chrono::milliseconds(SENSORS_POST_DISCOVERY_TIMEOUT),
          [this]{ return _service_ready; });
    else
      timeout = !_cv.wait_for(lk, std::chrono::seconds(SENSORS_SERVICE_DISCOVERY_TIMEOUT),
          [this]{ return _service_ready; });

    if (timeout) {
      if (num_tries == 0) {
        lk.unlock();
        qmi_client_release(notifier_handle);
      }
      sns_loge("timeout while waiting for sensors QMI service: "
          "will try %d more time(s)", num_tries);
    } else {
      _service_accessed = true;
    }
  }
  lk.unlock();
  qmi_client_release(notifier_handle);
}

void qsh_qmi::qmi_disconnect()
{
  std::unique_lock<std::mutex> lk(_mutex);
  if (_qmi_handle != nullptr) {
    qmi_client_release(_qmi_handle);
    _qmi_handle = nullptr;
    /*in ssr call back and sensor disabled , so notify qmi_connect to comeout*/
    if (_connection_closed)
      _cv.notify_one();
  }
  /*explicit unlock not required , just added not to miss the logic*/
  lk.unlock();
  sns_logv("disconnected from qsh for %p", (void *)this);
}

void qsh_qmi::qmi_indication_cb(qmi_client_type user_handle,
                                unsigned int msg_id, void* ind_buf,
                                unsigned int ind_buf_len,
                                void* ind_cb_data
                                )
{
  UNUSED(user_handle);
  qsh_qmi *conn = static_cast<qsh_qmi *>((qsh_interface *)ind_cb_data);
  conn->handle_event_cb(msg_id, ind_buf, ind_buf_len);
}

void qsh_qmi::qmi_error_cb(qmi_client_type user_handle,
                                      qmi_client_error_type error,
                                      void* err_cb_data)
{
  UNUSED(user_handle);
  qsh_qmi *conn = static_cast<qsh_qmi *>((qsh_interface *)err_cb_data);
  sns_loge("error=%d", error);

  for(auto itr = conn->_callback_map_table.begin(); itr!= conn->_callback_map_table.end(); ++itr) {
    itr->second.get_error_cb()(QSH_INTERFACE_RESET);
  }
}

void qsh_qmi::qmi_notify_cb(qmi_client_type user_handle,
                                       qmi_idl_service_object_type service_obj,
                                       qmi_client_notify_event_type service_event,
                                       void *notify_cb_data)
{
  UNUSED(user_handle);
  UNUSED(service_obj);
  UNUSED(service_event);
  qsh_qmi *conn = static_cast<qsh_qmi *>((qsh_interface *)notify_cb_data);
  unique_lock<mutex> lk(conn->_mutex);
  conn->_service_ready = true;
  conn->_cv.notify_one();
}

int qsh_qmi::register_cb(sensor_uid suid,   qsh_resp_cb resp_cb, qsh_error_cb error_cb, qsh_event_cb event_cb) {
  sns_logi("%s start this=%px" , __func__, this);

  std::lock_guard<mutex> lk(_cb_map_table_mutex);
  auto it = _callback_map_table.find(suid);
  if(it == _callback_map_table.end()) {
    qsh_register_cb cb(resp_cb,error_cb,event_cb);
    _callback_map_table.insert(std::pair<sensor_uid,qsh_register_cb>(suid, cb));
    sns_logi("%s End with succes" , __func__);
    return 0;
  } else {
    sns_loge("This suid is alread registered ");
    return -1;
  }
}

void qsh_qmi::send_request(sensor_uid suid, bool use_jumbo_request, std::string encoded_buffer) {
  sns_logi("%s start this=%px " , __func__, this);
  if (_reconnecting) {
    sns_loge("qmi connection failed, cannot send data");
    return;
  }
  sns_client_req_msg_v01 req_msg;

  if (encoded_buffer.length() > SNS_CLIENT_REQ_LEN_MAX_V01) {
    throw runtime_error("error: payload size too large");
  }

  memcpy(req_msg.payload, encoded_buffer.c_str(),
      encoded_buffer.length());
  req_msg.use_jumbo_report_valid = use_jumbo_request;
  req_msg.use_jumbo_report = use_jumbo_request;
  req_msg.payload_len = encoded_buffer.length();

  stream_req_info req_info;
  req_info.suid.low = suid.low;
  req_info.suid.high = suid.high;
  req_info.is_disable_req = is_disable_stream_request(encoded_buffer);
  _resp_queue_mutex.lock();
  _resp_queue.push_back(req_info);
  _resp_queue_mutex.unlock();
  qmi_client_error_type qmi_err;
  qmi_txn_handle qmi_txn_handle;
  qmi_err = qmi_client_send_msg_async(_qmi_handle, SNS_CLIENT_REQ_V01,
      (void*)&req_msg, sizeof(req_msg),
      &_resp,
      sizeof(sns_client_resp_msg_v01),
      qmi_response_cb,
      (void*)this,
      &qmi_txn_handle);

  if (qmi_err != QMI_NO_ERR){
    _resp_queue_mutex.lock();
    _resp_queue.pop_back();
    _resp_queue_mutex.unlock();
    _qmi_err_cnt++;
    throw qsh_qmi_error(qmi_err,
          "ssr triggered after qmi_client_send_msg_sync ");
  } else {
    /*occassional failure of QMI , recovered with in _max_qmi_error_cnt*/
    if (_qmi_err_cnt)
      _qmi_err_cnt = 0;
  }

  sns_logi("%s completed, this=%px" , __func__, this);
}

bool qsh_qmi::is_disable_stream_request(std::string encoded_buffer) {
  sns_client_request_msg pb_req_msg;
  pb_req_msg.ParseFromArray(encoded_buffer.c_str(), encoded_buffer.length());
  int msg_id = pb_req_msg.msg_id();
  if(SNS_CLIENT_MSGID_SNS_CLIENT_DISABLE_REQ == msg_id) {
    return true;
  }else {
    return false;
  }
}

void qsh_qmi::qmi_response_cb(qmi_client_type user_handle,
                              unsigned int msg_id,
                              void* resp_cb,
                              unsigned int resp_cb_len,
                              void* resp_cb_data,
                              qmi_client_error_type qmi_err)
{
  UNUSED(user_handle);
  UNUSED(msg_id);
  UNUSED(resp_cb_len);
  UNUSED(qmi_err);
  qsh_qmi *conn = static_cast<qsh_qmi *>((qsh_interface *)resp_cb_data);
  sns_client_resp_msg_v01 resp = *((sns_client_resp_msg_v01 *)resp_cb);
  conn->handle_resp_cb(resp);
}

void qsh_qmi::handle_resp_cb(sns_client_resp_msg_v01 resp) {
  if (_connection_closed) {
    sns_logi("qmi response is coming while connection is being closed");
    return;
  }
  _resp_queue_mutex.lock();

  if(false == _resp_queue.empty()) {
    auto front = _resp_queue.front();
    sensor_uid suid = front.suid;
    bool is_disable_resp = front.is_disable_req;
    _resp_queue.pop_front();
    _resp_queue_mutex.unlock();
    _cb_map_table_mutex.lock();

    auto it = _callback_map_table.find(suid);
    if(it == _callback_map_table.end()){

      _cb_map_table_mutex.unlock();

      return;
    }
    qsh_resp_cb current_resp_cb = it->second.get_resp_cb();
    if(true == is_disable_resp) {
        _callback_map_table.erase(it);
    }

    _cb_map_table_mutex.unlock();

    if(current_resp_cb && resp.result_valid && resp.client_id_valid) {

      current_resp_cb(resp.result, resp.client_id);
    }
  } else {

    _resp_queue_mutex.unlock();

  }

  sns_logi("%s Ended this=%px" , __func__, this);
}

int qsh_qmi::decode_qmi_buffer(unsigned int msg_id, void *ind_buf, unsigned int ind_buf_len) {
  int32_t qmi_err;
  if(SNS_CLIENT_REPORT_IND_V01 == msg_id){
    qmi_err = qmi_idl_message_decode(SNS_CLIENT_SVC_get_service_object_v01(),
        QMI_IDL_INDICATION, msg_id, ind_buf,
        ind_buf_len, (void*)_ind,
        sizeof(sns_client_report_ind_msg_v01));
    if (QMI_IDL_LIB_NO_ERR != qmi_err) {
      sns_loge("qmi_idl_message_decode() failed. qmi_err=%d SNS_CLIENT_REPORT_IND_V01", qmi_err);
      return -1;
    }
  }
  else if(SNS_CLIENT_JUMBO_REPORT_IND_V01 == msg_id){
    qmi_err = qmi_idl_message_decode(SNS_CLIENT_SVC_get_service_object_v01(),
        QMI_IDL_INDICATION, msg_id, ind_buf,
        ind_buf_len, (void*)_ind,
        sizeof(sns_client_jumbo_report_ind_msg_v01));
    if (QMI_IDL_LIB_NO_ERR != qmi_err) {
      sns_loge("qmi_idl_message_decode() failed. qmi_err=%d SNS_CLIENT_JUMBO_REPORT_IND_V01", qmi_err);
      return -1;
    }
  } else {
    sns_loge("not a valid qmi buffer ");
    return -1;
  }
  return 0;
}

void qsh_qmi::handle_event_cb(unsigned int msg_id, void *ind_buf, unsigned int ind_buf_len) {

  if (_connection_closed) {
    sns_logi("qmi indication is coming while connection is being closed");
    return;
  }
  uint64_t sample_received_ts = android::elapsedRealtimeNano();

  int ret = decode_qmi_buffer(msg_id, ind_buf, ind_buf_len);
  if(ret < 0) {
    sns_logi("error while decode_qmi_buffer ");
    return;
  }

  sns_client_event_msg pb_event_msg;
  pb_event_msg.ParseFromArray(_ind->payload, _ind->payload_len);
  if( false == pb_event_msg.has_suid()) {
    sns_logi("first level decoding suid failed");
    return;
  }
  sensor_uid suid;
  suid.low = pb_event_msg.suid().suid_low();
  suid.high = pb_event_msg.suid().suid_high();


  _cb_map_table_mutex.lock();

  auto it = _callback_map_table.find(suid);
  if(it == _callback_map_table.end()) {

    _cb_map_table_mutex.unlock();

    return;
  }
  qsh_event_cb current_event_cb = it->second.get_event_cb();

  _cb_map_table_mutex.unlock();

  current_event_cb( _ind->payload, _ind->payload_len, sample_received_ts);
  memset(_ind, 0 , sizeof(sns_client_jumbo_report_ind_msg_v01));

  return;
}
