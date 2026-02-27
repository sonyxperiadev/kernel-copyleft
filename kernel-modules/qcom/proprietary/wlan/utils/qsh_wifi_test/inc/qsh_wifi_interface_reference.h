/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#pragma once
#include <string>
#define UNUSED(var) (void)(var)

/**
 * @brief Struct to represent sensor's unique ID (128-bit)
 */
struct sensor_uid
{
  sensor_uid() : low(0), high(0) {}
  sensor_uid(uint64_t low, uint64_t high): low(low), high(high) {}
  bool operator==(const sensor_uid& rhs) const
  {
    return (low == rhs.low && high == rhs.high);
  }
  uint64_t low, high;
};

/*
 * Type of transport to establish the connection
 * QSH_QMI : for QMI as transport layer
 * QSH_GLINK: For Glink as transport layer
 * */
typedef enum qsh_connection_type{
  QSH_QMI,
  QSH_GLINK,
}qsh_connection_type;

 /*
 * error type if any from qsh_interface point of view
 * */
enum qsh_interface_error
{
  QSH_INTERFACE_RESET
};

/**
 * type alias for qsh_interface resp, error and event callbacks respectively.
 */
using qsh_resp_cb = std::function<void(const uint32_t error_value, uint64_t client_connect_id)>;
using qsh_error_cb = std::function<void(qsh_interface_error error)>;
using qsh_event_cb = std::function<void(const uint8_t *data, size_t size, uint64_t ts)>;

/*
* @brief: Configuration parameters exposed to client
*         for QMI transport layer.
* */
typedef struct qsh_qmi_config{
  char* transport_name;
}qsh_qmi_config;

/*
* @brief: Configuration parameters exposed to client
*         for Glink transport layer.
* @Param: glink transport name for the requested connection.
* */
typedef struct qsh_glink_config{
  char* transport_name;
}qsh_glink_config;

/*
* @brief: Configuration parameters exposed to client
*         for the requested connection
* @Paras:  can be one of connection type config
* */
typedef struct qsh_conn_config {
  union {
    qsh_qmi_config qmi_config;
    qsh_glink_config glink_config;
  };
}qsh_conn_config;

/*
 * Interface APIs to interact with sensors core from sensors Apps
 * */
class qsh_interface {
public:
  /* @brief: Create the interface between the clients and sensors core.
   * @param: transport layer type as per qsh_connection_type
   *
   * */
  static qsh_interface* create(qsh_connection_type connection_type, qsh_conn_config config_params);

  /**
   * @brief registers the callbacks for the given suid.
   *        Client needs to call only once per given suid.
   *
   * @param: callback pointers, resp, error and event.
   *         All are mandatory. Client can pass nullptr incase do not need any.
   *
   * @out: 0 upon success & -1 upon failure.
   */
  virtual int register_cb(sensor_uid suid, qsh_resp_cb resp_cb, qsh_error_cb error_cb, qsh_event_cb event_cb) = 0;

  /**
   * @brief send proto encoded message to sensors core via QMI.
   *
   * @param: suid of the given sensor
   *
   * @param: use_jumbo_request True if client needs jumbo event packet , else false
   *
   * @param: encoded_buffer: proto encoded request message
   */
  virtual void send_request(sensor_uid suid, bool use_jumbo_request, std::string encoded_buffer) = 0;
  virtual ~qsh_interface();
protected:
};
