/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#pragma once
#include "qsh_wifi_interface_reference.h"

class qsh_glink : public qsh_interface {
public:
  qsh_glink(qsh_glink_config config);
  ~qsh_glink();
  int register_cb(sensor_uid suid,   qsh_resp_cb resp_cb, qsh_error_cb error_cb, qsh_event_cb event_cb);
  void send_request(sensor_uid suid, bool use_jumbo_request, std::string encoded_buffer);
};
