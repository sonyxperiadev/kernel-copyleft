/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "qsh_wifi_glink_reference.h"

qsh_glink::qsh_glink(qsh_glink_config config) {
  UNUSED(config);
}

qsh_glink::~qsh_glink() {

}
int qsh_glink::register_cb(sensor_uid suid, qsh_resp_cb resp_cb, qsh_error_cb error_cb, qsh_event_cb event_cb) {
  UNUSED(suid);
  UNUSED(resp_cb);
  UNUSED(error_cb);
  UNUSED(event_cb);
  return 0;
}
void qsh_glink::send_request(sensor_uid suid, bool use_jumbo_request, std::string encoded_buffer) {
  UNUSED(suid);
  UNUSED(use_jumbo_request);
  UNUSED(encoded_buffer);
}
