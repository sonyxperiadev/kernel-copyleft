/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#pragma once
#include "qsh_wifi_interface_reference.h"

class qsh_register_cb {
public:
  qsh_register_cb(qsh_resp_cb resp_cb,
                  qsh_error_cb error_cb,
                  qsh_event_cb event_cb
                  ) {
    _resp_cb = resp_cb;
    _error_cb = error_cb;
    _event_cb = event_cb;
  }

  inline qsh_resp_cb get_resp_cb() { return _resp_cb;}
  inline qsh_error_cb get_error_cb() { return _error_cb;}
  inline qsh_event_cb get_event_cb() { return _event_cb;}

private:
  qsh_resp_cb _resp_cb;
  qsh_error_cb _error_cb;
  qsh_event_cb _event_cb;
};
