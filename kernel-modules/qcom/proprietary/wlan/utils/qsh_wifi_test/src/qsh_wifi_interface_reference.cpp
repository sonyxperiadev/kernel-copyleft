/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "qsh_wifi_interface_reference.h"
#include "qsh_wifi_qmi_reference.h"
#include "qsh_wifi_glink_reference.h"

qsh_interface* qsh_interface::create(qsh_connection_type connection_type, qsh_conn_config config) {
  switch (connection_type) {
  case QSH_QMI:
    return new qsh_qmi();
  case QSH_GLINK:
    return new qsh_glink(config.glink_config);
  default:
    return nullptr;
  }
}

qsh_interface::~qsh_interface(){

}
