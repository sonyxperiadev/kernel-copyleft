/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#pragma once
#include <vector>
#include <utils/Log.h>

#include "qsh_wifi_interface_reference.h"

#define sns_loge ALOGE
#define sns_logi ALOGI
#define sns_logd ALOGD
#define sns_logv ALOGV

#ifndef UNUSED_VAR
#define UNUSED_VAR(var) ((void)(var));
#endif

/**
 * @brief type alias for an suid event function
 *
 * param datatype: datatype of of the sensor associated with the
 * event
 * param suids: vector of suids available for the given datatype
 */
using suid_event_function =
    std::function<void(const std::string& datatype,
                       const std::vector<sensor_uid>& suids)>;

/**
 * @brief Utility class for discovering available sensors using
 *        dataytpe
 *
 */
class suid_lookup
{
public:
    /**
     * @brief creates a new connection to qsh for suid lookup
     *
     * @param cb callback function for suids
     */
    suid_lookup(suid_event_function cb);
    ~suid_lookup();

    /**
     *  @brief look up the suid for a given datatype, registered
     *         callback will be called when suid is available for
     *         this datatype
     *
     *  @param datatype data type for which suid is requested
     *  @param default_only option to ask for publishing only default
     *         suid for the given data type. default value is false
     */
    void request_suid(std::string datatype, bool default_only = false);

private:
    suid_event_function _cb;
    void handle_qsh_event(const uint8_t *data, size_t size, uint64_t time_stamp);
    qsh_interface *_conn;
    sensor_uid _suid;
    qsh_event_cb _event_cb = nullptr;
};
