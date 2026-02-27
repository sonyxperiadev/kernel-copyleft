/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include "wificfr.h"

extern "C"
{
#include "aidl.h"
}

using aidl::vendor::qti::hardware::wifi::wificfr::implementation::Wificfr;

#define WIFI_CFR_SERVICE_NAME "wificfr"
std::shared_ptr<Wificfr> wificfr;
/*
 * Function     : wificfr_aidl_process
 * Description  : registers as server instance for IWificfr service
 * Input params : pointer to wificfr_ctx
 * Return       : SUCCESS/FAILURE
 *
 */
int wificfr_hidl_process()
{
	android::base::InitLogging(NULL,
			android::base::LogdLogger(android::base::SYSTEM));
	LOG(INFO) << "Wifi CFR Aidl is booting up...";

	ABinderProcess_setThreadPoolMaxThreadCount(0);
	wificfr = ndk::SharedRefBase::make<Wificfr>();

	if (wificfr != nullptr) {
		std::string instance = std::string() + Wificfr::descriptor + "/default";
		if (AServiceManager_addService(wificfr->asBinder().get(),
			instance.c_str()) != STATUS_OK)
		{
			LOG(ERROR) << "Can't register Wificfr AIDL service";
			return 0;
		}
	} else {
		LOG(ERROR) << "Can't create instance of Wificfr, nullptr";
		return 0;
	}
	ABinderProcess_joinThreadPool();
	LOG(INFO) << "wificfr is terminating...";

	return 0;
}


void wificfr_hidl_notify_cfrdata(uint8_t* read_buf, int length){

	if (!wificfr) {
		return;
	}
	wificfr->wificfr_notify_cfrdata(read_buf, length);
}
