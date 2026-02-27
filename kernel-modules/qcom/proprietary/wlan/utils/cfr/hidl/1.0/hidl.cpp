/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <android-base/logging.h>
#include <hidl/HidlTransportSupport.h>

#include <android/hidl/manager/1.2/IClientCallback.h>
#include <android/hidl/manager/1.2/IServiceManager.h>

#include "wificfr.h"

extern "C"
{
#include "hidl.h"
}

using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using vendor::qti::hardware::wifi::wificfr::V1_0::IWificfr;
using vendor::qti::hardware::wifi::wificfr::V1_0::implementation::Wificfr;

using android::hardware::Return;
using android::hardware::Status;

using android::hidl::manager::V1_2::IServiceManager;
using android::hidl::manager::V1_2::IClientCallback;
using android::hardware::defaultServiceManager1_2;

#define WIFI_CFR_SERVICE_NAME "wificfr"
#define WIFI_CFR_FQ_SERVICE_NAME "vendor.qti.hardware.wifi.wificfr@1.0::IWificfr"


/*
 * Function     : wificfr_hidl_process
 * Description  : registers as server instance for IWificfr service
 * Input params : pointer to wificfr_ctx
 * Return       : SUCCESS/FAILURE
 *
 */
int wificfr_hidl_process()
{
	android::base::InitLogging(NULL,
			android::base::LogdLogger(android::base::SYSTEM));
	LOG(INFO) << "Wifi CFR Hal is booting up...";

	android::sp<IWificfr> service_wificfr = Wificfr::getInstance();
	configureRpcThreadpool(1, true /* callerWillJoin */);
	if (service_wificfr->registerAsService(WIFI_CFR_SERVICE_NAME) != android::OK) {
		LOG(ERROR) << "Cannot register WifiCfr HAL service";
		return 1;
	}

	joinRpcThreadpool();
	LOG(INFO) << "wificfr is terminating...";

	return 0;
}


void wificfr_hidl_notify_cfrdata(uint8_t* read_buf, int length){

	Wificfr *wifi_cfr = Wificfr::getInstance();
	if (!wifi_cfr)
		return;

	wifi_cfr->wificfr_notify_cfrdata(read_buf, length);
}
