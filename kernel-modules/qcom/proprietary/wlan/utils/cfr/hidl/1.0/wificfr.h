/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef WIFICFR_HIDL_H
#define WIFICFR_HIDL_H

#include <android-base/macros.h>
#include <vendor/qti/hardware/wifi/wificfr/1.0/IWificfr.h>
#include <vendor/qti/hardware/wifi/wificfr/1.0/IWificfrDataCallback.h>

extern "C"
{
//#include "wificsi.h"
//#include "nl_utils.h"
#include "cfrtool.h"
#include <string.h>
}


   namespace vendor {
   namespace qti {
   namespace hardware {
   namespace wifi {
   namespace wificfr {
   namespace V1_0 {
   namespace implementation {

using namespace android::hardware;
using namespace vendor::qti::hardware::wifi::wificfr::V1_0;


class Wificfr : public V1_0::IWificfr {
	public:
		Wificfr() = default;
		~Wificfr() override = default;
		static Wificfr* getInstance();
		static void destroyInstance();
		//HIDL exposed methods
		Return<void> csiCaptureStart(csiCaptureStart_cb _hidl_cb) override;
		Return<void> csiCaptureStop(csiCaptureStop_cb _hidl_cb) override;
		Return<void> registerEventCallback(const android::sp<IWificfrDataCallback>& callback,
				registerEventCallback_cb _hidl_cb) override;
		Return<void> unregisterEventCallback(const android::sp<IWificfrDataCallback>& callback,
				unregisterEventCallback_cb _hidl_cb) override;
		void wificfr_notify_cfrdata(uint8_t* read_buf, int length);
	private:
		std::vector<android::sp<IWificfrDataCallback>> callbacks_list;
		static Wificfr* instance_;
		CaptureStatus csiCaptureStartInternal();
		CaptureStatus csiCaptureStopInternal();
		DISALLOW_COPY_AND_ASSIGN(Wificfr);
		CaptureStatus registerEventCallbackInternal(
				const android::sp<IWificfrDataCallback>& callback);
		CaptureStatus unregisterEventCallbackInternal(
				const android::sp<IWificfrDataCallback>& callback);
		int addCallback(const android::sp<IWificfrDataCallback> &callback);
		void removeCallback(const android::sp<IWificfrDataCallback> &callback);
};


}  // namespace implementation
}  // namespace V1_0
}  // namespace wificfr
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif //WIFICFR_HIDL_H
