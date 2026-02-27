/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef WIFICFR_AIDL_H
#define WIFICFR_AIDL_H

#include <android-base/macros.h>
#include <aidl/vendor/qti/hardware/wifi/wificfr/IWificfr.h>
#include <aidl/vendor/qti/hardware/wifi/wificfr/IWificfrDataCallback.h>
#include <aidl/vendor/qti/hardware/wifi/wificfr/CaptureStatus.h>
#include <aidl/vendor/qti/hardware/wifi/wificfr/CaptureStatusCode.h>
#include <aidl/vendor/qti/hardware/wifi/wificfr/BnWificfr.h>

extern "C"
{
#include "cfrtool.h"
#include <string.h>
}

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace wifi {
namespace wificfr {
namespace implementation {

class Wificfr : public BnWificfr
{
public:
	Wificfr() = default;
	~Wificfr() override = default;
	bool isValid();
	//AIDL exposed methods
	::ndk::ScopedAStatus csiCaptureStart(::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus* _aidl_return) override;
	::ndk::ScopedAStatus csiCaptureStop(::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus* _aidl_return) override;
	::ndk::ScopedAStatus registerEventCallback(const std::shared_ptr<IWificfrDataCallback> &callback,
			::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus* _aidl_return) override;
	::ndk::ScopedAStatus unregisterEventCallback(const std::shared_ptr<IWificfrDataCallback>& callback,
			::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus* _aidl_return) override;
	void wificfr_notify_cfrdata(uint8_t* read_buf, int length);
private:
	std::vector<std::shared_ptr<IWificfrDataCallback>> callbacks_list;
	std::pair<::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus, ndk::ScopedAStatus>
		csiCaptureStartInternal();
	std::pair<::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus, ndk::ScopedAStatus>
		csiCaptureStopInternal();
	DISALLOW_COPY_AND_ASSIGN(Wificfr);
	std::pair<::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus, ndk::ScopedAStatus>
	registerEventCallbackInternal(
			const std::shared_ptr<IWificfrDataCallback>& callback);
	std::pair<::aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus, ndk::ScopedAStatus>
	unregisterEventCallbackInternal(
			const std::shared_ptr<IWificfrDataCallback>& callback);
	int addCallback(const std::shared_ptr<IWificfrDataCallback> &callback);
	void removeCallback(const std::shared_ptr<IWificfrDataCallback> &callback);
};

}  // namespace implementation
}  // namespace wificfr
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace aidl
#endif //WIFICFR_AIDL_H

