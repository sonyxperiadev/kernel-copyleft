/*
 * Copyright (c) 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include "wificfr.h"
#include "aidl_return_util.h"

using namespace std;
using aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatus;
using aidl::vendor::qti::hardware::wifi::wificfr::CaptureStatusCode;
using aidl::vendor::qti::hardware::wifi::wificfr::IWificfr;

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace wifi {
namespace wificfr {
namespace implementation {

using aidl_return_util::validateAndCall;

bool Wificfr::isValid()
{
	return true;
}

::ndk::ScopedAStatus Wificfr::csiCaptureStart(CaptureStatus* _aidl_return)
{
	return validateAndCall(
		this, CaptureStatusCode::FAILURE_UNKNOWN,
		&Wificfr::csiCaptureStartInternal, _aidl_return);
}

::ndk::ScopedAStatus Wificfr::csiCaptureStop(CaptureStatus* _aidl_return)
{
	return validateAndCall(
		this, CaptureStatusCode::FAILURE_UNKNOWN,
		&Wificfr::csiCaptureStopInternal, _aidl_return);
}

::ndk::ScopedAStatus Wificfr::registerEventCallback(
		const std::shared_ptr<IWificfrDataCallback>& callback,
		CaptureStatus* _aidl_return)
{
	return validateAndCall(
		this, CaptureStatusCode::FAILURE_UNKNOWN,
		&Wificfr::registerEventCallbackInternal, _aidl_return, callback);
}

::ndk::ScopedAStatus Wificfr::unregisterEventCallback(
		const std::shared_ptr<IWificfrDataCallback>& callback,
		CaptureStatus* _aidl_return)
{
	return validateAndCall(
		this, CaptureStatusCode::FAILURE_UNKNOWN,
		&Wificfr::unregisterEventCallbackInternal, _aidl_return, callback);
}

/**
 * Add callback to the corresponding list after linking to
 * death on the corresponding aidl object reference.
 */
template <class CallbackType>
int registerForDeathAndAddCallbackAidlObjectToList(
		const std::shared_ptr<CallbackType> &callback,
		const std::function<void(const std::shared_ptr<CallbackType> &)>
		&on_aidl_died_fctor,
		std::vector<std::shared_ptr<CallbackType>> &callback_list)
{

	callback_list.push_back(callback);
	return 0;
};

int Wificfr::addCallback(const std::shared_ptr<IWificfrDataCallback> &callback)
{
	auto on_aidl_died_fctor = std::bind(&Wificfr::removeCallback, this,
			std::placeholders::_1);

	return registerForDeathAndAddCallbackAidlObjectToList<
		IWificfrDataCallback>(callback, on_aidl_died_fctor,
				callbacks_list);
}

void Wificfr::removeCallback(const std::shared_ptr<IWificfrDataCallback> &callback)
{
	callbacks_list.clear();
}

std::pair<CaptureStatus, ndk::ScopedAStatus>
Wificfr::csiCaptureStartInternal()
{
	CaptureStatus status;
	status.debugMessage = "";
	int ret;

	ret = startCapture();
	if (ret <= 0) {
		status.code = CaptureStatusCode::FAILURE_UNKNOWN;
		return {status, ndk::ScopedAStatus::fromServiceSpecificError(
			static_cast<int32_t>(status.code))};
	}

	status.code = CaptureStatusCode::SUCCESS;
	return {status, ndk::ScopedAStatus::ok()};
}

std::pair<CaptureStatus, ndk::ScopedAStatus>
Wificfr::csiCaptureStopInternal()
{
	CaptureStatus status;
	status.debugMessage = "";
	int ret;

	ret = stopCapture();
	if (ret <= 0) {
		status.code = CaptureStatusCode::FAILURE_UNKNOWN;
		return {status, ndk::ScopedAStatus::fromServiceSpecificError(
			static_cast<int32_t>(status.code))};
	}

	status.code = CaptureStatusCode::SUCCESS;
	return {status, ndk::ScopedAStatus::ok()};
}

std::pair<CaptureStatus, ndk::ScopedAStatus>
Wificfr::registerEventCallbackInternal(
		const std::shared_ptr<IWificfrDataCallback>& callback)
{
	CaptureStatus status;
	status.debugMessage = "";

	addCallback(callback);

	status.code = CaptureStatusCode::SUCCESS;
	return {status, ndk::ScopedAStatus::ok()};
}

std::pair<CaptureStatus, ndk::ScopedAStatus>
Wificfr::unregisterEventCallbackInternal(
		const std::shared_ptr<IWificfrDataCallback>& callback)
{
	CaptureStatus status;
	status.debugMessage = "";

	removeCallback(callback);

	status.code = CaptureStatusCode::SUCCESS;
	return {status, ndk::ScopedAStatus::ok()};
}

void Wificfr::wificfr_notify_cfrdata(uint8_t* read_buf, int length)
{
	std::vector<uint8_t> aidl_info;
	aidl_info.resize(length);
	memcpy(&aidl_info[0], read_buf, length);

	for (const auto& callback : callbacks_list) {
		if (!callback->onCfrDataAvailable(aidl_info).isOk()) {
			LOG(ERROR)
				<< "Failed to invoke onCfrDataAvailable callback";
		}
	}
}

}  // namespace implementation
}  // namespace wificfr
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace aidl
