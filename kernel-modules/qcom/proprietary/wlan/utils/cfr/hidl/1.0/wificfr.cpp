/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include "wificfr.h"
#include "hidl_return_util.h"
#include <hidl/HidlTransportSupport.h>
using android::hardware::interfacesEqual;

namespace {
	using vendor::qti::hardware::wifi::wificfr::V1_0::IWificfr;
}  // namespace

namespace vendor {
namespace qti {
namespace hardware {
namespace wifi {
namespace wificfr {
namespace V1_0 {
namespace implementation {

using hidl_return_util::call;

Wificfr *Wificfr::instance_=NULL;

Wificfr *Wificfr::getInstance()
{
	if (!instance_)
		instance_ = new Wificfr();
	return instance_;
}

void Wificfr::destroyInstance()
{
	if (instance_)
		delete instance_;
	instance_ = NULL;
}

Return<void> Wificfr::csiCaptureStart(csiCaptureStart_cb _hidl_cb) {
	return call(this, &Wificfr::csiCaptureStartInternal, _hidl_cb);
}

Return<void> Wificfr::csiCaptureStop(csiCaptureStop_cb _hidl_cb) {
	return call(this, &Wificfr::csiCaptureStopInternal, _hidl_cb);
}

Return<void> Wificfr::registerEventCallback(
		const android::sp<IWificfrDataCallback>& callback,
		registerEventCallback_cb _hidl_cb)
{
	return call(this, &Wificfr::registerEventCallbackInternal, _hidl_cb,
			callback);
}

Return<void> Wificfr::unregisterEventCallback(
		const android::sp<IWificfrDataCallback>& callback,
		unregisterEventCallback_cb _hidl_cb)
{
	return call(this, &Wificfr::unregisterEventCallbackInternal, _hidl_cb,
			callback);
}

/**
 * Add callback to the corresponding list after linking to
 * death on the corresponding hidl object reference.
 */
template <class CallbackType>
int registerForDeathAndAddCallbackHidlObjectToList(
		const android::sp<CallbackType> &callback,
		const std::function<void(const android::sp<CallbackType> &)>
		&on_hidl_died_fctor,
		std::vector<android::sp<CallbackType>> &callback_list)
{

	callback_list.push_back(callback);
	return 0;
};

int Wificfr::addCallback(const android::sp<IWificfrDataCallback> &callback)
{
	auto on_hidl_died_fctor = std::bind(&Wificfr::removeCallback, this,
			std::placeholders::_1);
	return registerForDeathAndAddCallbackHidlObjectToList<
		IWificfrDataCallback>(callback, on_hidl_died_fctor,
				callbacks_list);
}

void Wificfr::removeCallback(const android::sp<IWificfrDataCallback> &callback)
{
	callbacks_list.clear();
}

CaptureStatus Wificfr::csiCaptureStartInternal()
{
	int ret;
	ret = startCapture();
	if (ret) {
		return {CaptureStatusCode::FAILURE_UNKNOWN, ""};
	}
	return {CaptureStatusCode::SUCCESS, ""};
}

CaptureStatus Wificfr::csiCaptureStopInternal()
{
	int ret;
	ret = stopCapture();
	if (ret) {
		return {CaptureStatusCode::FAILURE_UNKNOWN, ""};
	}
	return {CaptureStatusCode::SUCCESS, ""};
}

CaptureStatus Wificfr::registerEventCallbackInternal(
		const android::sp<IWificfrDataCallback>& callback)
{
	addCallback(callback);
	return {CaptureStatusCode::SUCCESS, ""};
}

CaptureStatus Wificfr::unregisterEventCallbackInternal(
		const android::sp<IWificfrDataCallback>& callback)
{
	removeCallback(callback);
	return {CaptureStatusCode::SUCCESS, ""};
}


void Wificfr::wificfr_notify_cfrdata(uint8_t* read_buf, int length)
{
	hidl_vec<uint8_t> hidl_info;
	hidl_info.resize(length);
	memcpy(&hidl_info[0], read_buf, length);
	for (const auto& callback : callbacks_list) {
		if (!callback->onCfrDataAvailable(hidl_info).isOk()) {
			LOG(ERROR)
				<< "Failed to invoke onCfrDataAvailable callback";
		}
	}
}

}  // namespace implementation
}  // namespace V1_0
}  // namespace wificfr
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
