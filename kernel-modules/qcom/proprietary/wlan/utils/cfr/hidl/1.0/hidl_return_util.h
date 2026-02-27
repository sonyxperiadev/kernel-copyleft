/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */


#ifndef HIDL_RETURN_UTIL_H_
#define HIDL_RETURN_UTIL_H_

#include <functional>

namespace vendor {
namespace qti {
namespace hardware {
namespace wifi {
namespace wificfr {
namespace V1_0 {
namespace implementation {
namespace hidl_return_util {

/**
 * These utility functions are used to invoke a method on the provided
 * HIDL interface object.
 */
// Use for HIDL methods which return only an instance of WifiCfrCaptureStatus.
template <typename ObjT, typename WorkFuncT, typename... Args>
Return<void> call(ObjT* obj, WorkFuncT&& work,
		const std::function<void(const CaptureStatus&)>& hidl_cb,
		Args&&... args)
{
	hidl_cb((obj->*work)(std::forward<Args>(args)...));
	return Void();
}


// Use for HIDL methods which return instance of WifiCsiCaptureStatus and a single
// return value.
	template<typename ObjT, typename WorkFuncT, typename ReturnT, typename... Args>
Return<void> call(ObjT* obj, WorkFuncT&& work,
		const std::function<void(const CaptureStatus&, ReturnT)>& hidl_cb,
		Args&&... args)
{
	const auto& ret_pair = (obj->*work)(std::forward<Args>(args)...);
	const CaptureStatus& status = std::get<0>(ret_pair);
	const auto& ret_value = std::get<1>(ret_pair);
	hidl_cb(status, ret_value);
	return Void();
}

}  // namespace hidl_return_util
}  // namespace implementation
}  // namespace V1_0
}  // namespace wificfr
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // HIDL_RETURN_UTIL_H_
