/**
  * Copyright (c) 2023 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <net/if.h>
#include <log/log.h>
#include <sys/socket.h>
#include <linux/if_bridge.h>

#include <android-base/file.h>
#include <android-base/stringprintf.h>
#include <android-base/unique_fd.h>

#include "qtiwifi.h"
#include "aidl_return_util.h"
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IQtiWifiCallback.h>
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IfaceInfo.h>
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IfaceType.h>

extern "C"
{
#include "qtiwifitool.h"
}

namespace {

using ::aidl::vendor::qti::hardware::wifi::qtiwifi::QtiWifiStatusCode;

inline ndk::ScopedAStatus createStatus(QtiWifiStatusCode status_code) {
	return ndk::ScopedAStatus::fromServiceSpecificError(
		static_cast<int32_t>(status_code));
}

// Method called by death_notifier_ on client death.
void onDeath(void* cookie) {
	ALOGI("client died, cookie = %d, terminating...", *(int*)cookie);
	eloop_terminate();
}

std::function<void(const char*, const char*)> on_ctrl_event_internal_callback;
void onVendorCtrlEventCb(const char *iface, const char* msg)
{
	if (!msg)
		return;
	if (on_ctrl_event_internal_callback) {
		 on_ctrl_event_internal_callback(iface, msg);
	}
}

}  // namespace

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace wifi {
namespace qtiwifi {

using ::aidl::vendor::qti::hardware::wifi::qtiwifi::aidl_return_util::validateAndCall;
using ::aidl::vendor::qti::hardware::wifi::qtiwifi::QtiWifiStatusCode;

QtiWifi::QtiWifi(struct qtiwifi_global* global)
    :global_(global)
{
	death_notifier_ = AIBinder_DeathRecipient_new(onDeath);
}

bool QtiWifi::isValid()
{
	return true;
}

::ndk::ScopedAStatus QtiWifi::registerQtiWifiCallback(
	const std::shared_ptr<IQtiWifiCallback>& callback)
{
	return registerQtiWifiCallbackInternal(callback);
}

::ndk::ScopedAStatus QtiWifi::listAvailableInterfaces(
	std::vector<IfaceInfo>* _aidl_return)
{
	return validateAndCall(
		this, QtiWifiStatusCode::FAILURE,
		&QtiWifi::listAvailableInterfacesInternal, _aidl_return);
}

::ndk::ScopedAStatus QtiWifi::doQtiWifiCmd(
	const std::string& iface, const std::string& cmd, std::string* _aidl_return)
{
	return validateAndCall(
		this, QtiWifiStatusCode::FAILURE,
		&QtiWifi::doQtiWifiCmdInternal, _aidl_return, iface, cmd);
}

::ndk::ScopedAStatus QtiWifi::registerQtiWifiCallbackInternal(
	const std::shared_ptr<IQtiWifiCallback>& callback)
{
	binder_status_t status = AIBinder_linkToDeath(callback->asBinder().get(),
			death_notifier_, this /* cookie */);
	if (status != STATUS_OK) {
		return createStatus(QtiWifiStatusCode::FAILURE);
	}
	// no iface_name provided, treat it as global callback
	// Hook the hapd callback if not registered.
	if (!on_ctrl_event_internal_callback) {
		on_ctrl_event_internal_callback =
			[this](const char *iface, const char *msg) {
			if (callbacks_.size() == 0) {
				return;
			}
			const std::string ifname(iface);
			const std::string event_str(msg);
			for (const auto& callback : callbacks_) {
				callback->onCtrlEvent(ifname, event_str);
			}
		};
	}
	global_->ctrl_event_qtiwifi_aidl_cb = onVendorCtrlEventCb;
	callbacks_.push_back(callback);
	return ndk::ScopedAStatus::ok();
}

std::pair<std::vector<IfaceInfo>, ndk::ScopedAStatus>
QtiWifi::listAvailableInterfacesInternal()
{
	std::vector<IfaceInfo> ifaces;
	qtiwifi_get_interfaces();
	for (int i = 0; i < qtiwifi_num_ifaces; ++i) {
		IfaceInfo info;
		info.name = qtiwifi_interfaces[i].name;
		info.type = qtiwifi_interfaces[i].type == 2 ?
			IfaceType::STA : IfaceType::AP;
		ifaces.push_back(info);
	}
	return {std::move(ifaces), ndk::ScopedAStatus::ok()};
}

std::pair<std::string, ndk::ScopedAStatus>
QtiWifi::doQtiWifiCmdInternal(const std::string& iface_name, const std::string& cmd)
{
	const int reply_cap = 4096;

	char* reply = (char*) malloc (reply_cap);
	if (reply == NULL) {
		return {"", createStatus(QtiWifiStatusCode::FAILURE)};
	}
	int reply_len = qtiwifi_process_cmd(global_, iface_name.c_str(), cmd.c_str(),
                                            reply, reply_cap);
	if (reply_len > reply_cap) reply_len = reply_cap;
	reply[reply_len] = '\0';
	std::string str_reply(reply);
	free(reply);
	return {std::move(str_reply), ndk::ScopedAStatus::ok()};
}

}  // namespace qtiwifi
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace aidl
