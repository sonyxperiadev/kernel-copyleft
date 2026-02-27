/**
  * Copyright (c) 2023 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */

#include <map>
#include <string>
#include <android-base/macros.h>

#include <aidl/vendor/qti/hardware/wifi/qtiwifi/BnQtiWifi.h>
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IQtiWifi.h>
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IQtiWifiCallback.h>
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IfaceInfo.h>
#include <aidl/vendor/qti/hardware/wifi/qtiwifi/IfaceType.h>

extern "C"
{
#include "eloop.h"
#include "qtiwifitool.h"
}

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace wifi {
namespace qtiwifi {

class QtiWifi : public BnQtiWifi
{
public:
	QtiWifi(struct qtiwifi_global*);
	~QtiWifi() override = default;
	bool isValid();

	// Aidl methods exposed.
	::ndk::ScopedAStatus listAvailableInterfaces(
		std::vector<IfaceInfo>* _aidl_return) override;
	::ndk::ScopedAStatus registerQtiWifiCallback(
	        const std::shared_ptr<IQtiWifiCallback>& callback) override;
	::ndk::ScopedAStatus doQtiWifiCmd(
		const std::string& iface, const std::string& cmd, std::string* _aidl_return) override;
private:
	// Corresponding worker functions for the AIDL methods.
	::ndk::ScopedAStatus registerQtiWifiCallbackInternal(
		const std::shared_ptr<IQtiWifiCallback>& callback);
	std::pair<std::vector<IfaceInfo>, ndk::ScopedAStatus> listAvailableInterfacesInternal();
	std::pair<std::string, ndk::ScopedAStatus> doQtiWifiCmdInternal(const std::string& iface, const std::string& cmd);

	struct qtiwifi_global *global_;
	// Callbacks registered.
	std::vector<std::shared_ptr<IQtiWifiCallback>> callbacks_;
	// Death notifier.
	AIBinder_DeathRecipient* death_notifier_;
};
}  // namespace qtiwifi
}  // namespace wifi
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace aidl
