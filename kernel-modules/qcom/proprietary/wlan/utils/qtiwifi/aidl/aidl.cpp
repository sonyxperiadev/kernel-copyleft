/**
  * Copyright (c) 2023 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */

#include "qtiwifi.h"
#include <android/binder_process.h>
#include <android/binder_manager.h>

extern "C"
{
#include "aidl.h"
#include "eloop.h"
#include "qtiwifitool.h"
}

using aidl::vendor::qti::hardware::wifi::qtiwifi::QtiWifi;

// This file is a bridge between the qtiwifitool code written in 'C' and the aidl
// interface in C++. So, using "C" style static globals here!
static int aidl_fd = -1;
static std::shared_ptr<QtiWifi> service;

void qtiwifi_aidl_sock_handler(
    int /* sock */, void * /* eloop_ctx */, void * /* sock_ctx */)
{
	ABinderProcess_handlePolledCommands();
}

int qtiwifi_aidl_init(struct qtiwifi_global* global)
{
	std::string instance;   // declared here to allow use of goto

	ABinderProcess_setupPolling(&aidl_fd);
	if (aidl_fd < 0)
		goto err;

	if (eloop_register_read_sock(
		aidl_fd, qtiwifi_aidl_sock_handler, NULL, NULL) < 0)
		goto err;

	service = ndk::SharedRefBase::make<QtiWifi>(global);
	if (!service)
		goto err;
	instance = std::string() + QtiWifi::descriptor + "/default";
	if (AServiceManager_addService(service->asBinder().get(), instance.c_str()) != STATUS_OK)
		goto err;
	return 0;
err:
	qtiwifi_aidl_deinit(NULL);
	return -1;
}

void qtiwifi_aidl_deinit(struct qtiwifi_global* global)
{
	if (global != NULL)
		global->cb = NULL;
	eloop_unregister_read_sock(aidl_fd);
	aidl_fd = -1;
}
