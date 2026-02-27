/*
 * Copyright (c) 2013-2021, 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc
 *
 */
#define LOG_TAG "lowirpcd"
#ifndef VERIFY_PRINT_ERROR
#define VERIFY_PRINT_ERROR
#endif
#define VERIFY_PRINT_INFO 0

#include <stdio.h>
#include <dlfcn.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <remote.h>
#include <AEEStdErr.h>
#include <verify.h>
#include <pthread.h>
pthread_mutex_t mutex;
pthread_cond_t  cond;

#ifndef ADSP_LIBHIDL_NAME
#define ADSP_LIBHIDL_NAME "libhidlbase.so"
#endif

#ifdef __clang__
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif // __clang__

typedef int (*adsp_default_listener_start_t)(int argc, char *argv[]);
typedef int (*remote_handle_control_t)(uint32_t req, void* data, uint32_t len);

/*check for slpi/adsp and load the library accordingly*/
const char *get_lib() {
  struct stat sb;
  if(!stat("/sys/kernel/boot_slpi", &sb)){
      return "libssc_default_listener.so";
  }
  else {
      return "libadsp_default_listener.so";
  }
}

bool is_slpi_present()
{
  struct stat sb;
  if(!stat("/sys/kernel/boot_slpi", &sb))
    return true;
  else
    return false;
}

int request_fastrpc_wakelock(void *adsphandler) {
  int nErr = 0;
#ifdef FASTRPC_WAKELOCK_CONTROL_SUPPORTED
  remote_handle_control_t handle_control;
  struct remote_rpc_control_wakelock data;

  data.enable = 1;

  if (NULL != (handle_control = (remote_handle_control_t)dlsym(adsphandler, "remote_handle_control"))) {
    VERIFY_IPRINTF("found remote_handle_control, requesting for wakelock");
    nErr = handle_control(DSPRPC_CONTROL_WAKELOCK, (void*)&data, sizeof(data));
    if (nErr == AEE_EUNSUPPORTEDAPI) {
      VERIFY_EPRINTF("fastrpc wakelock request is not supported");
      /* this feature may not be supported by all targets
         treat this case as normal since we still can call listener_start */
      nErr = AEE_SUCCESS;
    } else if (nErr) {
      VERIFY_EPRINTF("failed to enable fastrpc wake-lock control, %x", nErr);
    } else {
      VERIFY_IPRINTF("fastrpc wakelock control is enabled");
    }
  } else {
    VERIFY_EPRINTF("unable to find remote_handle_control, %s", dlerror());
    /* there should be no case where remote_handle_control doesn't exist */
    nErr = AEE_EFAILED;
  }
#endif
  return nErr;
}

int main(int argc, char *argv[]) {

  int nErr = 0;
  void *adsphandler = NULL, *libhidlbaseHandler = NULL;
  adsp_default_listener_start_t listener_start;

  pthread_mutex_init(&mutex, NULL);
  pthread_cond_init(&cond, NULL);

  if ((argv[1] == NULL) &&(!is_slpi_present()))
  {
    VERIFY_EPRINTF("Do not start for lowirpscd when SLPI Proc is not present");
    pthread_mutex_lock(&mutex);
    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex);
  }

  VERIFY_IPRINTF("lowirpcd daemon starting");
  if( NULL != ((libhidlbaseHandler = dlopen(ADSP_LIBHIDL_NAME, RTLD_NOW))) ){
    while (1) {
      if(NULL != (adsphandler = dlopen(get_lib(), RTLD_NOW))) {
        if(NULL != (listener_start =
          (adsp_default_listener_start_t)dlsym(adsphandler, "adsp_default_listener_start"))) {
          nErr = request_fastrpc_wakelock(adsphandler);
          if (nErr) {
            VERIFY_EPRINTF("request_fastrpc_wakelock failed with, %x", nErr);
          }
          listener_start(argc, argv);
          VERIFY_EPRINTF("listener_start exits");
        }
        if(0 != dlclose(adsphandler)) {
          VERIFY_EPRINTF("dlclose failed");
        }
      }else {
        VERIFY_EPRINTF("fail to open due to %s", dlerror());
      }
      VERIFY_EPRINTF("lowirpcd daemon will restart after 25ms...");
      usleep(25000);
    }
    if(0 != dlclose(libhidlbaseHandler)) {
      VERIFY_EPRINTF("libhidlbase dlclose failed");
    }
    VERIFY_IPRINTF("lowirpcd daemon exiting %x", nErr);

    return nErr;
  }else {
    VERIFY_EPRINTF(" libhidlbase dlopen failed");
  }
  pthread_mutex_destroy(&mutex);
  pthread_cond_destroy(&cond);
  return 0;
}
