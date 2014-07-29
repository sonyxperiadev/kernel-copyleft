
#include "usb/UsbSpy.h"

static int (*notify)(void *, int, void (*)(void *, int online), void *);

int msm_register_usb_ext_notification(struct usb_ext_notification *info){
	notify = info->notify;
	return 0;
}

int usb_spy_clean(void){
	notify = NULL;
}

notify_def get_registerred_device_discovery_pointer(void)
{
 	return notify;
}
