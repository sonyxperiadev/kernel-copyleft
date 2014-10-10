#ifndef USB_SPY_H_
#define USB_SPY_H_

#include <linux/usb/msm_hsusb.h>

typedef int (*notify_def)(void *, int, void (*)(void *, int online), void *);

int usb_spy_clean(void);
notify_def get_registerred_device_discovery_pointer(void);

enum dwc3_id_state {
	DWC3_ID_GROUND = 0,
	DWC3_ID_FLOAT
};

void spy_gpio_MockMode(void);

#endif /* USB_SPY_H_ */
