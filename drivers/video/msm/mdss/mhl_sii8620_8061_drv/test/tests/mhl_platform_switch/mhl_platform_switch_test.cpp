#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

extern "C" {

#include <linux/usb/msm_hsusb.h>
#include "usb/UsbSpy.h"
#include "GpioSpy.h"
#include "util.h"
#include "mhl_platform.h"
#include "mhl_common.h"
extern int mhl_pf_switch_init(void);
extern void mhl_pf_switch_exit(void);

}

int dummy_ctx;
static bool is_dd_cb_called = false;
static int online_ = -1;
static int (*cb_mhl_pf_switch_device_discovery)(void *, int, void (*)(void *, int online), void *);

static void notify_usb_online(void *ctx, int online)
{
	online_ = online;
}

static void mock_notify_usb_online(void *ctx, int online)
{
	mock("NOTIFY_USB_ONLINE").actualCall("mock_notify_usb_online");
}

static int dd_cb_inuse(void *context){
	return MHL_USB_INUSE;
}

static int dd_cb_non_inuse(void *context){
	return MHL_USB_NON_INUSE;
}

static int dd_cb(void *context){
	is_dd_cb_called = true;
	return MHL_USB_INUSE;
}

TEST_GROUP(MhlPlatformSwitch) {
	void setup() {
		usb_spy_clean();
		gpio_spy_clean();
		common_spy_clean();
		is_dd_cb_called = false;
		online_ = -1;

		/* Those become active when MockMode is enabled */
		mock("GPIO").strictOrder();
		mock("DELAY").strictOrder();
		mock("NOTIFY_USB_ONLINE").strictOrder();
	}
	void teardown() {
		/* Those become active when MockMode is enabled */
		mock("GPIO").checkExpectations();
		mock("DELAY").checkExpectations();
		mock("NOTIFY_USB_ONLINE").checkExpectations();
		mock().clear();
	}
};



TEST(MhlPlatformSwitch, 001_return_inuse_and_gpio_high_at_int_by_pmic) {
	int rc = MHL_USB_NON_INUSE;
	
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */
	/* Emulate that the PMIC irq*/
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);
	/* verify */
	/*
	 * Even if there is no registerred call back,
	 * MHL_USB_INUSE must be returned since the
	 * MHL ko object is supposed to be installed
	 * at boot timing and immediately registerring
	 * its call back. So this API must return INUSE
	 * to usb.
	 */
	LONGS_EQUAL(MHL_USB_INUSE, rc);
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();
}

TEST(MhlPlatformSwitch, 002_return_inuse_and_gpio_high_at_int_by_pmic) {
	int rc = MHL_USB_NON_INUSE;
	int inuse = 0;
	
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */
	/* client register its call back for the device discovery*/
	mhl_pf_switch_register_cb(dd_cb_inuse,NULL);
	/* Emulate that the PMIC irq*/
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);


	/* verify */
	LONGS_EQUAL(inuse, rc);
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 003_return_non_inuse_and_gpio_low_at_int_by_pmic) {
	int rc = MHL_USB_NON_INUSE;
	
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */
	/* client register its call back for the device discovery*/
	mhl_pf_switch_register_cb(dd_cb_non_inuse,NULL);
	/* 
	 * Emulate that the PMIC irq. Registerred dd_cb_non_inuse() is 
	* emulate that it is not MHL impedance and return MHL_USB_NON_INUSE.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);

	/* verify */
	CHECK_TRUE(rc != MHL_USB_INUSE);
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 005_registerred_cb_is_called_at_registration) {
	int rc = MHL_USB_NON_INUSE;
	
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);

	/* 
	 * client registers its call back (dd_cb) for the device discovery 
	 * and it is called by mhl_pf_switch in this API context.
	 */
	mhl_pf_switch_register_cb(dd_cb,NULL);

	/* verify */
	CHECK_TRUE(is_dd_cb_called);
	CHECK_EQUAL(MHL_USB_INUSE,rc);
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 006_sel_gpio_are_low_and_notify_0_when_switch_to_usb) {
	int rc = MHL_USB_NON_INUSE;
	int res = MHL_SUCCESS;
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */	
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);
	/* 
	 * This API must be called after 
	 * mhl_pf_switch_device_discovery is called
	 * by USB 
	 */
	res = mhl_pf_switch_to_usb();

	/* verify */
	CHECK_EQUAL(MHL_SUCCESS, res);
	CHECK_EQUAL(0, online_);
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 007_sel_gpio_are_low_and_online_0_when_switch_to_usb) {
	int rc = MHL_USB_NON_INUSE;
	int res = MHL_FAIL;
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */	
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);

	/* 
	 * This API must be called after 
	 * mhl_pf_switch_device_discovery is called
	 * by USB 
	 */
	res = mhl_pf_switch_to_usb();

	/* verify */
	LONGS_EQUAL(0, online_);
	LONGS_EQUAL(MHL_SUCCESS, res);
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 007_2_switch_to_usb_does_not_fail_even_before_dd_call) {
	int rc = MHL_USB_NON_INUSE;
	int res = MHL_FAIL;
	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();
	gpio_set_value(GPIO_MHL_SWITCH_SEL_1,1);
	gpio_set_value(GPIO_MHL_SWITCH_SEL_2,1);

	/* exe */	
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	/* rc = mhl_pf_switch_device_discovery(NULL,0,notify_usb_online); */

	/* 
	 * This API may be called after 
	 * mhl_pf_switch_device_discovery is called
	 * by USB, otherwise the "online" notification can't be sent to USB.
	 * Thought the notification doesn't occur, the GPIO will be somehow switched to USB.
	 * (USB is default. So, the system will try to switch to USB.)
	 */
	res = mhl_pf_switch_to_usb();

	/* verify */
	LONGS_EQUAL(-1, online_);
	LONGS_EQUAL(MHL_SUCCESS, res);
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}



TEST(MhlPlatformSwitch, 008_sel_gpio_are_high_and_online_1_when_switch_to_mhl) {
	int rc = MHL_USB_NON_INUSE;
	int res = MHL_FAIL;

	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();

	/* exe */	
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, 0, notify_usb_online ,&dummy_ctx);

	/* 
	 * This API must be called after 
	 * mhl_pf_switch_device_discovery is called
	 * by USB 
	 */
	res = mhl_pf_switch_to_mhl();

	/* verify */
	CHECK_EQUAL(1, online_);
	LONGS_EQUAL(MHL_SUCCESS, res);
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(1, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 008_2_switch_to_mhl_fails) {
	int rc = MHL_USB_NON_INUSE;
	int res = MHL_FAIL;

	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();
	gpio_set_value(GPIO_MHL_SWITCH_SEL_1,0);
	gpio_set_value(GPIO_MHL_SWITCH_SEL_2,0);

	/* exe */	
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	/*	rc = mhl_pf_switch_device_discovery(NULL,0,notify_usb_online); */

	/* 
	 * This API must be called after 
	 * mhl_pf_switch_device_discovery is called
	 * by USB 
	 */
	res = mhl_pf_switch_to_mhl();

	/* verify */
	CHECK_EQUAL(-1, online_);
	LONGS_EQUAL(MHL_FAIL, res);
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}

TEST(MhlPlatformSwitch, 009_PMIC_irq_with_ID_FLOAT) {
	int rc = MHL_USB_NON_INUSE;

	/* setup */
	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();
	gpio_set_value(GPIO_MHL_SWITCH_SEL_1,0);
	gpio_set_value(GPIO_MHL_SWITCH_SEL_2,0);

	/* exe */	
	/* 
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	rc = cb_mhl_pf_switch_device_discovery(NULL, DWC3_ID_FLOAT, notify_usb_online ,&dummy_ctx);

	/* verify */
	CHECK_EQUAL(-1, online_);
	LONGS_EQUAL(DWC3_ID_FLOAT, rc);
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	LONGS_EQUAL(0, gpio_get_value(GPIO_MHL_SWITCH_SEL_2));

	/* tear down */	
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();	
}


/* before notifying usb, the usb/mhl switch must be inclined to usb
and wait 20msec to make it stable. in the data sheet, the time is 500 micro sec.
however, the msleep (20msec) seems to be the plat form limit.
(I fogot where the description is, but it was described in kernel document somewhere) */
TEST(MhlPlatformSwitch, wait_after_switch_to_usb) {

	/* setup */

	/*driver init API. When driver is installed, the API is called by OS.*/
	mhl_pf_switch_init();
	/*
	 * Emulate that the PMIC irq. there is no registerred cb here.
	 */
	cb_mhl_pf_switch_device_discovery = get_registerred_device_discovery_pointer();
	cb_mhl_pf_switch_device_discovery(NULL, 0, mock_notify_usb_online ,&dummy_ctx);


	/* mock setup  */
	common_spy_MockMode();

	mock("GPIO").expectOneCall("gpio_set_value")
		.withParameter("gpio",(int)GPIO_MHL_SWITCH_SEL_1)
			.withParameter("value",(int)0);
	mock("GPIO").expectOneCall("gpio_set_value")
		.withParameter("gpio",(int)GPIO_MHL_SWITCH_SEL_2)
		.withParameter("value",(int)0);
	mock("GPIO").ignoreOtherCalls();

	mock("DELAY").expectOneCall("msleep")
		.withParameter("ms",(int)20);

	mock("NOTIFY_USB_ONLINE").expectOneCall("mock_notify_usb_online");

	/* exe */
	mhl_pf_switch_to_usb();

	/* tear down */
	/*driver exit API. When driver is uninstalled, the API is called by OS.*/
	mhl_pf_switch_exit();
}
