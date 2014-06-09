#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include "mhl_common.h"
#include "mhl_platform.h"
#include "mhl_sii8620_8061_device.h"

static irqreturn_t mhl_irq_handler(int irq, void *data)
{
	pr_debug("%s:irq : %d\n", __func__, irq);
	mhl_device_isr ();
	return  IRQ_HANDLED;
}

int mhl_tx_initialize(void)
{
	int ret = -1;
	pr_debug("%s()\n", __func__);

	ret = request_threaded_irq(mhl_pf_get_irq_number(), NULL,
								mhl_irq_handler,
								IRQF_TRIGGER_LOW | IRQF_ONESHOT,
								mhl_pf_get_device_name(),
								NULL);
	if (ret < 0) {
		pr_err("%s:request_threaded_irq failed, status: %d\n", __func__, ret);
	}

	/* mhl_sii8620_device_start() is necessary to carry
	it out after request_threaded_irq().
	Because it can't detect the RGND interrupt
	in the "mhl_irq_handler()"
	if it is registered before request_thread_irq. */
	mhl_pf_switch_register_cb(
		mhl_sii8620_device_start, NULL);

	return ret;
}

void mhl_tx_release(void)
{
	pr_debug("%s()\n", __func__);
	free_irq(mhl_pf_get_irq_number(), NULL);
}
