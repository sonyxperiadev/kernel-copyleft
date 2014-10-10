
#ifndef MOCK_REG_ACCESS_H_
#define MOCK_REG_ACCESS_H_

#include <linux/gpio.h>
#include "mhl_platform.h"
#include "si_8620_regs.h"

extern void mhl_pf_clock_disable(void);
int MOCK_mhl_pf_i2c_print(bool _isPrint);

#endif /* MOCK_REG_ACCESS_H_ */

