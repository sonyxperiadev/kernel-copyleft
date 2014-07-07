#ifndef __DUMMY_UTIL_SPY_H
#define __DUMMY_UTIL_SPY_H

#include "mhl_platform.h"

void common_spy_MockMode(void);
bool common_spy_get_MockMode(void);
void common_spy_DisableMockMode(void);

void common_spy_clean(void);

int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);

#endif
