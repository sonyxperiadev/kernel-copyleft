#ifndef __LINUX_CLK_H
#define __LINUX_CLK_H

#include <linux/platform_device.h>

struct clk {
	unsigned long rate;
};
struct clk *clk_get(struct device *dev, const char *id);
int clk_enable(struct clk *clk);
void clk_disable(struct clk *clk);
void clk_put(struct clk *clk);
int clk_prepare(struct clk *clk);




#endif /* __LINUX_CLK_H */

