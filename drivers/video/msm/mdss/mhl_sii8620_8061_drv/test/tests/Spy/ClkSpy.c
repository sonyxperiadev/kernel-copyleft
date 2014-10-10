#include "ClkSpy.h"


struct clk *clk_get(struct device *dev, const char *id){
		return (struct clk*)malloc(sizeof(struct clk));
}


int clk_enable(struct clk *clk){
	return 0;
}

void clk_disable(struct clk *clk){

}
void clk_put(struct clk *clk){
	free((void*)clk);
}



int clk_prepare(struct clk *clk)
{
	return 0;
}


