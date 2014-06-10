#include "CppUTestExt/MockSupport.h"

extern "C"{
#include "MockRegAccess.h"
#include "string.h"
}

static bool isPrint = false;

int MOCK_mhl_pf_i2c_print(bool _isPrint)
{
	isPrint = _isPrint;
}

int mhl_pf_write_reg_block(u8 page, u8 offset,
							  u16 count, u8 *values)
{
	mock("IO")
		.actualCall("mhl_pf_write_reg_block")
		.withParameter("page", (u8)page)
		.withParameter("offset", (u8)offset)
		.withParameter("count", (u16)count)
		.withParameter("values", (u8 *)values);
/*		.withParameterOfType("unsigned char *","values",values);*/

	return 0;
}

int mhl_pf_read_reg_block(u8 page, u8 offset,
							  u8 count, u8 *values)
{
int i;
void *res = mock("IO")
	.actualCall("mhl_pf_read_reg_block")
	.withParameter("page", (u8)page)
	.withParameter("offset", (u8)offset)
	.withParameter("count", (u8)count)
	.returnValue().getPointerValue();

	memcpy((void *)values, (const void *)res, count);

	return 0;
}


int mhl_pf_write_reg(u8 page, u8 offset, u8 value)
{
	mock("IO")
	.actualCall("mhl_pf_write_reg")
	.withParameter("page", (u8)page)
	.withParameter("offset", (u8)offset)
	.withParameter("value", (u8)value);
	if (isPrint) {
		printf("I2C %x.%x W %x\n",page,offset,value);
		printf("I2C %d.%d W %d\n",page,offset,value);
	}

	return 0;
}

int mhl_pf_read_reg(u8 page, u8 offset)
{

int res = mock("IO")
	.actualCall("mhl_pf_read_reg")
	.withParameter("page", (u8)page)
	.withParameter("offset", (u8)offset)
	.returnValue().getIntValue();

	if (isPrint) {
		printf("I2C %x.%x R %x\n",page,offset,res);
		printf("I2C %d.%d R %d\n",page,offset,res);
	}

	return res;
}

int mhl_pf_modify_reg(u8 page, u8 offset,
					  u8 mask, u8 value)
{
#if 0
	int	reg_value;
	int	write_status;

	reg_value = (u8)mhl_pf_read_reg(page, offset);
	if (reg_value < 0)
		return reg_value;

	reg_value &= ~mask;
	reg_value |= mask & value;

	write_status = mhl_pf_write_reg(page, offset, (u8)reg_value);

	if (write_status < 0)
		return write_status;
	else
		return reg_value;
#endif
	 mock("IO")
		.actualCall("mhl_pf_modify_reg")
		.withParameter("page", page)
		.withParameter("offset", offset)
		.withParameter("mask", mask)
		.withParameter("value", value);
	return 0;
}

/*
void gpio_set_value(unsigned int gpio, int value)
{
	mock("IO")
	.actualCall("gpio_set_value")
	.withParameter("gpio", (u8)gpio)
	.withParameter("value", (int)value);

}

int gpio_get_value(unsigned gpio){
	return mock("IO")
	.actualCall("gpio_get_value")
	.withParameter("gpio", (u8)gpio)
	.returnValue().getIntValue();
}

void clk_disable(struct clk* mhl_clk)
{
	mock("IO")
	.actualCall("clk_disable");
}
*/
