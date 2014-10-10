#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
extern "C" {
#include "mhl_platform.h"
#include "util.h"
extern int platform_write_i2c_block(struct i2c_adapter *i2c_bus,
				 u8 page,
				 u8 offset,
				 u16 count,
				 u8 *values);
extern int platform_read_i2c_block(struct i2c_adapter *i2c_bus
								, u8 page
								, u8 offset
								, u8 count
								, u8 *values);
}
struct i2c_adapter i2c_bus;

TEST_GROUP(MhlPlatform) {
	void setup() {
		mhl_pf_i2c_init(&i2c_bus);
	}
	void teardown() {
		mock("MHL_PLATFORM_CHIP").checkExpectations();
		mock("I2C").checkExpectations();
		mock().clear();
	}
};

/*
 * I2C Write test
 */
/* test for platform_write_i2c_block() */
TEST(MhlPlatform, platform_write_i2c_block__not_write_to_i2c_when_mhl_chip_is_off) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_off = 0;

	bool is_chip_on = false;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_off);

	platform_write_i2c_block(&i2c_bus,page,offset,count,&values);
}

TEST(MhlPlatform, platform_write_i2c_block__write_to_i2c_when_mhl_chip_is_on) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_on = 1;
	int i2c_success = 1;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_on);
	mock("I2C").expectOneCall("i2c_transfer")
			.andReturnValue(i2c_success);
	mock().ignoreOtherCalls();

	platform_write_i2c_block(&i2c_bus,page,offset,count,&values);
}

/* test for mhl_pf_write_reg_block() */
TEST(MhlPlatform, mhl_pf_write_reg_block__error_when_mhl_chip_is_off) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_off = 0;

	bool is_chip_on = false;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_off);

	LONGS_EQUAL(-MHL_I2C_NOT_AVAILABLE, mhl_pf_write_reg_block(page, offset, count, &values));
}

TEST(MhlPlatform, mhl_pf_write_reg_block__no_error_when_mhl_chip_is_on) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_on = 1;
	int i2c_success = 1;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_on);
	mock("I2C").expectOneCall("i2c_transfer")
			.andReturnValue(i2c_success);
	mock().ignoreOtherCalls();

	LONGS_EQUAL(0, mhl_pf_write_reg_block(page, offset, count, &values));
}


/* test for mhl_pf_write_reg() */
TEST(MhlPlatform, mhl_pf_write_reg__no_error_with__mhl_chip_is_on) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_on = 1;
	int i2c_success = 1;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_on);
	mock("I2C").expectOneCall("i2c_transfer")
			.andReturnValue(i2c_success);
	mock().ignoreOtherCalls();

	LONGS_EQUAL(0, mhl_pf_write_reg(page, offset, values));
}

TEST(MhlPlatform, mhl_pf_write_reg__error_with_mhl_chip_is_off) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_off = 0;
	bool is_chip_on = false;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_off);

	LONGS_EQUAL(-MHL_I2C_NOT_AVAILABLE, mhl_pf_write_reg(page, offset, values));
}






/*
 * I2C Read test
 */
/* test for platform_read_i2c_block() */
TEST(MhlPlatform, platform_read_i2c_block__not_read_from_i2c_when_mhl_chip_is_off) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_off = 0;

	bool is_chip_on = false;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_off);

	platform_read_i2c_block(&i2c_bus,page,offset,count,&values);
}

TEST(MhlPlatform, platform_read_i2c_block__read_from_i2c_when_mhl_chip_is_on) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_on = 1;
	int i2c_read_success = 2;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_on);
	mock("I2C").expectOneCall("i2c_transfer")
			.andReturnValue(i2c_read_success);
	mock().ignoreOtherCalls();

	platform_read_i2c_block(&i2c_bus,page,offset,count,&values);
}

/* test for mhl_pf_read_reg_block */
TEST(MhlPlatform, mhl_pf_read_reg_block__error_when_mhl_chip_is_off) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_off = 0;
	bool is_chip_on = false;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_off);

	LONGS_EQUAL(-MHL_I2C_NOT_AVAILABLE, mhl_pf_read_reg_block(page, offset, count, &values));
}

TEST(MhlPlatform, mhl_pf_read_reg_block__no_error_when_mhl_chip_is_on) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_on = 1;
	int i2c_read_success = 2;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_on);
	mock("I2C").expectOneCall("i2c_transfer")
			.andReturnValue(i2c_read_success);
	mock().ignoreOtherCalls();

	LONGS_EQUAL(0, mhl_pf_read_reg_block(page, offset, count, &values));
}

/* test for mhl_pf_read_reg */
TEST(MhlPlatform, mhl_pf_read_reg__error_when_mhl_chip_is_off) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	u16 count = 1;
	u8 values = 100;
	int chip_power_off = 0;
	bool is_chip_on = false;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_off);

	LONGS_EQUAL(-MHL_I2C_NOT_AVAILABLE, mhl_pf_read_reg(page, offset));
}

TEST(MhlPlatform, mhl_pf_read_reg__no_error_when_mhl_chip_is_on) {
	struct i2c_adapter i2c_bus;
	u8 page = 0;
	u8 offset = 0x50;
	int chip_power_on = 1;
	int i2c_read_success = 2;

	common_spy_MockMode();

	mock("MHL_PLATFORM_CHIP").expectOneCall("mhl_pf_is_chip_power_on")
			.andReturnValue(chip_power_on);
	mock("I2C").expectOneCall("i2c_transfer")
			.andReturnValue(i2c_read_success);
	mock().ignoreOtherCalls();

	LONGS_EQUAL(1, (-MHL_I2C_NOT_AVAILABLE != mhl_pf_read_reg(page, offset)));
}