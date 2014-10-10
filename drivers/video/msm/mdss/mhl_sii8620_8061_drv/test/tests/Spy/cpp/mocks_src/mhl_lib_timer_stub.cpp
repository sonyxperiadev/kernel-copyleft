#include "CppUTestExt/MockSupport.h"

extern "C" {
#include "mhl_lib_timer.h"
}

int mhl_lib_timer_init(void)
{
	return 0;
}
int mhl_lib_timer_create(void (*callback_handler)(void *callback_param),
			void *callback_param,
			void **timer_handle)
{
	return 0;
}
int mhl_lib_timer_delete(void **timer_handle)
{
	return 0;
}
int mhl_lib_timer_start(void *timer_handle, uint32_t time_msec)
{
	return 0;
}
int mhl_lib_timer_stop(void *timer_handle)
{
	return 0;
}
void mhl_lib_timer_release(void)
{
	return;
}
