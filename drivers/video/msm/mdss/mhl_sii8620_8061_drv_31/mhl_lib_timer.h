#ifndef __MHL_LIB_TIMER_H__
#define __MHL_LIB_TIMER_H_


int mhl_lib_timer_init(void);
int mhl_lib_timer_create(void (*callback_handler)(void *callback_param),
			void *callback_param,
			void **timer_handle);
int mhl_lib_timer_delete(void **timer_handle);
int mhl_lib_timer_start(void *timer_handle, uint32_t time_msec);
int mhl_lib_timer_stop(void *timer_handle);
void mhl_lib_timer_release(void);

#endif
