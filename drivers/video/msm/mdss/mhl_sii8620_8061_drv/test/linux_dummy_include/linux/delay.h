#ifndef __LINUX_DELAY_H
#define __LINUX_DELAY_H

#include <stdio.h>
#include <unistd.h>
#include "completion.h"


#define HZ 100
/* #define msleep(_x) usleep(_x*1000) */


struct timer_list {
	/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
		unsigned long expires;
			void (*function)(unsigned long);
				unsigned long data;
					/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};

#define jiffies 10000

#define msecs_to_jiffies(_time) 0

void init_timer(struct timer_list *timer);
int del_timer(struct timer_list *timer);
int mod_timer(struct timer_list *timer, unsigned long expires);

#endif /* __LINUX_DELAY_H */
