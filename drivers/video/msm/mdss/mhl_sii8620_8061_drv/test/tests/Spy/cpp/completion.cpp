extern "C" {
#include "linux/completion.h"
}

void complete(struct completion *)
{
	return;
}

long wait_for_completion_interruptible_timeout(
	struct completion *x, unsigned long timeout)
{
	return 0;
}

void init_completion(struct completion *x)
{
	x->done = 0;
	/*init_waitqueue_head(&x->wait);*/
}