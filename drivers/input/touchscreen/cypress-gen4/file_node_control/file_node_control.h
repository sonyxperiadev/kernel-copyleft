
#pragma once

#include <linux/platform_device.h>

// ==========================================================================================================================

#ifdef	CONFIG_FIH_FILE_NODE_CONTROL
	#define	REGISTER_FILE_NODE_CONTROL_DRIVER( DATA ) \
	{ \
		static struct platform_device tool; \
		tool.name = "fih_file_node_control", tool.id = -1; \
		tool.dev.platform_data = DATA; \
		platform_device_register( &tool ); \
		printk( "TTUCH : Register driver of file node control\n" ); \
	}
#else
	#define	REGISTER_FILE_NODE_CONTROL_DRIVER( DATA, READ_FUNCTION, WRITE_FUNCTION )
#endif
