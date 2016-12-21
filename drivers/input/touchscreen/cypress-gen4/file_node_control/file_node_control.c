
#include <linux/development-tool/united_file_node_register.h>

#include "file_node_function.h"

static int file_node_control_probe( struct platform_device *pdev )
{

	struct load_command_data	load_command[] = { LOAD_COMMAND_DATA };

	struct execution_command_information	execution_command = { load_command, ARRAY_COUNT( load_command ) };

	struct command_information	command_strings[] = { COMMAND_INFORMATION };

	struct show_command_information		show_command = { command_strings, ARRAY_COUNT( command_strings ), PARAMETER_STRING };

	struct united_file_node_register_data	file_node_register_data = { "touch", "cyttsp4", execution_command, show_command };

	return	united_file_node_register( pdev, &file_node_register_data );

}

static int file_node_control_remove( struct platform_device *pdev )
{

	return	united_file_node_unregister( pdev );

}

static struct platform_driver	file_node_control_device_driver =
{
	.probe		= file_node_control_probe,
	.remove		= file_node_control_remove,
	.driver		=
	{
		.name	= "fih_file_node_control",
		.owner	= THIS_MODULE,
	}
};

static int __init	file_node_control_init( void )
{

	return	platform_driver_register( &file_node_control_device_driver );

}

static void __exit	file_node_control_exit( void )
{

	platform_driver_unregister( &file_node_control_device_driver );

}

// ==========================================================================================================================

late_initcall( file_node_control_init );

module_exit( file_node_control_exit );

MODULE_AUTHOR( "Y.S Chang <yschang@fih-foxconn.com>" );

MODULE_DESCRIPTION( "FIH file node control" );

MODULE_LICENSE( "GPL v2" );

MODULE_ALIAS( "platform:control" );
