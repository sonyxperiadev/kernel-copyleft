
#include <linux/development-tool/united_file_node.h>

#include <linux/development-tool/united_file_node_register.h>

// ==========================================================================================================================

ssize_t cat_control( void*, struct device_attribute*, char* );

ssize_t echo_control( void*, struct device_attribute*, const char *, size_t );

ssize_t cat_info( void*, struct device_attribute*, char* );

struct united_file_node_data* allocate_memory( void );

bool create_command_string_list( struct show_command_information*, struct command_string* );

void release_memory( struct united_file_node_data* );

void release_command_resource( struct command_info* );

void info_print( int, char*, ... );

// ==========================================================================================================================

int	united_file_node_register( struct platform_device *pdev, struct united_file_node_register_data *register_data )
{

	struct united_file_node_data		*file_node_data;

	if( !register_data->execution_command.list || !register_data->execution_command.count )
	{

		printk( "UFN : The pointer of platform data is NULL\n" );

		return	-ENOMEM;

	}

	if( !pdev->dev.platform_data )
	{

		printk( "UFN : The pointer of platform data is NULL\n" );

		return	-ENOMEM;

	}

	if( !( file_node_data = allocate_memory() ) )
	{

		printk( "UFN : allocate Memory failed\n" );

		return	-ENOMEM;

	}

	if( !create_command_string_list( &register_data->show_command, &file_node_data->string ) )
	{

		printk( "UFN : Create command failed\n" );

		release_memory( file_node_data );

		return	-EINVAL;

	}

	file_node_data->string.parameter_information	= register_data->show_command.parameter_string;

	file_node_data->device_data = pdev->dev.platform_data;

	if( load_command( &file_node_data->command, register_data->execution_command.list, register_data->execution_command.count ) )
	{

		struct control_node_load	file_node;

		file_node.class_name = register_data->class_name, file_node.device_name = register_data->device_name;

		file_node.file_node_data = file_node_data, file_node.control_read = cat_control, file_node.control_write = echo_control, file_node.info_read = cat_info;

		file_node_data->file_node_class	= control_file_node_register( &file_node );

		dev_set_drvdata( &pdev->dev, file_node_data );

		info_print( INFO_BUFFER_INIT, NULL, file_node_data->info_buffer, INFO_BUFFER_SIZE );

		printk( "UFN : driver of file node tool is ready\n" );

		return	0;

	}

	release_memory( file_node_data );

	return	-EINVAL;

}
EXPORT_SYMBOL_GPL( united_file_node_register );

int	united_file_node_unregister( struct platform_device *pdev )
{

	struct united_file_node_data	*tool_data = dev_get_drvdata( &pdev->dev );

	control_file_node_unregister( tool_data->file_node_class );

	release_command_resource( &tool_data->command );

	release_memory( tool_data );

	return	0;

}
EXPORT_SYMBOL_GPL( united_file_node_unregister );
