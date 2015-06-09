
#include <linux/development-tool/development_tool.h>

#include <linux/development-tool/united_file_node_register.h>

#include <linux/development-tool/united_file_node.h>

#include <linux/development-tool/functions.h>

// ==========================================================================================================================

struct united_file_node_data*	allocate_memory( void )
{

	struct united_file_node_data	*tool_data;

	if( ( tool_data = kzalloc( sizeof( struct united_file_node_data ), GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : Get memory of tool data failed\n" );

		return	NULL;

	}

	if( ( tool_data->info_buffer = kzalloc( INFO_BUFFER_SIZE, GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : Get memory of infomation buffer failed\n" );

		kfree( tool_data );

		return	NULL;

	}

	if( ( tool_data->parameter_buffer = kzalloc( PARA_BUFFER_SIZE, GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : Get memory of parameter buffer failed\n" );

		kfree( tool_data->info_buffer );

		kfree( tool_data );

		return	NULL;

	}

	if( ( tool_data->return_buffer = kzalloc( RETURN_BUFFER_SIZE, GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : Get memory of return-buffer failed\n" );

		kfree( tool_data->parameter_buffer );

		kfree( tool_data->info_buffer );

		kfree( tool_data );

		return	NULL;

	}



	return	tool_data;

}

void	release_memory( struct united_file_node_data *tool_data )
{

	kfree( tool_data->return_buffer );

	kfree( tool_data->parameter_buffer );

	kfree( tool_data->info_buffer );

	kfree( tool_data );

}

bool	create_command_string_list( struct show_command_information *command_string_list, struct command_string *string_info )
{

	struct command_information	*file_node_command = command_string_list->list;

	struct BST_data	*sort_buffer;

	struct BS_data	*sort_temp;

	struct BST_info	sort_info;

	int	loop;

	if( ( sort_buffer = kzalloc( sizeof( struct BST_data ) * command_string_list->count, GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : no memory for sort buffer of command string\n" );

		return	false;

	}

	if( ( sort_temp = kzalloc( sizeof( struct BS_data ) * command_string_list->count, GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : no memory for sort buffer of command string\n" );

		kfree( sort_buffer );

		return	false;

	}

	if( ( string_info->list = kzalloc( sizeof( struct command_information ) * command_string_list->count, GFP_KERNEL ) ) == NULL )
	{

		printk( "UFN : no memory for data of command string\n" );

		kfree( sort_buffer );

		kfree( sort_temp );

		return	false;

	}

	BST_init( &sort_info );

	for( loop = 0 ; loop < command_string_list->count ; ++loop )
	{

		struct command_information	*info = file_node_command + loop;

		struct BST_data			*sort_data = sort_buffer + loop;

		if( !info->string )
		{

			printk( "UFN : The string of command ID(%d) is NULL\n", info->command_id );

			continue;

		}

		sort_data->index = info->command_id, sort_data->data = info->string;

		BST_add( &sort_info, sort_data );

	}

	string_info->count	= BST_sort( &sort_info, sort_temp, command_string_list->count );

	kfree( sort_buffer );

	if( !string_info->count )
	{

		printk( "UFN : Sort command failed\n" );

		kfree( string_info->list );

		kfree( sort_temp );

		return	false;

	}

	for( loop = 0 ; loop < string_info->count ; ++loop )
	{

		struct command_information	*info = string_info->list + loop;

		struct BS_data			*sort_data = sort_temp + loop;

		info->string = sort_data->data, info->command_id = sort_data->index;

	}

	kfree( sort_temp );

	return	true;

}