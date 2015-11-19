
#include <linux/development-tool/command_maker.h>

int	load_command( struct command_info *store_command, struct load_command_data *load_data, unsigned int count )
{

	struct BST_data	*sort_buffer;

	struct BST_info	sort_info;

	int	loop, return_value;

	return_value	= 1;

	if( !count )
	{

		printk( "CMK : Command count is zero\n" );

		return	0;
	}

	if( ( store_command->commmand_buffer = kzalloc( sizeof( struct BS_data ) * count, GFP_KERNEL ) ) == NULL )
	{

		printk( "CMK : no memory for command buffer\n" );

		return	0;

	}

	if( ( sort_buffer = kzalloc( sizeof( struct BST_data ) * count, GFP_KERNEL ) ) == NULL )
	{

		printk( "CMK : no memory for sort buffer of command\n" );

		kfree( store_command->commmand_buffer );

		return	0;

	}

	if( ( store_command->command_data_memory = kzalloc( sizeof( struct command_data ) * count, GFP_KERNEL ) ) == NULL )
	{

		printk( "CMK : no memory for data of command\n" );

		kfree( store_command->commmand_buffer );

		kfree( sort_buffer );

		return	0;

	}

	BST_init( &sort_info );

	for( loop = 0 ; loop < count ; ++loop )
	{

		struct load_command_data	*command = load_data + loop;

		struct command_data		*store_command_data = store_command->command_data_memory + loop;

		struct BST_data			*sort_data = sort_buffer + loop;

		if( !command->function )
		{

			printk( "CMK : Function pointer of command ID(%d) is NULL\n", command->command_id );

			continue;

		}

		store_command_data->function = command->function, store_command_data->parameter_string = command->parameter_string, store_command_data->parameter_count = command->parameter_count, store_command_data->other = command->other;

		sort_data->index = command->command_id, sort_data->data = store_command_data;

		BST_add( &sort_info, sort_data );

	}

	if( !( store_command->count = BST_sort( &sort_info, store_command->commmand_buffer, count ) ) )
	{

		printk( "CMK : Sort command failed\n" );

		kfree( store_command->command_data_memory );

		kfree( store_command->commmand_buffer );

		return_value	= 0;

	}

	kfree( sort_buffer );

	return	return_value;

}
EXPORT_SYMBOL_GPL( load_command );

void	release_command_resource( struct command_info *store_command )
{

	kfree( store_command->command_data_memory );

	kfree( store_command->commmand_buffer );

}
EXPORT_SYMBOL_GPL( release_command_resource );
