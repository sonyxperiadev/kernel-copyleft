
#include <linux/development-tool/united_file_node.h>

#include <linux/development-tool/functions.h>

// ==========================================================================================================================

void info_print( int command, char *string, ... );

// ==========================================================================================================================

ssize_t	cat_control( void *node_data, struct device_attribute *attribute, char *buffer )
{

	struct command_string	*string = &( ( struct united_file_node_data* )node_data )->string;

	unsigned int	index = 0, command_start = 0;

	bool	show_range = false;

	info_print( INFO_BUFFER_RESET, NULL );

	if( string->parameter_information )
		info_print( INFO_BUFFER_ADD, "%s", string->parameter_information );

	while( index < string->count )
	{

		struct command_information	*info = string->list + index++;

		while( info->command_id > command_start + COMMAND_GROUP_RANGE - 1 )
		{

			command_start += COMMAND_GROUP_RANGE;

			show_range	= false;

		}

		if( !show_range )
		{

			info_print( INFO_BUFFER_ADD, "\nCMD[%02d:%02d]\n", command_start, command_start + COMMAND_GROUP_RANGE - 1 );

			show_range	= true;

		}

		info_print( INFO_BUFFER_ADD, info->string, info->command_id );

	}

	info_print( INFO_BUFFER_ADD, "\n", NULL );

	return	snprintf( buffer, PAGE_SIZE, ( ( struct united_file_node_data* )node_data )->info_buffer );

}

// ==========================================================================================================================

ssize_t	echo_control( void *node_data, struct device_attribute *attribute, const char *buffer, size_t size )
{

	struct united_file_node_data	*file_node_data = node_data;
	
	char	*parameter_buffer = file_node_data->parameter_buffer;

	struct	command_info	*command = &file_node_data->command;

	unsigned int	pointer = 0, para_buffer_pointer = 0;

	int	rem_size = size;

	while( pointer < rem_size )
	{

		struct BS_data	*search_data;

		struct command_data	*valid_command;

		unsigned int	char_count;

		int	user_command;

		info_print( INFO_BUFFER_RESET, NULL );

		if( !get_para_from_buffer( buffer + pointer, "%d", &user_command ) )
		{

			printk( "UFN : Can't get command from buffer\n" );

			info_print( INFO_BUFFER_ADD, "Can't get command from buffer\n" );

			break;

		}

		if( !( search_data = binary_search( command->commmand_buffer, command->count, user_command ) ) )
		{

			printk( "UFN : Can't support command(%d)\n", user_command );

			info_print( INFO_BUFFER_ADD, "Can't support command(%d)\n", user_command );

			break;

		}

		valid_command	= ( struct command_data* )search_data->data;

		if( !( char_count = get_para_char_count( buffer + pointer, buffer + size, ONE_PARAMETER ) ) )
		{

			printk( "UFN : Can't get charactor count.\n" );

			info_print( INFO_BUFFER_ADD, "Can't get charactor count.\n" );

			break;

		}

		pointer += char_count;

		printk( "UFN : [CMD:PARA]=[%d:%d]\n", user_command, valid_command->parameter_count );

		{

			struct command_parameter	parameter;

			struct parameter_info	*parameter_pointer[] =
			{

				&parameter.para1, &parameter.para2, &parameter.para3, &parameter.para4, &parameter.para5,
				&parameter.para6, &parameter.para7, &parameter.para8, &parameter.para9, &parameter.para10,
				&parameter.para11, &parameter.para12, &parameter.para13, &parameter.para14, &parameter.para15,
				&parameter.para16, &parameter.para17, &parameter.para18, &parameter.para19, &parameter.para20

			};

			char	para_string[ THREE_CHARATOR ] = { 0 };

			unsigned int	para_count = 0, parameter_string_lenght = 0;

			int		command_parameter_count = valid_command->parameter_count;

			unsigned int	loop;

			if( command_parameter_count == COMMAND_N_PARAMENTER )
				command_parameter_count	= sizeof( parameter_pointer ) / sizeof( *parameter_pointer );

			if( command_parameter_count > sizeof( parameter_pointer ) / sizeof( *parameter_pointer ) )
			{

				printk( "UFN : Out of parameter array\n" );

				info_print( INFO_BUFFER_ADD, "Out of parameter array\n" );

				break;

			}

			if( command_parameter_count && !valid_command->parameter_string )
			{

				printk( "UFN : Parameter count(%d), but no parameter string\n", command_parameter_count );

				info_print( INFO_BUFFER_ADD, "Parameter count(%d), but no parameter string\n", command_parameter_count );

				break;

			}

			if( valid_command->parameter_string )
				parameter_string_lenght	= strlen( valid_command->parameter_string ) + 1;

			for( loop = 0 ; loop < command_parameter_count ; ++loop )
			{

				struct parameter_info	*para_info = *( parameter_pointer + loop );

				unsigned int	return_value;

				memcpy( para_string, valid_command->parameter_string + para_count, TWO_CHARATOR );

				para_count += get_para_char_count( valid_command->parameter_string + para_count, valid_command->parameter_string + parameter_string_lenght, ONE_PARAMETER );

				char_count	= get_para_char_count( buffer + pointer, buffer + size, ONE_PARAMETER );

				if( !strcmp( para_string, "%s" ) )
				{

					if( para_buffer_pointer >= PARA_BUFFER_SIZE || para_buffer_pointer + char_count >= PARA_BUFFER_SIZE )
					{

						printk( "UFN : Parameter buffer is full\n" );

						info_print( INFO_BUFFER_ADD, "Parameter buffer is full\n" );

						pointer	= rem_size;

						break;

					}

					para_info->kind	= PARAMETER_KIND_STRING;

					return_value	= get_para_from_buffer( buffer + pointer, para_string, parameter_buffer + para_buffer_pointer );

					para_info->container.string = parameter_buffer + para_buffer_pointer;

					para_buffer_pointer += char_count; 

				}
				else
				{

					para_info->kind	= PARAMETER_KIND_NUMBER;

					return_value	= get_para_from_buffer( buffer + pointer, para_string, &para_info->container.number );

				}

				if( !return_value )
				{

					if( valid_command->parameter_count == COMMAND_N_PARAMENTER )
					{

						printk( "UFN : Parameter count(%d)\n", loop );

						info_print( INFO_BUFFER_ADD, "Parameter count(%d)\n", loop );

						break;

					}

					printk( "UFN : Get parameter failed\n" );

					info_print( INFO_BUFFER_ADD, "Get parameter failed\n" );

					pointer	= rem_size;

					break;

				}

				if( !char_count )
				{

					printk( "UFN : Can't get charactor count..\n" );

					info_print( INFO_BUFFER_ADD, "Can't get charactor count..\n" );

					pointer	= rem_size;

					break;

				}

				pointer += char_count;

			}

			if( valid_command->parameter_count == COMMAND_N_PARAMENTER || loop == command_parameter_count )
			{

				struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )file_node_data->return_buffer;

				return_struct->backup_error_code	= return_struct->error_code;

				return_struct->error_code = ERROR_NONE, memcpy( return_struct->check_id, RETURN_STRUCT_STRING, sizeof( RETURN_STRUCT_STRING ) );

				valid_command->function( node_data, &parameter );

				return_struct->command_id	= user_command;

			}

		}

	}

	return size;

}

// ==========================================================================================================================

ssize_t	cat_info( void *node_data, struct device_attribute *attribute, char *buffer )
{

	struct united_file_node_data	*file_node_data = node_data;

	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )file_node_data->return_buffer;

	struct command_info	*command = &file_node_data->command;

	struct BS_data	*search_data;

	struct command_data	*valid_command;

	if( !( search_data = binary_search( command->commmand_buffer, command->count, return_struct->command_id ) ) )
	{

		printk( "UFN : Can't found command(%d)\n", return_struct->command_id );

		info_print( INFO_BUFFER_ADD, "Can't found command(%d)\n", return_struct->command_id );

		return_struct->error_code = ERROR_INVALID_DATA, memcpy( buffer, file_node_data->return_buffer, sizeof( struct file_node_return_struct ) );

		return	sizeof( struct file_node_return_struct );

	}

	valid_command	= ( struct command_data* )search_data->data;

	if( !valid_command->other )
		return	snprintf( buffer, PAGE_SIZE, file_node_data->info_buffer );

	{

		unsigned	total_size = return_struct->count + sizeof( struct file_node_return_struct );

		if( total_size > PAGE_SIZE )
			return	0;

		memcpy( buffer, file_node_data->return_buffer, total_size );

		return	total_size;

	}

}
