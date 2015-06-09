
#include <linux/delay.h>

#include <linux/development-tool/united_file_node.h>

#include <linux/development-tool/functions.h>

// ==========================================================================================================================

void	no_print( int command, char *string, ... )
{
}

void	log_print( int command, char *string, ... )
{

	char	buffer[ 256 ];

	va_list	vaList;

	if( command == INFO_BUFFER_RESET )
		return;

	va_start( vaList, string );

	vsnprintf( buffer, sizeof( buffer ), string, vaList );

	va_end( vaList );

	printk( "UFN : %s", buffer );

}

void	info_print( int command, char *string, ... )
{

	static char		*buffer = 0;

	static unsigned int	pointer = 0;

	static size_t		size = 0;

	va_list	vaList;

	va_start( vaList, string );

	switch( command )
	{

		case	INFO_BUFFER_RESET :

			pointer = 0;

			break;

		case	INFO_BUFFER_INIT :

			buffer	= va_arg( vaList, char* ), size = va_arg( vaList, unsigned int );

			break;

		case	INFO_BUFFER_ADD :

			if( !buffer || !size )
			{

				printk( "UFN : info_print() needs initialization\n" );

				break;

			}

			if( pointer >= size )
			{

				printk( "UFN : Out of range, range[Max:Using]=[%d:%d]\n", size, pointer );

				break;

			}

			{

				int	count = vsnprintf( buffer + pointer, size - pointer, string, vaList );

				if( count < 0 )
					printk( "UFN : Info buffer is full\n" );
				else
					pointer += count;

			}

			break;

		default :
			break;

	}

	va_end( vaList );

}

bool	add_return_data( char *return_buffer, char *parameter_string, ... )
{

	va_list	vaList;

	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )return_buffer;

	unsigned	parameter_index = 0, store_index = 0, parameter_length = strlen( parameter_string );

	void	*store_array = ( void* )( return_buffer + sizeof( struct file_node_return_struct ) );

	bool	return_status = true;

	va_start( vaList, parameter_string );

	while( parameter_length-- )
	{

		char	parameter = *( parameter_string + parameter_index++ );

		if( parameter == ' ' || parameter == 0x9 )
			continue;

		if( parameter == '%' && parameter_length )
		{

			switch( *( parameter_string + parameter_index++ ) )
			{

				case	'c' :
				{

					unsigned	value = va_arg( vaList, unsigned );

					char	*store = ( char* )store_array + store_index;

					if( store_index >= PAGE_SIZE )
						return_struct->error_code = ERROR_OUT_OF_RANGE, return_status = false;
					else
						*store = value, ++store_index;

					break;

				}

				case	'x' :
				{

					unsigned	value = va_arg( vaList, unsigned );

					unsigned	*store = ( unsigned* )( ( char* )store_array + store_index );

					if( store_index + sizeof( unsigned ) >= PAGE_SIZE )
						return_struct->error_code = ERROR_OUT_OF_RANGE, return_status = false;
					else
						*store = value, store_index += sizeof( unsigned );

					break;

				}

				case	's' :
				{

					char	*string = va_arg( vaList, char* );

					char	*store = ( char* )store_array + store_index;

					unsigned	string_length = strlen( string );

					if( string_length + 1 + store_index >= PAGE_SIZE )
						return_struct->error_code = ERROR_OUT_OF_RANGE, return_status = false;
					else
						memcpy( store, string, string_length + 1 ), store_index += string_length + 1;

					break;

				}

				default :

					printk( "UFN : Unknow parameter\n" );

					return_struct->error_code = ERROR_INVALID_PARAMETER, return_status = false;

					break;

			}

		}
		else
			return_status	= false;

		if( return_status == false )
			break;

	}

	va_end( vaList );

	return	return_status;

}