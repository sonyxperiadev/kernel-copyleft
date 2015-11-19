
#include <linux/development-tool/development_tool.h>

#define	CHARATOR_SPACE		0x20

#define	CHARATOR_NEXT_LINE	0xa

struct	control_node_data
{

	ssize_t ( *control_read )( void *data, struct device_attribute*, char* );

	ssize_t ( *control_write )( void *data, struct device_attribute*, const char*, size_t );

	void	*data;

};

ssize_t	empty_control_read( void *data, struct device_attribute *attr, char *buf )
{

	return	snprintf( buf, PAGE_SIZE, "Does not support this function\n" );

}

ssize_t empty_control_write( void *data, struct device_attribute *attr, const char *buf, size_t size )
{

	return	size;

}

ssize_t empty_info_read( void *data, struct device_attribute *attr, char *buf )
{

	return	snprintf( buf, PAGE_SIZE, "Does not support this function\n" );

}

ssize_t	control_file_node_read( struct device *dev, struct device_attribute *attr, char *buf )
{

	struct control_node_data	*internal_data	= dev_get_drvdata( dev );

	return	internal_data->control_read( internal_data->data, attr, buf );

}

ssize_t control_file_node_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{

	struct control_node_data	*internal_data	= dev_get_drvdata( dev );

	return	internal_data->control_write( internal_data->data, attr, buf, size );

}

static DEVICE_ATTR( control, S_IRUGO | S_IWUSR, control_file_node_read, control_file_node_write );

struct control_class_device*	control_file_node_register( struct control_node_load *load )
{

	struct control_class_device	*return_data;

	struct control_node_data		*file_node_data;

	return_data	= kzalloc( sizeof( struct control_class_device ), GFP_KERNEL );

	file_node_data	= kzalloc( sizeof( struct control_node_data ), GFP_KERNEL );

	if( !return_data || !file_node_data )
	{
		kfree( return_data );
		kfree( file_node_data );
		printk( KERN_ERR "DEV_T : Class %s, Device %s, Memory allocat failed\n", load->class_name, load->device_name );
		return	0;

	}

	return_data->Class		= class_create( THIS_MODULE, load->class_name );

	if( IS_ERR( return_data->Class ) )
	{
		printk( KERN_ERR "DEV_T : calss_create() failed\n" );
		return	0;
	}

	return_data->Device		= device_create( return_data->Class, NULL, MKDEV( 0, 0 ), NULL, load->device_name );

	if( IS_ERR( return_data->Device ) )
	{
		printk( KERN_ERR "DEV_T : device_create() failed\n" );
		return	0;
	}

	file_node_data->control_read	= load->control_read ? load->control_read : empty_control_read;

	file_node_data->control_write	= load->control_write ? load->control_write : empty_control_write;

	file_node_data->data		= load->file_node_data;

	dev_set_drvdata( return_data->Device, file_node_data );

	if( device_create_file( return_data->Device, &dev_attr_control ) )
		printk( KERN_ERR "DEV_T : Class %s, Device %s, device_create_file( control ) failed\n", load->class_name, load->device_name );

	return	0;
}
EXPORT_SYMBOL_GPL( control_file_node_register );

void control_file_node_unregister( struct control_class_device *unload )
{

	if( !unload )
		return;

	device_remove_file( unload->Device, &dev_attr_control );

	device_destroy( unload->Class, MKDEV( 0, 0 ) );

	dev_set_drvdata( unload->Device, NULL );

	class_destroy( unload->Class );

}
EXPORT_SYMBOL_GPL( control_file_node_unregister );

unsigned int	get_para_char_count( const char *buffer, const char *valid_addr, unsigned int para_count )
{

	int	loop, pointer;

	char	get_charactor;

	if( !buffer || !para_count || buffer >= valid_addr )
		return	0;

	pointer	= -1;

	for( loop = 0 ; loop < para_count ; ++loop )
	{

		do
		{

			++pointer;

			if( buffer + pointer >= valid_addr )
				return	0;

			get_charactor	= *( buffer + pointer );

		}
		while( get_charactor != CHARATOR_SPACE && get_charactor != CHARATOR_NEXT_LINE );

		++pointer;

		if( get_charactor == CHARATOR_NEXT_LINE )
			break;

	}

	return	pointer;

}
EXPORT_SYMBOL_GPL( get_para_char_count );

unsigned int	get_para_from_buffer( const char *buffer, const char *string, ... )
{

	va_list	args;

	int	return_value;

	va_start( args, string );

	return_value	= vsscanf( buffer, string, args );

	va_end( args );

	return	return_value;

}
EXPORT_SYMBOL_GPL( get_para_from_buffer );

void	BST_init( struct BST_info *info )
{

	info->BST_tree	= 0;

	info->count	= 0;

}
EXPORT_SYMBOL_GPL( BST_init );

void	BST_add( struct BST_info *info, struct BST_data *data )
{

	struct BST_data	**next	= &info->BST_tree;

	data->left	= data->right	= 0;

	while( *next )
		next	= ( *next )->index >= data->index ? &( *next )->left : &( *next )->right;

	*next	= data;

	info->count++;

}
EXPORT_SYMBOL_GPL( BST_add );

struct BST_data*	BST_delete_min( struct BST_info *info )
{

	struct BST_data	**next	= &info->BST_tree;

	struct BST_data	*return_data;

	if( !info->count )
		return	0;

	while( ( *next )->left )
		next	= &( *next )->left;

	return_data	= *next;

	*next		= ( *next )->right ? ( *next )->right : 0;

	info->count--;

	return	return_data;

}
EXPORT_SYMBOL_GPL( BST_delete_min );

struct BST_data*	BST_delete_max( struct BST_info *info )
{

	struct BST_data	**next	= &info->BST_tree;

	struct BST_data	*return_data;

	if( !info->count )
		return	0;

	while( ( *next )->right )
		next	= &( *next )->right;

	return_data	= *next;

	*next		= ( *next )->left ? ( *next )->left : 0;

	info->count--;

	return	return_data;

}
EXPORT_SYMBOL_GPL( BST_delete_max );

struct BS_data*	binary_search( struct BS_data *data, int length, int index )
{

	struct BS_data	*return_data	= 0;

	int	max = length - 1, min = 0, mid = ( 0 + length ) / 2;

	while( min <= max )
	{

		mid	= ( min + max ) / 2;

		if( ( data + mid )->index == index )
		{

			return_data	= ( data + mid );

			break;

		}

		if( ( data + mid )->index > index )
			max	= mid - 1;
		else
			min	= mid + 1;

	}

	return	return_data;

}
EXPORT_SYMBOL_GPL( binary_search );

int	BST_sort( struct BST_info *info, struct BS_data *buffer, int buffer_size )
{

	struct BST_data	*data;

	int	loop, length = info->count;

	if( info->count > buffer_size )
		return	0;

	for( loop = 0 ; loop < length ; ++loop )
	{

		data	= BST_delete_min( info );

		( buffer + loop )->index	= data->index;

		( buffer + loop )->data	= data->data;

	}

	return	length;

}
EXPORT_SYMBOL_GPL( BST_sort );
