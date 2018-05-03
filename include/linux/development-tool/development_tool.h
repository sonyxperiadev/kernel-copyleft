#ifndef	FIH_DEVELOPMENT_TOOL_H
#define	FIH_DEVELOPMENT_TOOL_H

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/err.h>

struct	control_node_load
{

	const char	*class_name;

	const char	*device_name;

	void		*file_node_data;

	ssize_t ( *control_read )( void*, struct device_attribute*, char* );

	ssize_t ( *control_write )( void*, struct device_attribute*, const char*, size_t );

	ssize_t ( *info_read )( void*, struct device_attribute*, char* );

};

struct	control_class_device
{

	struct class	*Class;

	struct device	*Device;

};

struct	BST_data
{

	int	index;

	struct BST_data	*left, *right;

	void	*data;

};

struct	BST_info
{

	struct BST_data	*BST_tree;

	int	count;

};

struct	BS_data
{

	int	index;

	void	*data;

};

struct control_class_device*	control_file_node_register( struct control_node_load* );

void control_file_node_unregister( struct control_class_device* );

unsigned int	get_para_char_count( const char*, const char*, unsigned int );

unsigned int	get_para_from_buffer( const char*, const char*, ... );

void		BST_init( struct BST_info* );

void		BST_add( struct BST_info*, struct BST_data* );

struct BST_data*	BST_delete_min( struct BST_info* );

struct BST_data*	BST_delete_max( struct BST_info* );

struct BS_data*	binary_search( struct BS_data*, int, int );

int		BST_sort( struct BST_info*, struct BS_data*, int );

#endif
