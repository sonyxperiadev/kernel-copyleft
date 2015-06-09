
#pragma once

#include <linux/module.h>

#include <linux/kernel.h>

#include <linux/init.h>

#include <linux/mutex.h>

#include <linux/platform_device.h>

#include <linux/slab.h>

#include <linux/development-tool/command_maker.h>

// ==========================================================================================================================

#define	INFO_BUFFER_SIZE	4096

#define	PARA_BUFFER_SIZE	1024

#define	RETURN_BUFFER_SIZE	4096

// ==========================================================================================================================

#define	INFO_BUFFER_RESET	0

#define	INFO_BUFFER_INIT	1

#define	INFO_BUFFER_ADD		2

// ==========================================================================================================================

struct	command_information
{

	char	*string;

	unsigned int	command_id;

};

struct	command_string
{

	char *parameter_information;

	struct command_information	*list;

	unsigned int	count;

};

struct	united_file_node_data
{

	struct control_class_device	*file_node_class;

	struct command_info		command;

	struct command_string		string;

	char	*info_buffer, *parameter_buffer, *return_buffer;

	void	*device_data;

};
