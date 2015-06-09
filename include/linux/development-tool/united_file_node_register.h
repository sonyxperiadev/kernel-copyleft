
#pragma once

#include <linux/development-tool/command_maker.h>

#include <linux/development-tool/united_file_node.h>

#include <linux/development-tool/functions.h>

// ==========================================================================================================================

#define	ARRAY_COUNT( ARRAY )	( sizeof( ARRAY ) / sizeof( *ARRAY ) )

// ==========================================================================================================================

#define	IBUFFER( COMMAND_ID, FUNCTION, STRING, COUNT )	{ COMMAND_ID, FUNCTION, STRING, COUNT, ( void* )0 }

#define	RBUFFER( COMMAND_ID, FUNCTION, STRING, COUNT )	{ COMMAND_ID, FUNCTION, STRING, COUNT, ( void* )1 }

#define	CAT_COMMAND( STRING, COMMAND_ID )	{ "%02d : "STRING"\n", COMMAND_ID }

// ==========================================================================================================================

struct	execution_command_information
{

	struct load_command_data	*list;

	unsigned	count;

};

struct	show_command_information
{

	struct command_information	*list;

	unsigned	count;

	char	*parameter_string;

};

struct	united_file_node_register_data
{

	char	*class_name, *device_name;

	struct execution_command_information	execution_command;

	struct show_command_information		show_command;

};

// ==========================================================================================================================

int united_file_node_register( struct platform_device*, struct united_file_node_register_data* );

int united_file_node_unregister( struct platform_device* );