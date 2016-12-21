
#pragma once

#include <linux/development-tool/development_tool.h>

#define	ONE_PARAMETER			1

#define	TWO_CHARATOR			2
#define	THREE_CHARATOR			3

enum	parameter_kind
{

	PARAMETER_KIND_NONE = 0,
	PARAMETER_KIND_NUMBER,
	PARAMETER_KIND_STRING

};

enum	command_parameter_define
{

	COMMAND_0_PARAMENTER = 0,
	COMMAND_1_PARAMENTER,
	COMMAND_2_PARAMENTER,
	COMMAND_3_PARAMENTER,
	COMMAND_4_PARAMENTER,
	COMMAND_5_PARAMENTER,
	COMMAND_6_PARAMENTER,
	COMMAND_7_PARAMENTER,
	COMMAND_8_PARAMENTER,
	COMMAND_9_PARAMENTER,
	COMMAND_10_PARAMENTER,
	COMMAND_11_PARAMENTER,
	COMMAND_12_PARAMENTER,
	COMMAND_13_PARAMENTER,
	COMMAND_14_PARAMENTER,
	COMMAND_15_PARAMENTER,
	COMMAND_16_PARAMENTER,
	COMMAND_17_PARAMENTER,
	COMMAND_18_PARAMENTER,
	COMMAND_19_PARAMENTER,
	COMMAND_20_PARAMENTER,
	COMMAND_MAX_PARAMENTER,
	COMMAND_N_PARAMENTER = -1

};

union	parameter_container
{

	int	number;

	char	*string;

};

struct	parameter_info
{

	enum parameter_kind		kind;

	union parameter_container	container;

};

struct	command_parameter
{

	struct parameter_info	para1;

	struct parameter_info	para2;

	struct parameter_info	para3;

	struct parameter_info	para4;

	struct parameter_info	para5;

	struct parameter_info	para6;

	struct parameter_info	para7;

	struct parameter_info	para8;

	struct parameter_info	para9;

	struct parameter_info	para10;

	struct parameter_info	para11;

	struct parameter_info	para12;

	struct parameter_info	para13;

	struct parameter_info	para14;

	struct parameter_info	para15;

	struct parameter_info	para16;

	struct parameter_info	para17;

	struct parameter_info	para18;

	struct parameter_info	para19;

	struct parameter_info	para20;

};

struct	load_command_data
{

	unsigned int			command_id;

	void	( *function )( void*, struct command_parameter* );

	char				*parameter_string;

	enum command_parameter_define	parameter_count;

	void				*other;

};

struct	command_data
{

	void	( *function )( void*, struct command_parameter* );

	char		*parameter_string;

	unsigned int	parameter_count;

	void		*other;

};

struct	command_info
{

	struct BS_data		*commmand_buffer;

	struct command_data	*command_data_memory;

	unsigned int		count;

};

int load_command( struct command_info*, struct load_command_data*, unsigned int );

void release_command_resource( struct command_info* );
