
#pragma once

// ==========================================================================================================================

#define	COMMAND_GROUP_RANGE		20

// ==========================================================================================================================

#define	RETURN_STRUCT_STRING		"a9115055"

#define	SECURITY_ID			0xa9115055

// ==========================================================================================================================

#pragma	pack( push )

#pragma	pack( 1 )

struct	file_node_return_struct
{

	unsigned	error_code, backup_error_code;

	char		check_id[ 9 ];

	unsigned	command_id;

	unsigned	count;

};

#pragma	pack( pop )

enum	return_error_define
{

	ERROR_BUSY			= 1,
	ERROR_NONE			= 0,
	ERROR_INVALID_DATA		= -1,
	ERROR_INVALID_PARAMETER		= -2,
	ERROR_OUT_OF_RANGE		= -3,
	ERROR_ACCESS			= -4,

};
