
#pragma once

#include <linux/development-tool/united_file_node_register.h>

#include <linux/i2c.h>

#include <linux/mutex.h>

#include <linux/pm_runtime.h>

#include <linux/fs.h>

#include <asm/uaccess.h>

#include <stdarg.h>

#include <linux/qpnp/pin.h>

#include <linux/qpnp/qpnp-adc.h>

#include <linux/fih_hw_info.h>

// ==========================================================================================================================

#define CY_I2C_DATA_SIZE  ( 3 * 256 )

struct cyttsp4_i2c {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	char const *id;
	struct mutex lock;
	struct qpnp_vadc_chip *touch_sensor;
};

int cyttsp4_i2c_read_block_data(struct cyttsp4_i2c*, u16, int, void*, int);

int cyttsp4_i2c_write_block_data(struct cyttsp4_i2c*, u16, int, const void*, int);

static inline int i2c_read(struct cyttsp4_i2c *ts, u16 addr, void *buf, int size, int max_xfer)
{
	int rc;

	pm_runtime_get_noresume(&ts->client->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_read_block_data(ts, addr, size, buf, max_xfer);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(&ts->client->dev);

	return rc;
}

static inline int i2c_write(struct cyttsp4_i2c *ts, u16 addr, const void *buf, int size, int max_xfer)
{
	int rc;

	pm_runtime_get_noresume(&ts->client->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_write_block_data(ts, addr, size, buf, max_xfer);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(&ts->client->dev);

	return rc;
}

static inline bool df_open( const char *file_name, int access, umode_t mode, struct file **file_pointer )
{
	if( !file_pointer )
	{
		printk( "FTUCH : NULL file pointer\n" );
		return	false;
	}

	{
		struct file *open_file_pointer = filp_open( file_name, access, mode );

		if( IS_ERR( open_file_pointer ) )
		{
			printk( "FTUCH : Failed to open file(%s)\n", file_name );
			return	false;
		}

		if( !open_file_pointer->f_op || !open_file_pointer->f_op->read || !open_file_pointer->f_op->write || !open_file_pointer->f_op->llseek )
		{
			printk( "FTUCH : Incorrect file operator pointer\n" );
			return	false;
		}

		*file_pointer	= open_file_pointer;
	}

	return	true;
}

static inline bool df_close( struct file *file_pointer )
{
	if( filp_close( file_pointer, NULL ) )
	{
		printk( "TTUCH : Failed to close file\n" );
		return	false;
	}

	return	true;
}

static inline void df_seek( struct file *file_pointer, unsigned offset, unsigned whence )
{
	( void )file_pointer->f_op->llseek( file_pointer, offset, whence );
}

static inline bool df_read( struct file *file_pointer, char *buffer, unsigned length, unsigned *return_count )
{
	int	read_count;
	if( !return_count )
	{
		printk( "TTUCH : The return pointer is NULL\n" );
		return	false;
	}

	set_fs( get_ds() );
	if( ( read_count = file_pointer->f_op->read( file_pointer, buffer, length, &file_pointer->f_pos ) ) < 0 )
	{
		printk( "TTUCH : Failed to read data from file\n" );
		return	false;
	}

	*return_count	= read_count;

	return	true;
}

static inline bool df_write( struct file *file_pointer, char *buffer, unsigned length, unsigned *return_count )
{
	int	write_count;
	if( !return_count )
	{
		printk( "TTUCH : The return pointer is NULL\n" );
		return	false;
	}

	set_fs( get_ds() );
	if( ( write_count = file_pointer->f_op->write( file_pointer, buffer, length, &file_pointer->f_pos ) ) < 0 )
	{
		printk( "TTUCH : Failed to write data to file\n" );
		return	false;
	}

	*return_count	= write_count;

	return	true;
}

static inline bool set_group_offset( unsigned group, unsigned offset )
{
	struct file	*grpnum_file, *grpoffset_file;
	char		group_value[ 32 ], offset_value[ 32 ];
	unsigned	return_count;

	if( !df_open( "/sys/bus/ttsp4/devices/cyttsp4_device_access.main_ttsp_core/ic_grpnum", O_WRONLY, 0, &grpnum_file ) )
	{
		return	false;
	}

	if( !df_open( "/sys/bus/ttsp4/devices/cyttsp4_device_access.main_ttsp_core/ic_grpoffset", O_WRONLY, 0, &grpoffset_file ) )
	{
		df_close( grpnum_file );
		return	false;
	}

	snprintf( group_value, sizeof( group_value ), "%d", group ), snprintf( offset_value, sizeof( offset_value ), "%d", offset );

	if( !df_write( grpnum_file, group_value, strlen( group_value ), &return_count ) || !df_write( grpoffset_file, offset_value, strlen( offset_value ), &return_count ) )
	{
		df_close( grpnum_file ), df_close( grpoffset_file );
		return	false;
	}

	df_close( grpnum_file ), df_close( grpoffset_file );
	return	true;
}

static inline bool set_group_data( unsigned *command_buffer, unsigned buffer_size )
{
	struct file	*grpdata_file;
	unsigned	loop, return_count, index;
	char		data[ 128 ] = { 0 };

	if( !df_open( "/sys/bus/ttsp4/devices/cyttsp4_device_access.main_ttsp_core/ic_grpdata", O_WRONLY, 0, &grpdata_file ) )
	{
		return	false;
	}

	for( loop = index = 0 ; loop < buffer_size ; ++loop )
		index += snprintf( data + index, sizeof( data ), !loop ? "0x%02x" : ", 0x%02x", *( command_buffer + loop ) );

	df_seek( grpdata_file, 0, 0 );

	if( !df_write( grpdata_file, data, strlen( data ), &return_count ) )
		return	false;

	df_close( grpdata_file );
	return	true;
}

static inline bool get_group_data( char *store_buffer, unsigned buffer_size )
{
	struct file	*grpdata_file;
	unsigned	return_count;

	if( !df_open( "/sys/bus/ttsp4/devices/cyttsp4_device_access.main_ttsp_core/ic_grpdata", O_WRONLY, 0, &grpdata_file ) )
	{
		return	false;
	}

	df_seek( grpdata_file, 0, 0 );

	if( !df_read( grpdata_file, store_buffer, buffer_size, &return_count ) )
		return	false;

	df_close( grpdata_file );
	return	true;
}

// ==========================================================================================================================

static void	read_from_i2c( void *file_node_data, struct command_parameter *parameter_data )
{
	struct	parameters_info
	{
		int	address, count;
	};

	struct parameters_info	parameters =
	{
		parameter_data->para1.container.number, parameter_data->para2.container.number,
	};

	struct cyttsp4_i2c	*i2c_info = ( struct cyttsp4_i2c* )( ( struct united_file_node_data* )file_node_data )->device_data;

	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;

	char	*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );

	if( parameters.count >= RETURN_BUFFER_SIZE - sizeof( struct file_node_return_struct ) )
	{
		printk( "FTUCH : Out of range\n" );
		return_struct->error_code	= ERROR_OUT_OF_RANGE;
		return;
	}

	return_struct->count	= parameters.count;

	if( i2c_read( i2c_info, parameters.address, return_buffer, parameters.count, parameters.count ) )
	{
		printk( "FTUCH : Failed to I2C read\n" );
		return_struct->error_code	= ERROR_ACCESS;
	}

}

static void	write_to_i2c( void *file_node_data, struct command_parameter *parameter_data )
{
	struct	parameters_info
	{
		int	address, count;
	};

	struct parameters_info	parameters =
	{
		parameter_data->para1.container.number, parameter_data->para2.container.number,
		
	};

	char	send_buffer[] =
	{
		parameter_data->para3.container.number, parameter_data->para4.container.number, parameter_data->para5.container.number,
		parameter_data->para6.container.number, parameter_data->para7.container.number, parameter_data->para8.container.number,
		parameter_data->para9.container.number, parameter_data->para10.container.number, parameter_data->para11.container.number,
		parameter_data->para12.container.number, parameter_data->para13.container.number, parameter_data->para14.container.number,
		parameter_data->para15.container.number, parameter_data->para16.container.number
	};

	struct cyttsp4_i2c	*i2c_info = ( struct cyttsp4_i2c* )( ( struct united_file_node_data* )file_node_data )->device_data;
	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;
	char	*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );

	if( parameters.count > sizeof( send_buffer ) )
	{
		printk( "FTUCH : Out of range\n" );
		return_struct->error_code	= ERROR_OUT_OF_RANGE;
		return;
	}

	return_struct->count	= parameters.count;

	memcpy( return_buffer, send_buffer, sizeof( send_buffer ) );

	if( i2c_write( i2c_info, parameters.address, send_buffer, parameters.count, parameters.count ) )
	{
		printk( "FTUCH : Failed to I2C write\n" );
		return_struct->error_code	= ERROR_ACCESS;
	}
}

static void	convert_to_information_buffer( void *file_node_data, struct command_parameter *parameter_data )
{
	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;
	char	*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );
	char	*info_buffer = ( ( struct united_file_node_data* )file_node_data )->info_buffer;
	unsigned	pointer, loop;

	pointer	= snprintf( info_buffer, INFO_BUFFER_SIZE, "Error(%d), ID(%s)\n[Command:Count]=[%d:%d]\n\n", return_struct->backup_error_code, return_struct->check_id, return_struct->command_id, return_struct->count );

	if( return_struct->count * 2 >= RETURN_BUFFER_SIZE - sizeof( struct file_node_return_struct ) )
		return;

	for( loop = 0 ; loop < return_struct->count ; ++loop )
		pointer += snprintf( info_buffer + pointer, INFO_BUFFER_SIZE, loop % 8 ? "%02x " : "\n%02x " , *( return_buffer + loop ) );

	snprintf( info_buffer + pointer, INFO_BUFFER_SIZE, "\n" );
}

static void	get_mode( void *file_node_data, struct command_parameter *parameter_data )
{
	char	*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );
	struct command_parameter	parameter;

	parameter.para1.kind = PARAMETER_KIND_NUMBER, parameter.para2.kind = PARAMETER_KIND_NUMBER;
	parameter.para1.container.number = /* IC address */ 0, parameter.para2.container.number = /* Read count */ 1;
	read_from_i2c( file_node_data, &parameter );
	*return_buffer = *return_buffer >> 4 & 0x7;
}

static void	get_firmware_version( void *file_node_data, struct command_parameter *parameter_data )
{
	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;
	char		*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );

	#define	GROUP	3
	#define	OFFSET	0
	#define	INFORMATION_STRING_LENGTH	( sizeof( "Group 3, Offset 0:" ) + 34 * 5 )
	#define	INFORMATION_VALUE_LENGTH	34
	#define	INFORMATION_OFFSET		2
	#define	NEXT_VALUE			5

	if( !set_group_offset( GROUP, OFFSET ) )
	{
		printk( "FTUCH : Failed to set group/offset\n" );
		return_struct->error_code	= ERROR_INVALID_PARAMETER;
		return;
	}

	if( !get_group_data( return_buffer, INFORMATION_STRING_LENGTH ) )
	{
		printk( "FTUCH : Failed to get ic information\n" );
		return_struct->error_code	= ERROR_ACCESS;
		return;
	}

	if( memcmp( "Group 3, Offset 0:", return_buffer, sizeof( "Group 3, Offset 0:" ) - 1 ) )
	{
		printk( "FTUCH : Failed to check header of data\n" );
		return_struct->error_code	= ERROR_INVALID_DATA;
		return;
	}

	{
		unsigned	loop, value, index = sizeof( "Group 3, Offset 0:" );

		for( loop = 0 ; loop < INFORMATION_VALUE_LENGTH ; ++loop )
		{
			 sscanf( return_buffer + index + INFORMATION_OFFSET, "%x", &value );
			 index += NEXT_VALUE;
			*( return_buffer + loop )	= value;
		}

		return_struct->count	= INFORMATION_VALUE_LENGTH;
	}

	#undef	GROUP
	#undef	OFFSET
	#undef	INFORMATION_STRING_LENGTH
	#undef	INFORMATION_VALUE_LENGTH
}

static void	show_information( void *file_node_data, struct command_parameter *parameter_data )
{
	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;
	char	*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );
	char	*info_buffer = ( ( struct united_file_node_data* )file_node_data )->info_buffer;

	get_firmware_version( file_node_data, parameter_data );

	if( return_struct->error_code != ERROR_NONE )
		return;

	snprintf( info_buffer, INFO_BUFFER_SIZE,
		"Cypress Gen4 information :\n"
		"Product ID(0x%x,0x%x)\n"
		"Firmware version[Major:Minor]=[0x%x,0x%x]\n"
		"Revision control(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n"
		"Configuration version(0x%x,0x%x)\n"
		"Bootloader version[Major:Minor]=[0x%x,0x%x]\n",
		*return_buffer, *( return_buffer + 1 ),
		*( return_buffer + 2 ), *( return_buffer + 3 ),
		*( return_buffer + 4 ), *( return_buffer + 5 ), *( return_buffer + 6 ), *( return_buffer + 7 ), *( return_buffer + 8 ), *( return_buffer + 9 ), *( return_buffer + 10 ), *( return_buffer + 11 ),
		*( return_buffer + 29 ), *( return_buffer + 30 ),
		*( return_buffer + 12 ), *( return_buffer + 13 ) );
}

static void	get_bootloader_mode_state( void *file_node_data, struct command_parameter *parameter_data )
{
	char	*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );
	struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;
	struct file	*get_bootloader_mode_file;
	unsigned	return_count;
	char		bootloader_state = 0;

	if( !df_open( "/sys/bus/ttsp4/devices/main_ttsp_core.cyttsp4_i2c_adapter/get_bootloader_state", O_RDONLY, 0, &get_bootloader_mode_file ) )
	{
		printk( "ITUCH : Failed to open set_mode file\n" );
		return_struct->error_code	= ERROR_INVALID_DATA;
		return;
	}

	if( !df_read( get_bootloader_mode_file, &bootloader_state, 1, &return_count ) )
	{
		printk( "Failed to get bootloader state!!!\n" );
		return_struct->error_code	= ERROR_ACCESS;
		return;
	}

	df_close( get_bootloader_mode_file );
	*return_buffer = bootloader_state, return_struct->count = 1;
}

static void	get_sensor_id( void *file_node_data, struct command_parameter *parameter_data )
{
	struct qpnp_vadc_chip *touch_sensor = ( ( struct cyttsp4_i2c* )( ( struct united_file_node_data* )file_node_data )->device_data )->touch_sensor;

	if( IS_ERR( touch_sensor ) )
	{
		printk( "ITUCH : Invalid vlaue!!!\n" );
		return;
	}

	{
		struct file_node_return_struct	*return_struct = ( struct file_node_return_struct* )( ( struct united_file_node_data* )file_node_data )->return_buffer;
		char		*return_buffer = ( ( struct united_file_node_data* )file_node_data )->return_buffer + sizeof( struct file_node_return_struct );
		unsigned	 product_phase = fih_get_product_phase();
		struct qpnp_vadc_result	result;

		if( product_phase <= PHASE_DP )
		{
			printk( "ITUCH : EVM or DP, WinTEK OGS\n" );
			*return_buffer = 5, return_struct->count = 1;
			return;
		}

		if( qpnp_vadc_read( touch_sensor, P_MUX8_1_3, &result ) )
			printk( "ITUCH : Failed to read ADC!!!\n" );
		else
		{
			struct adc_range
			{
				unsigned	min, max;
			};

			static struct adc_range	adc_map[] =
			{
				{ 78000, 251999 }, { 252000, 440999 }, { 441000, 652999 }, { 653000, 979999 }, { 980000, 1494999 }, { 1495000, 2103999 },
			};

			unsigned	adc_value = result.physical;
			unsigned	loop;

			printk( "ITUCH : ADC value = %u\n", adc_value );

			for( loop = 0 ; loop < sizeof( adc_map ) / sizeof( *adc_map ) ; ++loop )
				if( adc_value >= ( adc_map + loop )->min && adc_value <= ( adc_map + loop )->max )
				{
					unsigned	touhc_id = loop + 5;
					printk( "ITUCH : Sensor ID(%d)\n", touhc_id );
					*return_buffer = touhc_id, return_struct->count = 1;
					return;
				}

			return_struct->error_code	= ERROR_INVALID_DATA;
		}
	}
}

static void	set_touch_ic_mode( void *file_node_data, struct command_parameter *parameter_data )
{
	struct file	*set_mode_file;
	unsigned	return_count;
	char		mode = parameter_data->para1.container.number ^ 0x30;
	bool		return_status;

	if( !df_open( "/sys/bus/ttsp4/devices/main_ttsp_core.cyttsp4_i2c_adapter/set_mode", O_WRONLY, 0, &set_mode_file ) )
	{
		printk( "ITUCH : Failed to open set_mode file\n" );
		return;
	}

	return_status	= df_write( set_mode_file, &mode, 1, &return_count );
	df_close( set_mode_file );
}

// ==========================================================================================================================

enum	touch_command_id
{

	// 0 ~ 19
	SHOW_INFORMATION		= 0,

	// 20 ~ 39
	READ_FROM_I2C			= 20,
	WRITE_TO_I2C			= 21,

	// 40 ~ 59
	SET_MODE			= 40,

	// 60 ~ 79
	CONVERT_TO_INFORMATION_BUFFER	= 60,

	// 80 ~ 99
	GET_FIRMWARE_VERSION		= 80,
	GET_SENSOR_ID			= 81,
	GET_MODE			= 82,
	GET_BOOTLOADER_MODE_STATE	= 83,

};

// ==========================================================================================================================

#define	LOAD_COMMAND_DATA \
IBUFFER( SHOW_INFORMATION, show_information, NULL, COMMAND_0_PARAMENTER ), \
RBUFFER( GET_FIRMWARE_VERSION, get_firmware_version, NULL, COMMAND_0_PARAMENTER ), \
RBUFFER( READ_FROM_I2C, read_from_i2c, "%x %d", COMMAND_2_PARAMENTER ), \
RBUFFER( WRITE_TO_I2C, write_to_i2c, "%x %d %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x", COMMAND_N_PARAMENTER ), \
IBUFFER( CONVERT_TO_INFORMATION_BUFFER, convert_to_information_buffer, NULL, COMMAND_0_PARAMENTER ), \
RBUFFER( GET_MODE, get_mode, NULL, COMMAND_0_PARAMENTER ), \
RBUFFER( GET_BOOTLOADER_MODE_STATE, get_bootloader_mode_state, NULL, COMMAND_0_PARAMENTER ), \
RBUFFER( GET_SENSOR_ID, get_sensor_id, NULL, COMMAND_0_PARAMENTER ), \
RBUFFER( SET_MODE, set_touch_ic_mode, "%d", COMMAND_1_PARAMENTER ), \

// ==========================================================================================================================

#define	PARAMETER_STRING \
"\n[Parameter define]\n" \
"ADDR: address, COUT: count, VAL: value\n" \
"D : dec, H : hex\n\n" \

// ==========================================================================================================================

#define	COMMAND_INFORMATION \
CAT_COMMAND( "Show touch information, [NULL]", SHOW_INFORMATION ), \
CAT_COMMAND( "get firmware version, [NULL]", GET_FIRMWARE_VERSION ), \
CAT_COMMAND( "read from I2C, [ADDR,COUT]=[H,D]", READ_FROM_I2C ), \
CAT_COMMAND( "Write to I2C, [ADDR,COUT,VAL0-VAL15]=[H,D,H-H]", WRITE_TO_I2C ), \
CAT_COMMAND( "Convert to information buffer, [NULL]", CONVERT_TO_INFORMATION_BUFFER ), \
CAT_COMMAND( "Get mode of Cypress touch IC, [NULL]", GET_MODE ), \
CAT_COMMAND( "Get status of bootloader mode, [NULL]", GET_BOOTLOADER_MODE_STATE ), \
CAT_COMMAND( "Get sensor ID, [NULL]", GET_SENSOR_ID ), \
CAT_COMMAND( "Set touch mode, [D]", SET_MODE ), \

// ==========================================================================================================================
