/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#include "max3510x_regs.h"
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(MAX35102)
#define MAX3510X_MAX_HITCOUNT 3
#else
#define MAX3510X_MAX_HITCOUNT 6
#endif

#if defined(MAX35103) || defined(MAX35104)

typedef struct _rtc_date_t
{
	// RTC data format used by the MAX35103 and MAX35104
	uint8_t	year;
	uint8_t month;
	uint8_t day_of_month;
	uint8_t day_of_week;
	uint8_t hour;
	uint8_t minute;
	uint8_t seconds;
	uint8_t hundredths;
}
rtc_date_t;

void max3510x_rtc_get( rtc_date_t *p_date, const void *pv_rtc_data );
void max3510x_rtc_set( uint16_t *p_rtc_data, const rtc_date_t *p_date );

#endif // #if defined(MAX35103) || defined(MAX35104)

typedef enum
{
	// event timing modes for the MAX35103 and MAX35104
	max3510x_event_timing_mode_1,
	max3510x_event_timing_mode_2,
	max3510x_event_timing_mode_3
}
max3510x_event_timing_mode_t;

#pragma pack(1)

typedef struct _max3510x_fixed_t
{
	// integer/fraction representation used by the MAX3510x
	// use max3510x_ratio_to_float() 
	int16_t		integer;
	uint16_t	fraction;
}
max3510x_fixed_t;

typedef struct _max3510x_measurement_t
{
	// raw measurement data returned by the MAX3510x
	uint8_t				t2_ideal;	// t1/ideal ratio
	uint8_t				t1_t2;		// t1/t2 ratio (use max3510x_ratio_to_float() )
	max3510x_fixed_t	hit[6];		// hit values (MAX35101 only returns 3 max)
	max3510x_fixed_t	average;	// average of all hit values
}
max3510x_measurement_t;

typedef struct _max3510x_results_t
{
	// raw measurement data returned by the MAX3510x

	uint8_t					start_register;
	max3510x_measurement_t	up;
	max3510x_measurement_t	down;
	max3510x_fixed_t		tof_diff;
	uint8_t					tof_range;
	uint8_t					tof_cycle_count;
	max3510x_fixed_t		tof_diff_ave;
	max3510x_fixed_t		temp[4];
	uint8_t					undefined2;
	uint8_t					temp_cycle_count;
	max3510x_fixed_t		ave_temp[4];
	max3510x_fixed_t		calibration;
}
max3510x_results_t;

typedef struct _max3510x_register_t
{
	uint8_t		offset;
	uint16_t	value;
}
max3510x_register_t;

#if defined(MAX35104)

typedef struct _max35104_registers_t
{
	uint8_t		start_register;
	uint16_t	switcher1;
	uint16_t	switcher2;
	uint16_t	afe1;
	uint16_t	afe2;
}
max35104_registers_t;

#endif

typedef struct _max3510x_common_registers_t
{
	uint8_t		start_register;
	uint16_t	tof1;
	uint16_t	tof2;
	uint16_t	tof3;
	uint16_t	tof4;
	uint16_t	tof5;
	uint16_t	tof6;
	uint16_t	tof7;
#if defined(MAX35101) || defined(MAX35103) || defined(MAX35104)
	uint16_t	event_timing1;
	uint16_t	event_timing2;
#endif
#if defined(MAX35102)
	uint16_t	reserved;
	uint16_t	temperature;
#endif
	uint16_t	measurement_delay;
	uint16_t	calibration;
	uint16_t	rtc;
}
max3510x_common_registers_t;

typedef struct _max3510x_registers_t
{
#if defined(MAX35104)
	max35104_registers_t 		max35104_registers;
#endif
	max3510x_common_registers_t 	common;
}
max3510x_registers_t;


typedef void * max3510x_t;

#pragma pack()

typedef struct _max3510x_float_measurement_t
{
	// 32-bit float representation for measurement data

	float_t				t2_ideal;
	float_t				t1_t2;
	float_t				hit[MAX3510X_MAX_HITCOUNT];
	float_t				average;
}
max3510x_float_measurement_t;

typedef struct _max3510x_float_results_t
{
	// 32-bit float representation for all measurement data

	max3510x_float_measurement_t	up;
	max3510x_float_measurement_t	down;
	float_t				tof_diff;
	uint8_t				tof_range;
	uint8_t				tof_cycle_count;
	float_t				tof_diff_ave;
	float_t				temp[4];
	uint8_t				temp_cycle_count;
	float_t				ave_temp[4];
	float_t				calibration;
}
max3510x_float_results_t;

void max3510x_set_measurement_delay( max3510x_t *p_max3510x, float_t delay_us );
float_t max3510x_fixed_to_float( const max3510x_fixed_t *p_number );
double_t max3510x_fixed_to_double(const max3510x_fixed_t *p_number );
float_t max3510x_ratio_to_float( uint8_t fixed );
void max3510x_convert_results( max3510x_float_results_t *p_float_results, const max3510x_results_t * p_results );
bool max3510x_validate_measurement( const max3510x_measurement_t *p_measurement, uint8_t hit_count );
void max3510x_read_fixed( max3510x_t *p_max3510x, uint8_t reg, max3510x_fixed_t *p_fixed );
uint16_t max3510x_read_register( max3510x_t *p_max3510x, uint8_t reg_offset );
void max3510x_write_register( max3510x_t *p_max3510x, uint8_t reg_offset, uint16_t value );
void max3510x_write_registers( max3510x_t *p_max3510x, uint8_t register_offset, max3510x_register_t *p_reg, uint8_t size );
void max3510x_write_registers_const( max3510x_t *p_max3510x, const max3510x_register_t *p_reg, uint8_t size );
void max3510x_read_registers( max3510x_t *p_max3510x, uint8_t register_offset, max3510x_register_t *p_reg, uint8_t size );
uint16_t max3510x_interrupt_status( max3510x_t *p_max3510x );
void max3510x_read_results( max3510x_t *p_max3510x, max3510x_results_t *p_results );
void max3510x_init( max3510x_t *p_max3510x, const max3510x_registers_t * p_init );
void max3510x_tof_up( max3510x_t *p_max3510x );
void max3510x_tof_down( max3510x_t *p_max3510x );
void max3510x_tof_diff( max3510x_t *p_max3510x );
void max3510x_temperature( max3510x_t *p_max3510x );
void max3510x_reset( max3510x_t *p_max3510x );
void max3510x_initialize( max3510x_t *p_max3510x );
void max3510x_enable_interrupt( max3510x_t *p_max3510x, bool enable );

#if !defined(MAX35104)

uint16_t max3510x_unlock(max3510x_t *p_max3510x );
void max3510x_flash_configuration( max3510x_t *p_max3510x );

#else

void max3510x_bandpass_calibrate( max3510x_t *p_max3510x );

#endif

void max3510x_event_timing( max3510x_t *p_max3510x, max3510x_event_timing_mode_t mode );
void max3510x_halt( max3510x_t *p_max3510x );

#if defined(MAX35103) || defined(MAX35101)

typedef enum
{
	// LDO settings for the MAX35103/MAX35101
	max3510x_ldo_mode_timed,
	max3510x_ldo_mode_on,
	max3510x_ldo_mode_off
}
max3510x_ldo_mode_t;

void max3510x_ldo( max3510x_t *p_max3510x, max3510x_ldo_mode_t mode );
void max3510x_write_flash( max3510x_t *p_max3510x, uint16_t address, uint16_t value );
uint16_t max3510x_read_flash( max3510x_t *p_max3510x, uint16_t address );
void max3510x_erase_flash_block( max3510x_t *p_max3510x, uint16_t address );

#endif

void max3510x_calibrate( max3510x_t *p_max3510x );
uint32_t max3510x_input_frequency( max3510x_fixed_t * p_calibration_value );

float_t max3510x_calibration_factor( uint32_t input_frequency );

void max3510x_spi_xfer( max3510x_t *p_max3510x, void *pv_in, const void *pv_out, uint8_t count );	// should be instantiated in the target board module

#if defined( __GNUC__ ) || defined( __IAR_SYSTEMS_ICC__ ) || defined( __CC_ARM )

#if !defined(__BIG_ENDIAN)

#define MAX3510X_ENDIAN(w)	( (((uint16_t)(w))>>8) | (uint16_t)((((uint16_t)(w))<<8)) )
#define MAX3510X_ENDIAN_SWAP
#else
#define MAX3510X_ENDIAN(w)	(w)
#endif

#else
#error "Define byte-order for unsupported compiler."
#endif


#define MAX3510X_WRITE_BITFIELD( c, r, bf, v ) max3510x_write_bitfield( (max3510x_t*)c, MAX3510X_REG_##r, MAX3510X_REG_MASK(r##_##bf) << MAX3510X_REG_##r##_##bf##_SHIFT, (v & MAX3510X_REG_MASK(r##_##bf)) << MAX3510X_REG_##r##_##bf##_SHIFT  )
#define MAX3510X_READ_BITFIELD( c, r, bf) (max3510x_read_bitfield( (max3510x_t*)c, MAX3510X_REG_##r, MAX3510X_REG_MASK(r##_##bf##) << MAX3510X_REG_##r##_##bf##_SHIFT ) >> MAX3510X_REG_##r##_##bf##_SHIFT)

void max3510x_write_bitfield( max3510x_t *p_max3510x, uint8_t reg_offset, uint16_t mask, uint16_t value );
uint16_t max3510x_read_bitfield( max3510x_t *p_max3510x, uint8_t reg_offset, uint16_t mask );

#ifdef __cplusplus
};
#endif

