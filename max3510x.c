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


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "max3510x.h"
#include "max3510x_regs.h"

#define FREQ_REF	4000000.0	// the assumed input 4MX clock frequency.


void max3510x_write_registers_const( max3510x_t p_max3510x, const max3510x_register_t *p_reg, uint8_t size )
{
	// this function does not handle endianess.  Registers must be in little-endian format.
	max3510x_spi_xfer( p_max3510x, NULL, p_reg, size );
}


void max3510x_write_registers( max3510x_t p_max3510x, uint8_t register_offset, max3510x_register_t *p_reg, uint8_t size )
{
#if defined(MAX3510X_ENDIAN_SWAP)

	uint8_t i;
	uint16_t *p = &p_reg->value;

#endif
	
	p_reg->offset = MAX3510X_OPCODE_WRITE_REG(register_offset);

#if defined(MAX3510X_ENDIAN_SWAP)

	for(i=0;i<(size-1)/2;i++)
	{
		p[i]=MAX3510X_ENDIAN(p[i]);
	}
#endif

	max3510x_spi_xfer( p_max3510x, NULL, p_reg, size );
}

void max3510x_read_registers( max3510x_t p_max3510x, uint8_t register_offset, max3510x_register_t *p_reg, uint8_t size )
{
	p_reg->offset = MAX3510X_OPCODE_READ_REG(register_offset);
	max3510x_spi_xfer( p_max3510x, p_reg, p_reg, size );
	p_reg->offset = register_offset;
#if defined(MAX3510X_ENDIAN_SWAP)
	uint8_t i;
	uint16_t *p = &p_reg->value;
	for(i=0;i<(size-1)/2;i++)
	{
		p[i] = MAX3510X_ENDIAN(p[i]);
	}
#endif // #if defined(MAX3510X_ENDIAN_SWAP)
}

uint16_t max3510x_read_register( max3510x_t p_max3510x, uint8_t reg_offset )
{
	max3510x_register_t r;
	max3510x_read_registers( p_max3510x, reg_offset, &r, sizeof(r));
	return r.value;
}

void max3510x_write_register( max3510x_t p_max3510x, uint8_t reg_offset, uint16_t value )
{
	max3510x_register_t r;
	r.value = value;
	max3510x_write_registers( p_max3510x, reg_offset, &r, sizeof(r) );
}

uint16_t max3510x_write_shadow_bitfield( max3510x_t p_max3510x, uint8_t reg_offset, uint16_t mask, uint16_t shadow, uint16_t value )
{
	value = (shadow & ~mask) | value;
	max3510x_write_register(p_max3510x, reg_offset, value );
	return value;
}

void max3510x_write_bitfield( max3510x_t p_max3510x, uint8_t reg_offset, uint16_t mask, uint16_t value )
{
	// value must be shifted into the proper posisiton
	// use the MAX3510X_WRITE_BITFIELD macro instead
	value = (max3510x_read_register(p_max3510x, reg_offset ) & ~mask) | value;
	max3510x_write_register(p_max3510x, reg_offset, value );
}

uint16_t max3510x_read_bitfield( max3510x_t p_max3510x, uint8_t reg_offset, uint16_t mask )
{
	// returned bitfield must be shifted into the proper posisiton
	// use the MAX3510X_READ_BITFIELD macro instead

	uint16_t value;
	value = max3510x_read_register(p_max3510x, reg_offset ) & mask;
	return value;
}

static void send_opcode( max3510x_t p_max3510x, uint8_t code )
{
	// send an opcode to the MAX3510x
	uint8_t result;
	do
	{
		max3510x_spi_xfer( p_max3510x, &result, &code, sizeof(code) );
	}
	while( result != 0xFF );
}

#if defined(MAX35103) || defined(MAX35104) || defined(MAX35101)

static uint8_t rtc_get_year( uint16_t reg_value )
{
	uint8_t year = MAX3510X_REG_GET( MONTH_YEAR_YEAR, reg_value );
	year += MAX3510X_REG_GET( MONTH_YEAR_10YEAR, reg_value ) * 10;
	return year;
}

static uint16_t rtc_set_year( uint8_t year )
{
	return MAX3510X_REG_GET( MONTH_YEAR_YEAR, year % 10 ) | MAX3510X_REG_GET( MONTH_YEAR_10YEAR, year / 10 );
}

static uint8_t rtc_get_month( uint16_t reg_value )
{
	uint8_t month = MAX3510X_REG_GET( MONTH_YEAR_MONTH, reg_value );
	month += MAX3510X_REG_GET( MONTH_YEAR_10MONTH, reg_value ) * 10;
	return month;
}

static uint16_t rtc_set_month( uint8_t month )
{
	return MAX3510X_REG_GET( MONTH_YEAR_10MONTH, month / 10 ) | MAX3510X_REG_GET( MONTH_YEAR_MONTH, month % 10 );
}

static uint8_t rtc_get_day_of_month( uint16_t reg_value )
{
	uint8_t day_of_month = MAX3510X_REG_GET( DAY_DATE_10DATE, reg_value );
	day_of_month += MAX3510X_REG_GET( DAY_DATE_DATE, reg_value );
	return day_of_month;
}

static uint16_t rtc_set_day_of_month( uint8_t day_of_month )
{
	return MAX3510X_REG_GET( DAY_DATE_DATE, day_of_month );
}

static uint8_t rtc_get_day_of_week( uint16_t reg_value )
{
	return MAX3510X_REG_GET( DAY_DATE_DAY, reg_value );
}

static uint16_t rtc_set_day_of_week( uint8_t day_of_week )
{
	return MAX3510X_REG_SET( DAY_DATE_DAY, day_of_week );
}

static uint8_t rtc_get_hours( uint16_t reg_value )
{
	// returns 0-24

	uint8_t hours = MAX3510X_REG_GET( MINS_HRS_HOURS, reg_value );
	if( MAX3510X_REG_GET( MINS_HRS_12_24, reg_value ) == MAX3510X_REG_MINS_HRS_12_24_MODE_12 )
	{
		// 12 hour mode.
		if( MAX3510X_REG_GET( MINS_HRS_20HR_AMPM, reg_value ) == MAX3510X_REG_MINS_HRS_20HR_AMPM_PM )
		{
			// PM
			hours += 12;
		}
	}
	else
	{
		// 24 hour mode.
		if( MAX3510X_REG_GET( MINS_HRS_20HR_AMPM, reg_value ) )
		{
			hours += 20;
		}
	}
	if( MAX3510X_REG_GET( MINS_HRS_10HR, reg_value ) )
	{
		hours += 10;
	}
	return hours;
}

static uint16_t rtc_set_hour( uint8_t hour )
{
	uint16_t reg_value = 0;
	if( hour > 19 )
	{
		reg_value |= MAX3510X_REG_SET( MINS_HRS_20HR_AMPM, 1 );
		hour -= 20;
	}
	if( hour > 9 )
	{
		reg_value |= MAX3510X_REG_SET( MINS_HRS_10HR, 1 );
		hour -= 20;
	}
	reg_value |= MAX3510X_REG_SET( MINS_HRS_HOURS, hour );
	return reg_value;
}

static uint8_t rtc_get_minutes( uint16_t reg_value )
{
	uint8_t minutes = MAX3510X_REG_GET( MINS_HRS_10MINS, reg_value ) * 10;
	minutes += MAX3510X_REG_GET( MINS_HRS_1MINS, reg_value );
	return minutes;
}

static uint16_t rtc_set_minutes( uint8_t minutes )
{
	return MAX3510X_REG_SET( MINS_HRS_10MINS, minutes / 10 ) | MAX3510X_REG_SET( MINS_HRS_1MINS, minutes % 10 );
}

static uint8_t rtc_get_seconds( uint16_t reg_value )
{
	uint8_t seconds = MAX3510X_REG_GET( SECONDS_10SECS, reg_value ) * 10;
	seconds += MAX3510X_REG_GET( SECONDS_1SECS, reg_value );
	return seconds;
}

static uint16_t rtc_set_seconds( uint8_t seconds )
{
	return MAX3510X_REG_SET( SECONDS_10SECS, seconds / 10 ) | MAX3510X_REG_SET( SECONDS_1SECS, seconds % 10 );
}

static uint8_t rtc_get_hundredths( uint16_t reg_value )
{
	uint8_t hundredths = MAX3510X_REG_GET( SECONDS_TENTHS, reg_value ) * 10;
	hundredths += MAX3510X_REG_GET( SECONDS_HUNDREDTHS, reg_value );
	return hundredths;
}

static uint16_t rtc_set_hundredths( uint8_t hundredths )
{
	uint8_t tenths = hundredths / 10;
	uint8_t remainder = hundredths % 10;
	return MAX3510X_REG_SET( SECONDS_HUNDREDTHS, remainder ) | MAX3510X_REG_SET( SECONDS_TENTHS, tenths );
}


void max3510x_rtc_get( rtc_date_t *p_date, const void *pv_rtc_data )
{
	const uint16_t *p_rtc_data = (const uint16_t *)pv_rtc_data;
	p_date->year = rtc_get_year( p_rtc_data[MAX3510X_REG_MONTH_YEAR] );
	p_date->month = rtc_get_month( p_rtc_data[MAX3510X_REG_MONTH_YEAR] );
	p_date->day_of_month = rtc_get_day_of_month( p_rtc_data[MAX3510X_REG_DAY_DATE] );
	p_date->day_of_week = rtc_get_day_of_week( p_rtc_data[MAX3510X_REG_DAY_DATE] );
	p_date->hour = rtc_get_hours( p_rtc_data[MAX3510X_REG_MINS_HRS] );
	p_date->minute = rtc_get_minutes( p_rtc_data[MAX3510X_REG_MINS_HRS] );
	p_date->seconds = rtc_get_seconds( p_rtc_data[MAX3510X_REG_SECONDS] );
	p_date->hundredths = rtc_get_hundredths( p_rtc_data[MAX3510X_REG_SECONDS] );
}

void max3510x_rtc_set( uint16_t *p_rtc_data, const rtc_date_t *p_date )
{
	p_rtc_data[MAX3510X_REG_SECONDS] = rtc_set_hundredths( p_date->hundredths ) | rtc_set_seconds( p_date->seconds );
	p_rtc_data[MAX3510X_REG_MINS_HRS] = rtc_set_minutes( p_date->minute ) | rtc_set_hour( p_date->hour );
	p_rtc_data[MAX3510X_REG_DAY_DATE] = rtc_set_day_of_week( p_date->day_of_week ) | rtc_set_day_of_month( p_date->day_of_month );
	p_rtc_data[MAX3510X_REG_MONTH_YEAR] = rtc_set_month( p_date->month ) | rtc_set_year( p_date->year );
}

#endif // defined(MAX35103) || defined(MAX35104) || defined(MAX35101)

#if defined(MAX35104)

uint16_t max3510x_unlock(max3510x_t p_max3510x )
{
	uint16_t afe1 = max3510x_read_register(p_max3510x, MAX3510X_REG_AFE1 );
	uint16_t write_back = MAX3510X_REG_GET(AFE1_WRITEBACK,afe1);
	max3510x_write_register(p_max3510x, MAX3510X_REG_AFE1, afe1 );
	return write_back;
}
#endif

void max3510x_init( max3510x_t p_max3510x, const max3510x_registers_t * p_regs )
{

#if defined(MAX35104)

	max35104_registers_t regs_104 = p_regs->max35104_registers;
	// read/writeback AFE1 register to unlock
	uint16_t write_back = max3510x_unlock(p_max3510x);
	regs_104.afe1 |= MAX3510X_ENDIAN(MAX3510X_REG_SET(AFE1_WRITEBACK,write_back));
	max3510x_write_registers_const( p_max3510x, (const max3510x_register_t *)&regs_104, sizeof(p_regs->max35104_registers) );

#endif // #if defined(MAX35104)

	max3510x_write_registers_const( p_max3510x, (const max3510x_register_t *)&p_regs->common, sizeof(p_regs->common) );

}

uint16_t max3510x_interrupt_status( max3510x_t p_max3510x )
{
	return max3510x_read_register( p_max3510x, MAX3510X_REG_INTERRUPT_STATUS );
}

uint16_t max3510x_control_register( max3510x_t p_max3510x )
{
	return max3510x_read_register( p_max3510x, MAX3510X_REG_CONTROL );
}

uint16_t max3510x_poll_interrupt_status( max3510x_t p_max3510x )
{
	uint16_t status;
	while( !(status = max3510x_interrupt_status( p_max3510x ) ) );
	return status;
}

void max3510x_wait_for_reset_complete( max3510x_t p_max3510x )
{
	// afeter issuing max3510x_reset(), this function can be used to delay until the chip
	// has completed reset.
	while( max3510x_interrupt_status(p_max3510x) == MAX3510X_REG_INTERRUPT_STATUS_INVALID );  // the max3510x is unresponsive during reset processing
}

void max3510x_read_tof_results( max3510x_t p_max3510x, max3510x_tof_results_t *p_results )
{
	max3510x_read_registers( p_max3510x, MAX3510X_REG_WVRUP, (max3510x_register_t*)p_results, sizeof(max3510x_tof_results_t) );
}

void max3510x_read_temp_results( max3510x_t p_max3510x, max3510x_temp_results_t *p_results )
{
	max3510x_read_registers( p_max3510x, MAX3510X_REG_T1INT, (max3510x_register_t*)p_results, sizeof(max3510x_temp_results_t) );
}

void max3510x_read_direction( max3510x_t p_max3510x, max3510x_direction_t direction, max3510x_direction_result_t * p_result )
{
	max3510x_read_registers( p_max3510x, direction==max3510x_direction_up ? MAX3510X_REG_WVRUP : MAX3510X_REG_WVRDN, (max3510x_register_t*)p_result, sizeof(max3510x_direction_result_t) );
}

void max3510x_read_ratios( max3510x_t p_max3510x, uint8_t reg, uint8_t *p_t1_t2, uint8_t *p_t2_ideal )
{
	uint8_t buf[sizeof(uint8_t)*2+1];
	max3510x_read_registers(p_max3510x,reg,(max3510x_register_t*)&buf[0],sizeof(buf));
	if( p_t1_t2 )
		*p_t1_t2 = buf[2];
	if( p_t2_ideal )
		*p_t2_ideal = buf[1];
}

void max3510x_read_fixed( max3510x_t p_max3510x, uint8_t reg, max3510x_fixed_t *p_fixed )
{
	max3510x_fixed_register_t fixed_reg;
	max3510x_read_registers(p_max3510x,reg,(max3510x_register_t*)&fixed_reg,sizeof( fixed_reg ));
	*p_fixed = fixed_reg.value;
}

void max3510x_tof_up( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_TOF_UP );
}

void max3510x_tof_down( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_TOF_DOWN );
}

void max3510x_tof_diff( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_TOF_DIFF );
}

void max3510x_temperature( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_TEMPERATURE );
}

void max3510x_reset( max3510x_t p_max3510x )
{
	// the chip will become unresponsive to SPI transactions for 275us (typical) after recieving this command.
	// while unresponsive, reads to status register will return MAX3510X_REG_INTERRUPT_STATUS_INVALID
	send_opcode( p_max3510x, MAX3510X_OPCODE_RESET );
}

void max3510x_initialize( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_INITIALIZE );
}

#if !defined(MAX35104)
void max3510x_flash_configuration( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_FLASH );
}
#else // #if !defined(MAX35104)
void max3510x_bandpass_calibrate( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_BANDPASS_CALIBRATE );
}
#endif // #if !defined(MAX35104)

void max3510x_halt( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_HALT );
}

void max3510x_event_timing( max3510x_t p_max3510x, max3510x_event_timing_mode_t mode )
{
	// send event timing opcodes

	uint8_t opcode;
	switch( mode )
	{
		case max3510x_event_timing_mode_tof:
			opcode = MAX3510X_OPCODE_EVTMG2;
			break;
		case max3510x_event_timing_mode_tof_temp:
			opcode = MAX3510X_OPCODE_EVTMG1;
			break;
		case max3510x_event_timing_mode_temp:
			opcode = MAX3510X_OPCODE_EVTMG3;
			break;
		default:
			return;
	}
	send_opcode( p_max3510x, opcode );
}

#if defined(MAX35103) || defined(MAX35101)

void max3510x_ldo( max3510x_t p_max3510x, max3510x_ldo_mode_t mode )
{
	// send LDO-related opcodes (not available on the MAX35101)

	uint8_t opcode;
	switch( mode )
	{
		case max3510x_ldo_mode_timed:
			opcode = MAX3510X_OPCODE_LDO_TIMED;
			break;
		case max3510x_ldo_mode_on:
			opcode = MAX3510X_OPCODE_LDO_ON;
			break;
		case max3510x_ldo_mode_off:
			opcode = MAX3510X_OPCODE_LDO_OFF;
			break;
		default:
			return;
	}
	send_opcode( p_max3510x, opcode );
}

#endif

void max3510x_calibrate( max3510x_t p_max3510x )
{
	send_opcode( p_max3510x, MAX3510X_OPCODE_CALIBRATE );
}

float_t max3510x_ratio_to_float( uint8_t ratio )
{
	// converts a ratiometric value (t1/t2 related registers) to
	// a 32-bit floating point value

	float_t f = 0;
	static const float_t c_ratio[8] = 
	{ 
		(float_t)(1.0/128.0), (float_t)(1.0/64.0), (float_t)(1.0/32.0), (float_t)(1.0/16.0),
		(float_t)(1.0/8.0), (float_t)(1.0/4.0), (float_t)(1.0/2.0), (float_t)(1.0)
	};
	uint8_t i;
	for(i=0;i<8;i++)
	{
		if( ratio & (1<<i) )
		{
			f += c_ratio[i];
		}
	}
	return f;
}

bool max3510x_validate_measurement( const max3510x_measurement_t *p_measurement, uint8_t hit_count )
{
	// validates hit values returnd by the MAX3510x
	// this is for useful detecting SPI bus related problems.

	uint8_t i;
	uint32_t m = 0;
	uint32_t hit, last_hit = 0;
	uint32_t ave;
	for(i=0;i<hit_count;i++)
	{
		if( p_measurement->hit[i].integer & 0x8000 )
		  return false;
		hit = p_measurement->hit[i].fraction + (((uint32_t)p_measurement->hit[i].integer) << 16);
		if(i)
		{
			if( hit < last_hit )
			  return false;
		}
		last_hit = hit;
		m += hit;
	}
	// round like the chip does
	m = (m<<1) / hit_count;
	if( m & 1 )
	  m++;
	m >>= 1;
	ave = ( (((uint32_t)p_measurement->average.integer) << 16) + p_measurement->average.fraction );
	return ave == m;
}

void max3510x_set_measurement_delay( max3510x_t p_max3510x, float_t delay_us )
{
	uint16_t reg_value = (uint16_t)MAX3510X_REG_TOF_MEASUREMENT_DELAY_DLY_US(delay_us);
	if(  reg_value < MAX3510X_REG_TOF_MEASUREMENT_DELAY_DLY_MINIMUM )
		reg_value = MAX3510X_REG_TOF_MEASUREMENT_DELAY_DLY_MINIMUM;
	max3510x_write_register(p_max3510x, MAX3510X_REG_TOF_MEASUREMENT_DELAY, reg_value );
}


uint32_t max3510x_input_frequency( max3510x_fixed_t *p_calibration_value )
{
	// returns the oscillation frequency of the signal on the 4MX1 pin.
	// This fucntion assumes that the 32.768KHz oscillator period is exactly 32.768KHz
	uint32_t v = (((uint32_t)p_calibration_value->integer) << 16) + ((uint32_t)p_calibration_value->fraction);
	return v >> 1;
}

float_t max3510x_calibration_factor( float_t input_frequency )
{
	// returns the factor by which uncorrected time values based
	// on the 4MX/X1 oscillator input can be corrected.
	return (input_frequency)/((float_t)FREQ_REF);
}

int32_t max3510x_fixed_to_time( const max3510x_fixed_t *p_fixed )
{
	return (p_fixed->integer << 16) | p_fixed->fraction;
}

double_t max3510x_time_to_double( max3510x_time_t time )
{
	const double_t c = 1.0 / (FREQ_REF*65536.0);
	return (double_t)time * c;
}

max3510x_time_t max3510x_float_to_time( float_t time )
{
	const float_t c = (FREQ_REF*65536.0);
	return time * c;
}

float_t max3510x_time_to_float( max3510x_time_t time )
{
	const float_t c = 1.0 / (FREQ_REF*65536.0);
	return (float_t)time * c;
}

float_t max3510x_fixed_to_float( const max3510x_fixed_t *p_number )
{
	// convert an integer/fraction formated register value to a 32-bit float
	return max3510x_time_to_float(max3510x_fixed_to_time(p_number));
}

double_t max3510x_fixed_to_double( const max3510x_fixed_t *p_number )
{
	// convert an integer/fraction formated register value to a 64-bit float
	return max3510x_time_to_double(max3510x_fixed_to_time(p_number));
}

void max3510x_convert_temp_results( max3510x_float_temp_results_t *p_float_results, const max3510x_temp_results_t * p_results )
{
	uint8_t i;
	for(i=0;i<MAX3510X_TEMP_COUNT;i++)
	{
		p_float_results->temp[i] = max3510x_fixed_to_float( &p_results->temp[i] );
		p_float_results->ave_temp[i] = max3510x_fixed_to_float( &p_results->ave_temp[i] );
	}
}


void max3510x_convert_tof_results( max3510x_float_tof_results_t *p_float_results, const max3510x_tof_results_t * p_results )
{
	// convert all measurement results to 32-bit floats

	p_float_results->up.t1_t2 = max3510x_ratio_to_float(p_results->up.t1_t2);
	p_float_results->up.t2_ideal = max3510x_ratio_to_float(p_results->up.t2_ideal);
	p_float_results->down.t1_t2 = max3510x_ratio_to_float(p_results->down.t1_t2);
	p_float_results->down.t2_ideal = max3510x_ratio_to_float(p_results->down.t2_ideal);

	uint8_t i;
	for(i=0;i<MAX3510X_MAX_HITCOUNT;i++)
	{
		p_float_results->up.hit[i] = max3510x_fixed_to_float( &p_results->up.hit[i] );
		p_float_results->down.hit[i] = max3510x_fixed_to_float( &p_results->down.hit[i] );
	}
	p_float_results->up.average = max3510x_fixed_to_float( &p_results->up.average );
	p_float_results->down.average = max3510x_fixed_to_float( &p_results->down.average );
	p_float_results->tof_diff = max3510x_fixed_to_float( &p_results->tof_diff );

	p_float_results->tof_diff_ave = max3510x_fixed_to_float( &p_results->tof_diff_ave );

}

#if defined(MAX35103)

#pragma pack(1)
typedef struct _max3510x_flash_cmd_t
{
	uint8_t		cmd;
	uint16_t	address;
}
max3510x_flash_cmd_t;

typedef struct _max3510x_flash_t
{
	max3510x_flash_cmd_t 	cmd;
	uint16_t				value;
}
max3510x_flash_t;

#pragma pack()

void max3510x_write_flash( max3510x_t p_max3510x, uint16_t address, uint16_t value )
{
	max3510x_flash_t flash;
	flash.cmd.cmd = MAX3510X_OPCODE_WRITE_FLASH;
	flash.cmd.address = MAX3510X_ENDIAN(address);
	flash.value = value;
	max3510x_spi_xfer( p_max3510x, NULL, &flash, sizeof(flash) );
}

uint16_t max3510x_read_flash( max3510x_t p_max3510x, uint16_t address )
{
	max3510x_flash_t flash;
	flash.cmd.cmd = MAX3510X_OPCODE_READ_FLASH;
	flash.cmd.address = MAX3510X_ENDIAN(address);
	flash.value = ~0;
	max3510x_spi_xfer( p_max3510x, &flash, &flash, sizeof(flash) );
	return flash.value;
}

void max3510x_erase_flash_block( max3510x_t p_max3510x, uint16_t address )
{
	max3510x_flash_cmd_t cmd;
	cmd.cmd = MAX3510X_OPCODE_ERASE_FLASH;
	cmd.address = MAX3510X_ENDIAN(address);
	max3510x_spi_xfer( p_max3510x, NULL, &cmd, sizeof(cmd) );
}

#endif // #if defined(MAX35103)

void max3510x_enable_interrupt( max3510x_t p_max3510x, bool enable )
{
	MAX3510X_WRITE_BITFIELD(p_max3510x, CALIBRATION_CONTROL, INT_EN, enable ? MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_ENABLED : MAX3510X_REG_CALIBRATION_CONTROL_INT_EN_DISABLED );
}

uint16_t max3510x_spi_test( max3510x_t p_max3510x )
{
	// test to help validate SPI hardware and max3510x_spi_xfer()
	
	uint32_t x;
	uint16_t e = 0;
	for(x=0;x<0x10000;x++)
	{
		max3510x_write_register(p_max3510x, MAX3510X_REG_TOF_MEASUREMENT_DELAY, x );
		if( x != max3510x_read_register(p_max3510x, MAX3510X_REG_TOF_MEASUREMENT_DELAY) )
			e++;
	}
	return e;
}
	
	
void max3510x_read_hitwave_config( max3510x_hitwave_config_t *p_hitwave_config )
{
	// retreive hit wave count and index configuration.

#if defined(MAX35102)
	max3510x_register3_t hitregs;
#else
	max3510x_register5_t hitregs;
#endif
	max3510x_read_registers( NULL, MAX3510X_REG_TOF2, (max3510x_register_t*)&hitregs, sizeof(hitregs) );
	p_hitwave_config->hit_count = MAX3510X_REG_TOF2_STOP( MAX3510X_REG_GET(TOF2_STOP, hitregs.value[0] ) );

	if( p_hitwave_config->hit_count >= 1 )
		p_hitwave_config->hit_wave[0] = MAX3510X_REG_GET(TOF3_HIT1WV, hitregs.value[1] );
	if( p_hitwave_config->hit_count >= 2 )
		p_hitwave_config->hit_wave[1] = MAX3510X_REG_GET(TOF3_HIT2WV, hitregs.value[1] );
	if( p_hitwave_config->hit_count >= 3 )
		p_hitwave_config->hit_wave[2] = MAX3510X_REG_GET(TOF4_HIT3WV, hitregs.value[2] );
#if !defined(MAX35102)
	if( p_hitwave_config->hit_count >= 4 )
		p_hitwave_config->hit_wave[3] = MAX3510X_REG_GET(TOF4_HIT4WV, hitregs.value[2] );
	if( p_hitwave_config->hit_count >= 5 )
		p_hitwave_config->hit_wave[4] = MAX3510X_REG_GET(TOF5_HIT5WV, hitregs.value[3] );
	if( p_hitwave_config->hit_count >= 6 )
		p_hitwave_config->hit_wave[5] = MAX3510X_REG_GET(TOF5_HIT6WV, hitregs.value[3] );
#endif
}

max3510x_time_t max3510x_wave_period( const max3510x_hitwave_config_t *p_config, const max3510x_fixed_t *p_hitwave_times )
{
	// calculates the average period of the received wave based on the hit values and the hit wave configuration.
	// this routine requires at least two hit waves times in order to calculate the period.
	max3510x_time_t period = 0;
	if( p_config->hit_count > 1 )
	{
		uint8_t i;
		max3510x_time_t wave_delta;
		max3510x_time_t period_sum=0;
		for(i=1;i<p_config->hit_count;i++)
		{
			wave_delta = max3510x_fixed_to_time( &p_hitwave_times[i] ) - max3510x_fixed_to_time( &p_hitwave_times[i-1] );
			period_sum += wave_delta / ( p_config->hit_wave[i] - p_config->hit_wave[i-1] );
		}
		period = period_sum / (p_config->hit_count - 1);
	}
	return period;
}


int8_t max3510x_wave_shift( max3510x_time_t ref_tof, max3510x_time_t sample_tof, max3510x_time_t osc_period )
{
	// This function is used to detect tof values that differ by more than 1/2 of an oscillation period
	// when compared to a reference sample tof (likely the previous known good sample).
	// This is important to know because ultrasonic amplitude variations can cause tof values to 'shift' in time by
	// an integer number of oscillation periods and thereby cause time measurement errors.
	// 
	// return value is a signed integer that indicates how 'sample_tof' compares to 'ref_tof' in terms of oscillation period 'osc_period'.
	// negative values indicate that 'ref_tof' has shifted earlier in time compared to 'sample_tof'
	// positive values indicate that 'ref_tof' has shifted later in time compared to 'sample_tof'
	// zero indicates that no shift occured and that 'sample_tof' is valid.
	// 
	// 'ref_tof' is the time-of-flight of a correctly indexed wave
	// 'sample_tof' is the time-of-flight of an unknown wave.
	// 'osc_period' is the typical oscillation period of ref_tof and sample_tof

	int8_t shift;
	max3510x_time_t half_osc_period = osc_period >> 1;
	max3510x_time_t delta = sample_tof - ref_tof;
	max3510x_time_t abs_delta = delta>=0 ? delta : -delta;
	shift = (abs_delta + half_osc_period) / osc_period;
	shift *= delta > 0 ? 1 : -1;

	return shift;
}

void max3510x_read_thresholds( max3510x_t p_max3510x, int8_t * p_up, int8_t * p_down )
{
	// returns the T1 (aka early edge) threshold in both directions
	max3510x_register2_t t1_offsets;
	max3510x_read_registers( p_max3510x, MAX3510X_REG_TOF6, (max3510x_register_t*)&t1_offsets, sizeof(t1_offsets) );
	*p_up = MAX3510X_REG_GET(TOF6_C_OFFSETUP,t1_offsets.value[0]);
	*p_down = MAX3510X_REG_GET(TOF7_C_OFFSETDN,t1_offsets.value[1]);
}

void max3510x_write_thresholds( max3510x_t p_max3510x, int8_t up, int8_t down )
{
	// sets the T1 (aka early edge) threshold in both directions.
	// implicity sets the return threshold to zero which is fine for most applications.

	max3510x_register2_t t1_offsets;
	t1_offsets.value[0] = MAX3510X_REG_SET( TOF6_C_OFFSETUP, up );
	t1_offsets.value[1] = MAX3510X_REG_SET( TOF7_C_OFFSETDN, down );
	max3510x_write_registers( p_max3510x,MAX3510X_REG_TOF6, (max3510x_register_t*)&t1_offsets, sizeof(t1_offsets) );
}

void max3510x_read_config( max3510x_t p_max3510x, max3510x_registers_t *p_config )
{
#if defined(MAX35104)	
		max3510x_read_registers( p_max3510x, MAX3510X_REG_SWITCHER1, (max3510x_register_t*)&p_config->max35104_registers, sizeof(p_config->max35104_registers) );
#endif
		max3510x_read_registers( p_max3510x, MAX3510X_REG_TOF1, (max3510x_register_t*)&p_config->common, sizeof(p_config->common) );
}

void max3510x_write_config( max3510x_t p_max3510x, max3510x_registers_t * p_regs )
{

#if defined(MAX35104)
	// read/writeback AFE1 register to unlock
	uint16_t write_back = max3510x_unlock(p_max3510x);
	p_regs->max35104_registers.afe1 |= MAX3510X_REG_SET(AFE1_WRITEBACK,write_back);
	max3510x_write_registers( p_max3510x, MAX3510X_REG_SWITCHER1, (max3510x_register_t*)&p_regs->max35104_registers, sizeof(p_regs->max35104_registers) );

#endif // #if defined(MAX35104)

	max3510x_write_registers( p_max3510x, MAX3510X_REG_TOF1, (max3510x_register_t*)&p_regs->common, sizeof(p_regs->common) );
}

