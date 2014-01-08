/**
 * Written by Icewire Technologies.
 * 
 * Conversions.
 *
 */

#include <stdint.h>
#include "conversions.h"
#include "const.h"

uint8_t* itoa(int32_t value, uint8_t* result) {
	uint8_t* ptr = result, *ptr1 = result, tmp_char;
	int32_t tmp_value;
	uint8_t base = 10;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "9876543210123456789" [9 + (tmp_value - value * base)];
	} while (value);

	/* Apply negative sign */
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = NULL_TERMINATOR;
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

uint8_t* uitoa(uint32_t value, uint8_t* result) {
	uint8_t* ptr = result, *ptr1 = result, tmp_char;
	uint32_t tmp_value;
	uint8_t base = 10;
	
	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "0123456789" [(tmp_value - value * base)];
	} while (value);
	
	*ptr-- = NULL_TERMINATOR;
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

int16_t int8arr_to_int16(uint8_t *value) {
	int16_t result = ((uint16_t)*value << 8) | *(value + 1);
	return result;
	
// Old method; see TODO below
//	int32_t result = (value[0] << 8) | value[1];
//	/* Check whether the number is negative by two's complement */
//	if (result >= 32768) {
//		// TODO why -0 instead of -32768? Might have to change function name...
//		result = -((~result + 1) & 0x7FFF);
//	}
//	return result;
}

uint32_t int8arr_to_uint32(uint8_t *value) {
	uint32_t result = ((uint32_t)*value << 16) | ((uint16_t)*(value + 1) << 8) | *(value + 2);
	return result;
}