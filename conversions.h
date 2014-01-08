/**
 * Written by Icewire Technologies.
 *
 * Conversions.
 */

#ifndef _CONVERSIONS_H
#define _CONVERSIONS_H

/*
 * C++ version 0.4 char* style "itoa":
 * Original written by Lukas Chmela
 * Released under GPLv3.
 *
 * Convert from 32 bit integer to ascii char array
 */
uint8_t* itoa(int32_t value, uint8_t* result);

/* 
 * Convert from 32 bit unsigned integer to ascii char array
 *
 * Adapted from the itoa function.
 */
uint8_t* uitoa(uint32_t value, uint8_t* result);

/* Convert a two-byte array to a 16 bit signed integer */
int16_t int8arr_to_int16(uint8_t *value);

/* Convert a three-byte array to a 32 bit unsigned integer */
uint32_t int8arr_to_uint32(uint8_t *value);

#endif