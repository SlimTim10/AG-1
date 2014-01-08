/**
 * Icewire Technologies
 * 
 * byte: 8 bits
 * word: 16 bits
 * dword: 32 bits
 *
 */

#ifndef _SDFATLIBUTILS_H
#define _SDFATLIBUTILS_H

/* Extract bytes from word */
#define WTOB_L(w)	(uint8_t)w
#define WTOB_H(w)	(uint8_t)(w >> 8)

/* Extract bytes from dword */
#define DTOB_LL(d)	(uint8_t)d
#define DTOB_LH(d)	(uint8_t)(d >> 8)
#define DTOB_HL(d)	(uint8_t)(d >> 16)
#define DTOB_HH(d)	(uint8_t)(d >> 24)

/* Convert 2 little-endian bytes to word */
#define BTOW(a, b)	(uint32_t)(a | ((uint32_t)b << 8))

/* Convert 4 little-endian bytes to dword */
#define BTOD(a, b, c, d)	(uint32_t)(a | ((uint32_t)b << 8) | ((uint32_t)c << 16) | ((uint32_t)d << 24))

/* Constants */
enum { 
	DUMMY = 0xFF,
	MAXBYTE = 0xFF
};

#endif