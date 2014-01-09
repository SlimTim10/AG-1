/**
 * Written by Tim Johns.
 * Icewire Technologies
 * 
 * Useful macros.
 * byte: 8 bits
 * word: 16 bits
 * dword: 32 bits
 *
 */

#ifndef _MACRO_H
#define _MACRO_H

/* Uncomment for easier debugging */
#define DEBUG

/* Update for new firmware versions */
#define FIRMWARE_NAME			"AG-1"
#define FIRMWARE_VERSION		"20140110"

/* Name of log files (max. 5 chars) */
#define FILE_NAME	"DATA"

/* DCO speed (MHz) */
#define CLOCK_SPEED	12

/* Infinite loop */
#define HANG()	for (;;);

/* Small delay before powering on components */
#define POWER_ON_DELAY() \
	for (uint8_t j = 0; j < CLOCK_SPEED; j++) { \
		for (uint16_t i = 0; i < 5000; i++) __no_operation(); \
	}

/* Delay between multiple LED flashes */
#define LED_FLASH_DELAY(DELAY) \
	for (uint8_t j = 0; j < CLOCK_SPEED; ++j) { \
		for (uint16_t i = 0; i < DELAY; ++i) __no_operation(); \
	}


#endif
