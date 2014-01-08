/**
 * Written by Tim Johns.
 * Icewire Technologies
 * 
 * Constants.
 *
 */

#ifndef _CONST_H
#define _CONST_H

/* Amount of time between LED flashes in seconds when idle */
enum { IDLE_FLASH_RATE = 4 };

/* Amount of time between LED flashes in seconds when logging */
enum { LOG_FLASH_RATE = 1 };

/* Amount of time between LED flashes in seconds when waiting for format action */
enum { FORMAT_FLASH_RATE = 1 };

/* Size of raw data buffers */
//enum { RAW_SAMPLE_BUFF_SIZE = 250 };
enum { RAW_SAMPLE_BUFF_SIZE = 217 };
//enum { RAW_SAMPLE_BUFF_SIZE = 149 };

/* Should be a multiple of SD card write block (512B) */
//enum { SD_SAMPLE_BUFF_SIZE = 512 };
enum { SD_SAMPLE_BUFF_SIZE = 1024 };
//enum { SD_SAMPLE_BUFF_SIZE = 2048 };

/* 1B */
enum { BUTTON_BUFF_SIZE = 1 };

/* Time (in cycles) to wait for button debouncing */
enum { BUTTON_DEBOUNCE_TIME = 0x1000 };

/* Time (in seconds) to detect a held button press */
enum { BUTTON_HOLD_TIME = 2 };

/* Time (in seconds) to detect a successive button press */
enum { BUTTON_TIME_WINDOW = 1 };

/* ASCII character used as delimiter for logger data */
enum { DELIMITER = ',' };

/* ASCII character used to indicate that the next character starts on a new line */
enum { NEW_LINE = '\n' };

/* Indicates the end of a character array */
enum { NULL_TERMINATOR = '\0' };

#endif
