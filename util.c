/**
 * Written by Tim Johns.
 *
 * Utility functions used to find and parse a config.ini file for accelerometer
 * and gyroscope user-defined configuration values.
 *
 * The format for config.ini is as follows:
 *     Lines beginning with semicolons are considered comments.
 *     A line that matches /^ar *= *[0-9]+$/ is used to set the range of the
 *         accelerometer. Valid range values: 2, 6.
 *     A line that matches /^as *= *[0-9]+$/ is used to set the sample rate of
 *         the accelerometer. Valid bandwidth values: 40, 160, 640, 2560.
 *     A line that matches /^gr *= *[0-9]+$/ is used to set the range of the
 *         gyroscope. Valid range values: 250, 500, 2000.
 *     A line that matches /^gs *= *[0-9]+$/ is used to set the sample rate of
 *         the gyroscope. Valid bandwidth values: 100, 200, 400, 800.
 *
 * This file requires SDLIB for use in get_config_values.
 */

#ifndef _UTILLIB_C
#define _UTILLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "sdfat.h"
#include "util.h"
#include "main.h"

/* Size of a block in bytes */
#define BLOCK_SIZE 512

/* Max line length for a property (key-value pair or key only) */
#define MAX_PROP_LENGTH 80

/* The size of one FAT16 cluster (32KB) */
#define MAX_FILE_SIZE 0x8000

/* test */
enum MaxFileSize {
	MAX_FILE_SIZE_ENUM = 0x8000
};

extern uint8_t sectors_per_cluster;
extern uint32_t bytes_per_cluster;
extern uint32_t dir_table_offset;
extern uint32_t dir_table_size;
extern uint32_t file_cluster_offset;
extern uint32_t boot_offset;

/* Boolean type */
typedef enum { FALSE, TRUE } Bool;

/* Constants for special characters */
enum SpecialCharacters {
	/* End of file */
	EOF = 0x00,
	
	/* End of line */
	EOL = 0x0A,
	
	/* Carriage return */
	CR = 0x0D,
	
	/* Comment identifier */
	COMMENT_ID = ';',
	
	/* Key-value delimiter */
	KEY_VALUE_DELIM = '=',
	
	/* White space */
	WHITE_SPACE = ' '
};

/* States for the simple FSM */
enum State {
	/* Determine what kind of parsing state below to enter */
	IDLE_STATE,
	
	/* Line has a comment */
	COMMENT_STATE,
	
	/* Line has a key-value pair */
	KEY_VALUE_STATE,
};

/* A subsequence of consecutive chars in a char array */
struct substring {
	/* Point to the char array (i.e. original string) */
	uint8_t *string;
	/* Starting position of index in char array */
	uint8_t pos;
	/* Number of chars */
	uint8_t length;
};

/*
 * Return accelerometer range bits corresponding to range n
 * LIS3LV02DL Accelerometer
 */
uint8_t range_bits_accel(uint16_t n);

/*
 * Return accelerometer range Gs ASCII value corresponding to range bits n
 * LIS3LV02DL Accelerometer
 */
uint8_t range_ascii_accel(uint8_t n);

/*
 * Return accelerometer bandwidth bits corresponding to bandwidth n
 * LIS3LV02DL Accelerometer
 */
uint8_t bandwidth_bits_accel(uint16_t n);

/*
 * Return gyroscope range bits corresponding to range n
 * L3G4200D Gyroscope
 */
uint8_t range_bits_gyro(uint16_t n);

/*
 * Return gyroscope range DPS ASCII value corresponding to range bits n
 * L3G4200D Gyroscope
 */
uint16_t range_ascii_gyro(uint8_t n);

/*
 * Return gyroscope bandwidth bits corresponding to bandwidth n
 * L3G4200D Gyroscope
 */
uint8_t bandwidth_bits_gyro(uint16_t n);

/*
 * Convert a substring to a 16 bit unsigned integer
 *
 * substring: The substring to be converted
 * 
 * Return the converted value or, if unconvertible, 0 
 */
uint16_t substring_to_uint16_t(const struct substring *const substring);

/* 
 * Check whether string is an exact match with substring:
 * all characters in string match those in substring.
 *
 * string: The string to compare against the substring.
 *
 * substring: The substring to compare against the string.
 *
 * Return TRUE if there is an exact match, FALSE otherwise.
 */
Bool same(const uint8_t *const string, const struct substring *const substring);

/*
 * Parse and set global variables from properties in config file
 *
 * data: The raw data
 *
 * block_offset: Offset of first block with file's data
 */
void get_config_values(uint8_t *data, uint32_t block_offset);

/* 
 * Parse key-value pair properties
 *
 * line[]: The char array being parsed
 *
 * line_length: The number of relevant consecutive chars in the line
 * from the start
 */
void parse_key_value_pair(uint8_t line[], uint8_t line_length);

/* 
 * Parse key only properties
 *
 * line[]: The char array being parsed
 *
 * line_length: The number of relevant consecutive chars in the line
 * from the start
 */
void parse_key_only(uint8_t line[], uint8_t line_length);

/* 
 * Store the key in the given substring of line
 *
 * key: Substring of line for the key
 *
 * line_length: The number of relevant consecutive chars in the line
 * from the start
 */
void store_key(struct substring *key, uint8_t line_length);

/* 
 * Store the value in the given substring of line
 *
 * value: Substring of line for the value
 *
 * line_length: The number of relevant consecutive chars in the line
 * from the start
 */
void store_value(struct substring *value, uint8_t line_length);

/* TODO refactor for general use */
void set_key_value_pair_globals(struct substring *key, struct substring *value);

/* TODO refactor for general use */
void set_key_only_globals(struct substring *key);

void get_user_config(uint8_t *data, struct fatstruct *info) {
	uint32_t i, j, k;
	uint32_t config_file_offset = 0; // Offset of first block with file's data

	range_accel = DEFAULT_RANGE_ACCEL;
	bandwidth_accel = DEFAULT_BANDWIDTH_ACCEL;
	range_gyro = DEFAULT_RANGE_GYRO;
	bandwidth_gyro = DEFAULT_BANDWIDTH_GYRO;
	
	/* Read first block of directory table */
	read_block(data, info->dtoffset);

/* Find config.ini file in directory table */
	for (	i = 0;
			i < dir_table_size &&
			config_file_offset == 0 &&
			data[0] != 0x00;
			i += 512) {
		read_block(data, info->dtoffset + i);
		for (j = 0; j < 512; j += 32) {
// Deleted file
			if (data[j] == 0xE5) continue;
// End of directory table entries
			if (data[j] == 0x00) break;

			k = j;
			if (data[k] == 'C') { k++; if (data[k] == 'O') { k++;
			if (data[k] == 'N') { k++; if (data[k] == 'F') { k++;
			if (data[k] == 'I') { k++; if (data[k] == 'G') { k++;
			if (data[k] == ' ') { k++; if (data[k] == ' ') { k++;
			if (data[k] == 'I') { k++; if (data[k] == 'N') { k++;
			if (data[k] == 'I') {
			// config.ini entry found. Store starting cluster
				config_file_offset = file_cluster_offset +
									 (((uint8_t)(data[j+27] << 8) +
									 (uint8_t)(data[j+26]) - 2) *
									 bytes_per_cluster);
				break;
			}}}}}}}}}}}
		}
	}

// If the config file was found
	if (config_file_offset > 0) {
		//config_file_offset += boot_offset;
	
	// Get values from config file and set variables
		get_config_values(data, config_file_offset);
	}
}

void get_config_values(uint8_t *data, uint32_t block_offset) {
	/* Current state in the Start the FSM in idle state */
	enum State state = IDLE_STATE;

	/* Current line being parsed. Initialize all characters to 0. */
	uint8_t line[MAX_PROP_LENGTH];
	for (uint8_t i = 0; i < MAX_PROP_LENGTH; ++i) {
		line[i] = 0;
	}
	
	/* Current line's column index */
	uint8_t col_idx = 0;

	/* Read the first block */
	read_block(data, block_offset);
	
	/* 
	 * Parse file until end.
	 * Use a flag to determine when to exit the loop so we
	 * can parse the last line if it ends with EOF as opposed
	 * to EOL.
	 */
	uint16_t i = 0;
	Bool stop = FALSE;
	while (stop == FALSE) {
		/* End of the block is reached so fetch the next block */
		if (i >= BLOCK_SIZE) {
			/* Update the block offset */
			block_offset += BLOCK_SIZE;
			/* Read the next block into data[] */
			read_block(data, block_offset);
			/* Reset the counter */
			i = 0;
		} else {

			/* 
			 * TODO create step function using the following as params:
			 * (the state, the current line, the current column index for the line???, the current input value).
			 * line could be a struct containing the char *line and uint8_t col_idx???
			 * step(&state, &line, col_idx, data[i]);
			 */
			
			/* Simple FSM to handle the different kinds of line input */
			switch(state) {
				case IDLE_STATE:
					line[col_idx] = data[i];
					if (data[i] == COMMENT_ID) {
						/* The entire line is a comment */
						if (col_idx != 0) {
							/* Parse the line for keys only */
							parse_key_only(line, col_idx);
						}
						state = COMMENT_STATE;
					} else if (data[i] == EOL || data[i] == EOF) {
						/* Parse the line for keys only, checking for carriage return */
						if (col_idx > 0 && line[col_idx - 1] == CR) {
							parse_key_only(line, col_idx - 1);
						} else {
							parse_key_only(line, col_idx);
						}
						/* Stay in idle state */
					} else if (data[i] == KEY_VALUE_DELIM) {
						state = KEY_VALUE_STATE;
					}
					break;
				case COMMENT_STATE:
					if (data[i] == EOL) {
						state = IDLE_STATE;
					}
					break;
				case KEY_VALUE_STATE:
					line[col_idx] = data[i];
					if (data[i] == EOL || data[i] == EOF) {
						/* Parse the line for key-value pairs, checking for carriage return */
						if (col_idx > 0 && line[col_idx - 1] == CR) {
							parse_key_value_pair(line, col_idx - 1);
						} else {
							parse_key_value_pair(line, col_idx);
						}
						state = IDLE_STATE;
					} else if (data[i] == COMMENT_ID) {
						/* Parse the line for key-value pairs */
						parse_key_value_pair(line, col_idx);
						state = COMMENT_STATE;
					}
					break;
			}
			
			/* End of file; break out of the loop */
			if (data[i] == EOF) {
				stop = TRUE;
			} else if (data[i] == EOL) {
				/* Reset the column index for the next line */
				col_idx = 0;
			} else {
				/* Line exceeded the max line length */
				if (col_idx == MAX_PROP_LENGTH - 1) {
					/* TODO PANIC! Line is too long! */
				} else {
					++col_idx;
				}
			}
			
			/* File exceeded the max file size */
			if (i == MAX_FILE_SIZE - 1) {
				/* TODO PANIC! File is too big! */
			} else {
				++i;
			}
		}
	}
}

void parse_key_value_pair(uint8_t line[], uint8_t line_length) {
	/* Skip empty line */
	if (line_length == 0) {
		return;
	}

	/* Get the key */
	struct substring key;
	key.string = line;
	store_key(&key, line_length);
	
	/* Skip empty key */
	if (key.length == 0) {
		return;
	}
	
	/* Get the value */
	struct substring value;
	value.string = line;
	store_value(&value, line_length);
	
	/* Set global vars */
	set_key_value_pair_globals(&key, &value);
}

void parse_key_only(uint8_t line[], uint8_t line_length) {
	/* Skip empty line */
	if (line_length == 0) {
		return;
	}
	
	/* Get the key */
	struct substring key;
	key.string = line;
	store_key(&key, line_length);
	
	/* Skip empty key */
	if (key.length == 0) {
		return;
	}
	
	/* Set global vars */
	set_key_only_globals(&key);
}

void store_key(struct substring *key, uint8_t line_length) {
	/* Skip over white space before the key to find the key's starting index */
	uint8_t i = 0;
	while (key->string[i] == WHITE_SPACE) {
		++i;
	}
	key->pos = i;
	/* Determine the key's ending index, trimming white space or '=' */
	uint8_t j = i;
	while (i < line_length &&
			key->string[i] != WHITE_SPACE &&
			key->string[i] != KEY_VALUE_DELIM) {
		++i;
	}
	key->length = i-j;
}

void store_value(struct substring *value, uint8_t line_length) {
	/* Skip over key */
	uint8_t i = 0;
	while (value->string[i] != KEY_VALUE_DELIM) {
		++i;
	}
	/* Skip over the key-value delimiter */
	++i;
	/* Skip over white space before the value */
	while (value->string[i] == WHITE_SPACE) {
		++i;
	}
	value->pos = i;
	/* Determine the value's ending index, trimming white space */
	uint8_t j = i;
	while (i < line_length &&
			value->string[i] != WHITE_SPACE) {
		++i;
	}
	value->length = i-j;
}

void set_key_value_pair_globals(struct substring *key, struct substring *value) {
	/* Accelerometer range (e.g., ar=2) */
	if (same((uint8_t *)"ar", key) == TRUE) {
		range_accel = range_bits_accel(substring_to_uint16_t(value));
	} else if (same((uint8_t *)"as", key) == TRUE) {
		/* Accelerometer sample rate (e.g., as=40) */
		bandwidth_accel = bandwidth_bits_accel(substring_to_uint16_t(value));
	} else if (same((uint8_t *)"gr", key) == TRUE) {
		/* Gyroscope range (e.g., gr=2000) */
		range_gyro = range_bits_gyro(substring_to_uint16_t(value));
	} else if (same((uint8_t *)"gs", key) == TRUE) {
		/* Gyroscope sample rate (e.g., gs=100) */
		bandwidth_gyro = bandwidth_bits_gyro(substring_to_uint16_t(value));
	}
}

void set_key_only_globals(struct substring *key) {
	/* Accelerometer is disabled */
	if (same((uint8_t *)"disable_accel", key) == TRUE) {
		logger_status.accel_enabled = 0;
	} else if (same((uint8_t *)"disable_gyro", key) == TRUE) {
		/* Gyroscope is disabled */
		logger_status.gyro_enabled = 0;
	}
}

uint8_t range_bits_accel(uint16_t n) {
	if (n == 2) {
		return 0;			// 0: +/-2 g
	} else if (n == 6) {
		return 1;			// 1: +/-6 g
	} else {
		return DEFAULT_RANGE_ACCEL;
	}
}

uint8_t range_ascii_accel(uint8_t n) {
	if (n == 0) {
		return 2;
	} else {
		return 6;
	}
}

uint8_t bandwidth_bits_accel(uint16_t n) {
	if (n == 40) {
		return 0;				// 00: 40 Hz
	} else if (n == 160) {
		return 1;				// 01: 160 Hz
	} else if (n == 640) {
		return 2;				// 10: 640 Hz
	} else if (n == 2560) {
		return 3;				// 11: 2560 Hz
	} else {
		return DEFAULT_BANDWIDTH_ACCEL;
	}
}

uint8_t range_bits_gyro(uint16_t n) {
	if (n == 250) {
		return 0;			// 0: 250 dps
	} else if (n == 500) {
		return 1;			// 1: 500 dps
	} else if (n == 2000) {
		return 2;			// 2: 2000 dps
	} else {
		return DEFAULT_RANGE_GYRO;
	}
}

uint16_t range_ascii_gyro(uint8_t n) {
	if (n == 0) {
		return 250;
	} else if (n == 1) {
		return 500;
	} else {
		return 2000;
	}
}

uint8_t bandwidth_bits_gyro(uint16_t n) {
	if (n == 100) {
		return 0;				// 00: 100 Hz
	} else if (n == 200) {
		return 1;				// 01: 200 Hz
	} else if (n == 400) {
		return 2;				// 10: 400 Hz
	} else if (n == 800) {
		return 3;				// 11: 800 Hz
	} else {
		return DEFAULT_BANDWIDTH_GYRO;
	}
}

uint16_t substring_to_uint16_t(const struct substring *const substring) {
	/* Value to return */
	uint16_t n = 0;
	
	/* Max number of digits for an unsigned 16 bit integer */
	const uint8_t max_digits = 5;
	
	/* Check whether substring is too long */
	if (substring->length > max_digits) {
		return 0;
	}
	
	/* Loop through the digits to convert one-by-one from left to right */
	for (uint8_t i = 0; i < substring->length; ++i) {
		uint8_t digit = substring->string[substring->pos + i];
		if (digit < '0' || digit > '9') {
			/* Not an integer */
			return 0;
		}
		/* Convert the digit from a char to an integer */
		digit -= '0';
		/* Check whether we exceeded the maximum value, 65535, for a uint16_t */
		if (n > 6553) {
			return 0;
		} else if (n == 6553 && digit >= 6) {
			return 0;
		}
		/* Add digit to n */
		n *= 10;
		n += digit;
	}
	
	return n;
}

Bool same(const uint8_t *const string, const struct substring *const substring) {
	/* Make sure string is no longer than substring */
	if (string[substring->length] != '\0') { /* provided string is NULL terminated */
		return FALSE;
	}
	/* Make sure no characters in substring differ from string for the same index */
	for (uint8_t i = 0; i < substring->length; ++i) {
		if (string[i] != substring->string[substring->pos + i]) {
			return FALSE;
		}
	}
	/* Must be an exact match */
	return TRUE;
}

#endif