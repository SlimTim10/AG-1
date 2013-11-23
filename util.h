/**
 * Written by Tim Johns.
 */

#ifndef _UTILLIB_H
#define _UTILLIB_H

/*
 * Determines the key to look for in parsing the config file and the appropriate means
 * of setting the key's value
 */
struct Setting {
	uint8_t *key;
	void (*set_value)(uint16_t);
};

/*
 * Set an array of custom defined structs for key-value pair settings
 */
void set_key_value_settings(struct Setting *_key_value_settings, uint8_t _num_key_value_settings);

/*
 * Set an array of custom defined structs for the key only settings
 */
void set_key_only_settings(struct Setting *_key_only_settings, uint8_t _num_key_only_settings);

/*
 * Find and parse config.ini file and set configuration values (range,
 * bandwidth)
 */
void get_user_config(uint8_t *data, struct fatstruct *info);

#endif