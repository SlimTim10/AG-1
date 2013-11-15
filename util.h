/**
 * Written by Tim Johns.
 */

#ifndef _UTILLIB_H
#define _UTILLIB_H

#define DEFAULT_RANGE_ACCEL			0	// Default range value (0: +/-2 g)
#define DEFAULT_BANDWIDTH_ACCEL		0	// Default bandwidth value (00: 40 Hz)
#define DEFAULT_RANGE_GYRO			1	// Default range value (01: 500 dps)
#define DEFAULT_BANDWIDTH_GYRO		0	// Default bandwidth value (00: 100 Hz)

/*
 * Find and parse config.ini file and set configuration values (range,
 * bandwidth)
 */
void get_user_config(uint8_t *data, struct fatstruct *info);

#endif