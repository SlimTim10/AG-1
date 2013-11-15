/**
 * Written by Icewire Technologies
 */

#ifndef _MAIN_H
#define _MAIN_H

/* Holds the status of all loggers: accelerometer, gyroscope */
struct logger_status {
	/* Enabled = 1; disabled = 0 */
	uint8_t accel_enabled : 1;
	uint8_t gyro_enabled : 1;
};

/*
 * Declare external variables
 */
extern struct logger_status logger_status;
extern uint8_t range_accel, bandwidth_accel;
extern uint8_t range_gyro, bandwidth_gyro;

#endif