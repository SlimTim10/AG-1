/**
 * Written by Icewire Technologies
 */

#ifndef _ACCELBUFFER_H
#define _ACCELBUFFER_H

#include <stdint.h>
#include <stdbool.h>

/* Hold the accelerometer's sample data */
struct AccelSample {
	uint32_t delta_time;
	uint8_t x_axis[2];
	uint8_t y_axis[2];
	uint8_t z_axis[2];
};

/* Circular buffer that holds the accelerometer samples */
struct AccelSampleBuffer {
	struct AccelSample *accel_samples;
	uint16_t size;
	uint16_t start;
	uint16_t end;
	uint16_t count;
};

/* 
 * DESC
 * 
 * PARAMS
 */
void construct_accel_sample_buffer(struct AccelSampleBuffer *accel_sample_buffer, struct AccelSample *accel_samples, uint16_t size);

/* 
 * DESC
 * 
 * PARAMS
 */
void clear_accel_sample_buffer(struct AccelSampleBuffer *accel_sample_buffer);

/* 
 * Insert a new accelerometer sample into the specified sample buffer.
 * 
 * PARAMS TODO
 *
 * Returns true if the insertion was successful, false if not
 */
bool add_accel_sample(struct AccelSampleBuffer *accel_sample_buffer, uint32_t delta_time, uint8_t x_axis_h, uint8_t x_axis_l, uint8_t y_axis_h, uint8_t y_axis_l, uint8_t z_axis_h, uint8_t z_axis_l);

/* 
 * Retrieves and removes the oldest accelerometer sample from the specified sample buffer.
 * 
 * PARAMS TODO
 *
 * Returns true if the retrieval was successful, false if not
 */
bool remove_accel_sample(struct AccelSampleBuffer *accel_sample_buffer, struct AccelSample *accel_sample_ret);

#endif