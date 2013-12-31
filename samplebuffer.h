/**
 * Written by Icewire Technologies
 */

#ifndef _SAMPLEBUFFER_H
#define _SAMPLEBUFFER_H

#include <stdint.h>
#include <stdbool.h>

/* Hold the logger's sample axes */
struct LoggerSample {
	uint8_t x_axis[2];
	uint8_t y_axis[2];
	uint8_t z_axis[2];
};

/* Hold the loggers' sample data */
struct Sample {
	uint8_t delta_time[3];
	struct LoggerSample accel;
	struct LoggerSample gyro;
};

/* Circular buffer that holds the samples */
struct SampleBuffer {
	struct Sample *samples;
	uint16_t size;
	uint16_t start;
	uint16_t end;
	uint16_t count;
};

/* 
 * Set up a new sample buffer using the provided samples.
 * 
 * sample_buffer: the buffer for holding the specified samples.
 *
 * samples: the samples to insert into the specified sample_buffer.
 *
 * size: the number of samples to be inserted.
 */
void construct_sample_buffer(struct SampleBuffer *sample_buffer, struct Sample *samples, uint16_t size);

/* 
 * Set all samples in the buffer to default values.
 * 
 * sample_buffer: the buffer for which the samples inside will be cleared
 */
void clear_sample_buffer(struct SampleBuffer *sample_buffer);

/* 
 * Insert a new sample into the buffer.
 * 
 * PARAMS TODO
 *
 * Return true if the insertion was successful, false if not
 */
bool add_sample(struct SampleBuffer *sample_buffer, uint8_t delta_time_h, uint8_t delta_time_m, uint8_t delta_time_l,
						uint8_t accel_x_axis_h, uint8_t accel_x_axis_l, uint8_t accel_y_axis_h, uint8_t accel_y_axis_l, uint8_t accel_z_axis_h, uint8_t accel_z_axis_l,
						uint8_t gyro_x_axis_h, uint8_t gyro_x_axis_l, uint8_t gyro_y_axis_h, uint8_t gyro_y_axis_l, uint8_t gyro_z_axis_h, uint8_t gyro_z_axis_l);

/* 
 * Retrieve and removes the oldest sample from the buffer.
 * 
 * PARAMS TODO
 *
 * Return true if the retrieval was successful, false if not
 */
bool remove_sample(struct SampleBuffer *sample_buffer, struct Sample *sample_ret);

#endif