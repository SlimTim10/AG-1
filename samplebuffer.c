/**
 * Written by Icewire Technologies
 */

#include "samplebuffer.h"

/* 
 * Set the sample's dt and loggers' axes to 0.
 *
 * sample: The sample to be cleared
 */
void clear_sample(struct Sample *sample);

void construct_sample_buffer(struct SampleBuffer *sample_buffer, struct Sample *samples, uint16_t size) {
	sample_buffer->samples = samples;
	sample_buffer->size = size;
	sample_buffer->start = 0;
	sample_buffer->end = 0;
	sample_buffer->count = 0;
}

void clear_sample_buffer(struct SampleBuffer *sample_buffer) {
	for (uint16_t i = 0; i < sample_buffer->size; ++i) {
		struct Sample *sample = &sample_buffer->samples[i];
		clear_sample(sample);
	}
	sample_buffer->start = 0;
	sample_buffer->end = 0;
	sample_buffer->count = 0;
}

bool add_sample(struct SampleBuffer *sample_buffer, uint32_t delta_time, 
						uint8_t accel_x_axis_h, uint8_t accel_x_axis_l, uint8_t accel_y_axis_h, uint8_t accel_y_axis_l, uint8_t accel_z_axis_h, uint8_t accel_z_axis_l,
						uint8_t gyro_x_axis_h, uint8_t gyro_x_axis_l, uint8_t gyro_y_axis_h, uint8_t gyro_y_axis_l, uint8_t gyro_z_axis_h, uint8_t gyro_z_axis_l) {
	/* The buffer is full */
	if (sample_buffer->count == sample_buffer->size) {
		return false;
	}
	/* Set data for new sample based on parameters */
	struct Sample *sample = &sample_buffer->samples[sample_buffer->end];
	sample->delta_time = delta_time;
	/* From accelerometer */
	sample->accel.x_axis[0] = accel_x_axis_h;
	sample->accel.x_axis[1] = accel_x_axis_l;
	sample->accel.y_axis[0] = accel_y_axis_h;
	sample->accel.y_axis[1] = accel_y_axis_l;
	sample->accel.z_axis[0] = accel_z_axis_h;
	sample->accel.z_axis[1] = accel_z_axis_l;
	/* From gyroscope */
	sample->gyro.x_axis[0] = gyro_x_axis_h;
	sample->gyro.x_axis[1] = gyro_x_axis_l;
	sample->gyro.y_axis[0] = gyro_y_axis_h;
	sample->gyro.y_axis[1] = gyro_y_axis_l;
	sample->gyro.z_axis[0] = gyro_z_axis_h;
	sample->gyro.z_axis[1] = gyro_z_axis_l;
	/* Increment the index for the next sample */
	sample_buffer->end = (sample_buffer->end + 1) % sample_buffer->size;
	++sample_buffer->count;
	return true;
}

bool remove_sample(struct SampleBuffer *sample_buffer, struct Sample *sample_ret) {
	/* The buffer is empty */
	if (sample_buffer->count == 0) {
		return false;
	}
	struct Sample *sample = &sample_buffer->samples[sample_buffer->start];
	/* Store the sample in a new variable so the data won't be overwritten by a new sample when read */
	sample_ret->delta_time = sample->delta_time;
	/* From accelerometer */
	sample_ret->accel.x_axis[0] = sample->accel.x_axis[0];
	sample_ret->accel.x_axis[1] = sample->accel.x_axis[1];
	sample_ret->accel.y_axis[0] = sample->accel.y_axis[0];
	sample_ret->accel.y_axis[1] = sample->accel.y_axis[1];
	sample_ret->accel.z_axis[0] = sample->accel.z_axis[0];
	sample_ret->accel.z_axis[1] = sample->accel.z_axis[1];
	/* From gyroscope */
	sample_ret->gyro.x_axis[0] = sample->gyro.x_axis[0];
	sample_ret->gyro.x_axis[1] = sample->gyro.x_axis[1];
	sample_ret->gyro.y_axis[0] = sample->gyro.y_axis[0];
	sample_ret->gyro.y_axis[1] = sample->gyro.y_axis[1];
	sample_ret->gyro.z_axis[0] = sample->gyro.z_axis[0];
	sample_ret->gyro.z_axis[1] = sample->gyro.z_axis[1];
	/* Set next sample to be removed as next sample in buffer */
	sample_buffer->start = (sample_buffer->start + 1) % sample_buffer->size;
	--sample_buffer->count;
	return true;
}

void clear_sample(struct Sample *sample) {
	sample->delta_time = 0;
	/* From accelerometer */
	sample->accel.x_axis[0] = 0;
	sample->accel.x_axis[1] = 0;
	sample->accel.y_axis[0] = 0;
	sample->accel.y_axis[1] = 0;
	sample->accel.z_axis[0] = 0;
	sample->accel.z_axis[1] = 0;
	/* From gyroscope */
	sample->gyro.x_axis[0] = 0;
	sample->gyro.x_axis[1] = 0;
	sample->gyro.y_axis[0] = 0;
	sample->gyro.y_axis[1] = 0;
	sample->gyro.z_axis[0] = 0;
	sample->gyro.z_axis[1] = 0;
}