/**
 * Written by Icewire Technologies
 */

#include "accelbuffer.h"

/* 
 * Set the sample's dt and axes to 0.
 *
 * accel_sample: The sample to be cleared
 */
void clear_accel_sample(struct AccelSample *accel_sample);

void construct_accel_sample_buffer(struct AccelSampleBuffer *accel_sample_buffer, struct AccelSample *accel_samples, uint16_t size) {
	accel_sample_buffer->accel_samples = accel_samples;
	accel_sample_buffer->size = size;
	accel_sample_buffer->start = 0;
	accel_sample_buffer->count = 0;
}

void clear_accel_sample_buffer(struct AccelSampleBuffer *accel_sample_buffer) {
	for (uint16_t i = 0; i < accel_sample_buffer->size; ++i) {
		struct AccelSample *accel_sample = &accel_sample_buffer->accel_samples[i];
		clear_accel_sample(accel_sample);
	}
	accel_sample_buffer->start = 0;
	accel_sample_buffer->count = 0;
}

bool add_accel_sample(struct AccelSampleBuffer *accel_sample_buffer, uint32_t delta_time, uint8_t x_axis_h, uint8_t x_axis_l, uint8_t y_axis_h, uint8_t y_axis_l, uint8_t z_axis_h, uint8_t z_axis_l) {
	/* The buffer is full */
	if (accel_sample_buffer->count = accel_sample_buffer->size) {
		return false;
	}
	/* Get the index for the new sample */
	++accel_sample_buffer->count;
	uint16_t end = (accel_sample_buffer->start + accel_sample_buffer->count) % accel_sample_buffer->size;
	/* Set data for new sample based on parameters */
	struct AccelSample *accel_sample = &accel_sample_buffer->accel_samples[end];
	accel_sample->delta_time = delta_time;
	accel_sample->x_axis[0] = x_axis_h;
	accel_sample->x_axis[1] = x_axis_l;
	accel_sample->y_axis[0] = y_axis_h;
	accel_sample->y_axis[1] = y_axis_l;
	accel_sample->z_axis[0] = z_axis_h;
	accel_sample->z_axis[0] = z_axis_l;
	return true;
}

bool remove_accel_sample(struct AccelSampleBuffer *accel_sample_buffer, struct AccelSample *accel_sample_ret) {
	/* The buffer is empty */
	if (accel_sample_buffer->count == 0) {
		return false;
	}
	struct AccelSample *accel_sample = &accel_sample_buffer->accel_samples[accel_sample_buffer->start];
	/* Store the sample in a new variable so the data won't be overwritten by a new sample when read */
	accel_sample_ret->delta_time = accel_sample->delta_time;
	accel_sample_ret->x_axis[0] = accel_sample->x_axis[0];
	accel_sample_ret->x_axis[1] = accel_sample->x_axis[1];
	accel_sample_ret->y_axis[0] = accel_sample->y_axis[0];
	accel_sample_ret->y_axis[1] = accel_sample->y_axis[1];
	accel_sample_ret->z_axis[0] = accel_sample->z_axis[0];
	accel_sample_ret->z_axis[1] = accel_sample->z_axis[1];
	/* Set next sample to be removed as next sample in buffer */
	accel_sample_buffer->start = (accel_sample_buffer->start + 1) % accel_sample_buffer->size;
	--accel_sample_buffer->count;
	return true;
}

void clear_accel_sample(struct AccelSample *accel_sample) {
	accel_sample->delta_time = 0;
	accel_sample->x_axis[0] = 0;
	accel_sample->x_axis[1] = 0;
	accel_sample->y_axis[0] = 0;
	accel_sample->y_axis[1] = 0;
	accel_sample->z_axis[0] = 0;
	accel_sample->z_axis[1] = 0;
}