/**
 * Written by Icewire Technologies
 */

#include "buttonbuffer.h"

void construct_button_press_buffer(struct ButtonPressBuffer *button_press_buffer, enum ButtonPress *button_presses, uint16_t size) {
	button_press_buffer->button_presses = button_presses;
	button_press_buffer->size = size;
	button_press_buffer->start = 0;
	button_press_buffer->end = 0;
	button_press_buffer->count = 0;
}

void clear_button_press_buffer(struct ButtonPressBuffer *button_press_buffer) {
	for (uint16_t i = 0; i < button_press_buffer->size; ++i) {
		button_press_buffer->button_presses[i] = BUTTON_NONE;
	}
	button_press_buffer->start = 0;
	button_press_buffer->end = 0;
	button_press_buffer->count = 0;
}

bool add_button_press(struct ButtonPressBuffer *button_press_buffer, enum ButtonPress button_press) {
	/* The buffer is full */
	if (button_press_buffer->count == button_press_buffer->size) {
		return false;
	}
	/* Set data for new button press based on parameter */
	button_press_buffer->button_presses[button_press_buffer->end] = button_press;
	/* Increment the index for the next button press */
	button_press_buffer->end = (button_press_buffer->end + 1) % button_press_buffer->size;
	++button_press_buffer->count;
	return true;
}

bool remove_button_press(struct ButtonPressBuffer *button_press_buffer, enum ButtonPress *button_press_ret) {
	/* The buffer is empty */
	if (button_press_buffer->count == 0) {
		return false;
	}
	/* Store the button press in a new variable so the data won't be overwritten by a new button press when read */
	*button_press_ret = button_press_buffer->button_presses[button_press_buffer->start];
	/* Set next button press to be removed as next button press in buffer */
	button_press_buffer->start = (button_press_buffer->start + 1) % button_press_buffer->size;
	--button_press_buffer->count;
	return true;
}