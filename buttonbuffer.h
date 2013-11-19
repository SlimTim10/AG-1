/**
 * Written by Icewire Technologies
 */

#ifndef _BUTTONBUFFER_H
#define _BUTTONBUFFER_H

#include <stdint.h>
#include <stdbool.h>

/* Types of a button press */
enum ButtonPress {
	BUTTON_NONE,
	BUTTON_TAP,
	BUTTON_HOLD,
	BUTTON_TRIPLE_TAP
};

/* Circular buffer that holds button press data */
struct ButtonPressBuffer {
	enum ButtonPress *button_presses;
	uint16_t size;
	uint16_t start;
	uint16_t count;
};

/* 
 * DESC
 * 
 * PARAMS
 */
void construct_button_press_buffer(struct ButtonPressBuffer *button_press_buffer, enum ButtonPress *button_presses, uint16_t size);

/* 
 * DESC
 * 
 * PARAMS
 */
void clear_button_press_buffer(struct ButtonPressBuffer *button_press_buffer);

/* 
 * DESC
 * 
 * PARAMS
 */
bool add_button_press(struct ButtonPressBuffer *button_press_buffer, enum ButtonPress button_press);

/* 
 * DESC
 * 
 * PARAMS
 */
bool remove_button_press(struct ButtonPressBuffer *button_press_buffer, enum ButtonPress *button_press_ret);

#endif