/**
 * Written by Icewire Technologies.
 */

#include <msp430f5310.h>
#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "sdfat.h"
#include "LIS3LV02DL.h"
#include "L3G4200D.h"
#include "util.h"
#include "msp430f5310_extra.h"
#include "circuit.h"
#include "main.h"
#include "accelbuffer.h"
#include "buttonbuffer.h"

#define DEBUG

#define CLOCK_SPEED		12		// DCO speed (MHz)

#define CTRL_TAP		0		// Button tap (shorter than hold)
#define CTRL_HOLD		1		// Button hold

// Infinite loop
#define HANG()			for (;;);


/* Size of data buffers */
/*** 4660 bytes available to use */
/* 1.5KB */
enum { RAW_SAMPLE_BUFF_SIZE = 150 };

/* 512B */
enum { ASCII_SAMPLE_BUFF_SIZE = 512 };

/* 1B */
enum { BUTTON_BUFF_SIZE = 1 };

/* Time (in cycles) to wait for button debouncing */
enum { BUTTON_DEBOUNCE_TIME = 0x1000 };

/* Time (in seconds) to detect a held button press */
enum { BUTTON_HOLD_TIME = 2 };

/* Time (in seconds) to detect a successive button press */
enum { BUTTON_TIME_WINDOW = 1 };

/* Possible states of the device */
enum DeviceState {
	OFF_STATE,
	IDLE_STATE,
	LOG_STATE
};

/*
 * External variables
 */
// extern uint32_t file_cluster_offset;
// extern uint32_t bytes_per_cluster;
// extern uint8_t sectors_per_cluster;

/*
 * Function prototypes
 */
void start_watchdog(void);
void stop_watchdog(void);
void feed_watchdog(void);
enum DeviceState init(void);
enum DeviceState turn_on(void);
enum DeviceState turn_off(void);
enum DeviceState start_logging(void);
enum DeviceState stop_logging(void);
void off_step(void);
void idle_step(void);
void log_step(void);
void button_press_event(void);
void accel_sample_event(void);
void gyro_sample_event(void);
bool timer_interrupt_triggered(void);
void clear_timer_interrupt(void);
bool button_interrupt_triggered(void);
enum ButtonPress get_button_press(bool can_triple_tap);
enum ButtonPress wait_for_button_release(void);
// old
void LED1_DOT(void);
void LED1_DASH(void);
void LED1_PANIC(void);
void LED1_LOW_VOLTAGE(void);
void morse_delay(uint8_t t);
enum ButtonPress wait_for_ctrl(void);

/*
 * Define global variables
 */
/* High byte for continuous timer */
uint32_t time_cont;

/* Time of last sample for getting delta timestamp for acceleration data for new sample */
uint32_t timestamp_accel;

/* Buffer for accelerometer samples */
struct AccelSampleBuffer accel_sample_buffer;

/* Samples for buffer of accelerometer samples */
struct AccelSample accel_samples[RAW_SAMPLE_BUFF_SIZE];

/* Buffer for button presses */
struct ButtonPressBuffer button_press_buffer;

/* Button presses for buffer of button presses */
enum ButtonPress button_presses[BUTTON_BUFF_SIZE];

/* Whether the user can triple tap */
bool triple_tap_enabled;

// old
// Firmware name and version
// struct {
	// uint8_t name[4];
	// uint8_t version[8];
// } firmware = {"AG-1", "20130715"};
// // Data buffer for R/W to SD card (also used in SDLIB and UTILLIB)
// uint8_t data[BUFF_SIZE];
// uint8_t data_accel[BUFF_SIZE];	// Buffer for acceleration data
// uint8_t data_gyro[BUFF_SIZE];	// Buffer for gyroscope data

// uint8_t new_data_accel;			// New data flag for accelerometer
// uint8_t new_data_gyro;			// New data flag for gyroscope

// /* Configuration values (initialized in UTILLIB) */
// uint8_t range_accel, bandwidth_accel;
// uint8_t range_gyro, bandwidth_gyro;

// uint8_t time_cont;				// High byte for continuous timer
// // Time tracker for getting delta timestamp for acceleration data
// uint32_t time_accel;
// uint32_t d_time_accel;			// Delta timestamp for acceleration data
// // Time tracker for getting delta timestamp for gyroscope data
// uint32_t time_gyro;
// uint32_t d_time_gyro;			// Delta timestamp for gyroscope data

// uint8_t logging;				// Set to 1 to signal device is logging
// uint8_t stop_flag;				// Set to 1 to signal stop logging

// uint8_t format_sd_flag;			// Flag to determine when to format SD card
								// // (Set in PORT1_ISR)
// /* Holds the current status of all loggers */
// struct logger_status logger_status = {
	// .accel_enabled = 1,
	// .gyro_enabled = 1
// };

void start_watchdog(void) {
#ifndef DEBUG
	wdt_config();
#endif
}

void stop_watchdog(void) {
	wdt_stop();
}

void feed_watchdog(void) {
#ifndef DEBUG
	wdt_config();
#endif
}

/* Return int for compiler compatibility */
int main(void) {
	/* Initialize upon startup */
	enum DeviceState device_state = init();

#ifdef DEBUG
	//device_state = start_logging();
#endif
	
	/* Application loop */
	while (true) {
		feed_watchdog();
		switch(device_state) {
			case OFF_STATE:
				off_step();
				break;
			case IDLE_STATE:
				idle_step();
				break;
			case LOG_STATE:
				log_step();
				break;
		}
#ifdef DEBUG
		break;
#endif
	}
	return 0;
}

enum DeviceState init(void) {
	/* Construct data buffers */
	construct_accel_sample_buffer(&accel_sample_buffer, accel_samples, RAW_SAMPLE_BUFF_SIZE);
	construct_button_press_buffer(&button_press_buffer, button_presses, BUTTON_BUFF_SIZE);
	/* Watchdog timer is on by default */
	stop_watchdog();
	/* Set up and configure the clock */
	clock_config();
	/* Configure MCU pins */
	mcu_pin_config();
	/* Set up ADC */
	/* adc_config(); TODO unnecessary? */
	return turn_off();
}

enum DeviceState turn_on(void) {
	// TODO
	/* We don't want to waste time checking for triple taps in this state */
	triple_tap_enabled = false;
	/* Clear button press buffer */
	clear_button_press_buffer(&button_press_buffer);
	return IDLE_STATE;
}

enum DeviceState turn_off(void) {
	/* Deactivate all interrupts except control button interrupt */
	deactivate_interrupts();
	activate_ctrl_interrupt();
	/* We have triple tapping features in this state */
	triple_tap_enabled = true;
	/* Clear button press buffer */
	clear_button_press_buffer(&button_press_buffer);
	enable_interrupts();
	/* Wait for control button interrupt */
	enter_LPM();
	/* Control button interrupt happened, so continue */
	exit_LPM();
	return OFF_STATE;
}

enum DeviceState start_logging(void) {
	/* Reset timer */
	time_cont = 0;
	/* Clear accelerometer samples buffer */
	clear_accel_sample_buffer(&accel_sample_buffer);
	// TODO
	
	return LOG_STATE;
}

enum DeviceState stop_logging(void) {
	// TODO
	return IDLE_STATE;
}

void off_step(void) {
	enum ButtonPress button_press;
	bool success = remove_button_press(&button_press_buffer, &button_press);
#ifdef DEBUG
	if (!success) {
		success = false;
	}
#endif
	switch(button_press) {
		case BUTTON_TAP:
			// TODO
			break;
		case BUTTON_HOLD:
			// TODO
			break;
		case BUTTON_TRIPLE_TAP:
			// TODO
			break;
	}
}

void idle_step(void) {
	// TODO
}

void log_step(void) {
	// TODO
}

/*----------------------------------------------------------------------------*/
/* Flash LED the length of a dot											  */
/*----------------------------------------------------------------------------*/
void LED1_DOT(void) {
	uint16_t i;
	uint8_t j;
	LED1_ON();
	for (j = 0; j < CLOCK_SPEED; j++) {
		for (i = 0; i < 10000; i++);
	}
	LED1_OFF();
}

/*----------------------------------------------------------------------------*/
/* Flash LED the length of a dash											  */
/*----------------------------------------------------------------------------*/
void LED1_DASH(void) {
	uint16_t i;
	uint8_t j;
	LED1_ON();
	for (j = 0; j < CLOCK_SPEED; j++) {
		for (i = 0; i < 60000; i++);
	}
	LED1_OFF();
}

/*----------------------------------------------------------------------------*/
/* Flash LED multiple times quickly to show "panic"							  */
/*----------------------------------------------------------------------------*/
void LED1_PANIC(void) {
	uint16_t i;
	uint8_t j, k;
	LED1_OFF();
	for (k = 0; k < 20; k++) {
		LED1_TOGGLE();
		for (j = 0; j < CLOCK_SPEED; j++) {
			for (i = 0; i < 8000; i++);
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Flash LED dimly multiple times to signal low voltage						  */
/*----------------------------------------------------------------------------*/
void LED1_LOW_VOLTAGE(void) {
	uint8_t i;
	uint32_t j;
	for (i = 0; i < 20; i++) {
		if (i % 2 == 0) {
			LED1_ON();
			for (j = 0; j < 0x800; j++) _NOP();
		} else {
			LED1_OFF();
			for (j = 0; j < 0x20000; j++) _NOP();
		}
	}
	LED1_OFF();
}

/*----------------------------------------------------------------------------*/
/* Morse code delay (1 = delay between signals, 2 = delay between letters)	  */
/*----------------------------------------------------------------------------*/
void morse_delay(uint8_t t) {
	uint16_t i;
	uint8_t j, k;
	LED1_OFF();
	for (j = 0; j < CLOCK_SPEED; j++) {
		for (k = 0; k < t; k++) {
			for (i = 0; i < 30000; i++);
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Wait for CTRL button to be pressed (do nothing while CTRL is low)		  */
/* Return CTRL_TAP on button tap, CTRL_HOLD on button hold.				  */
/* NOTE: This function uses the MSP430F5310 Real-Time Clock module			  */
/*----------------------------------------------------------------------------*/
// TODO refactor using wait_for_button_release()
enum ButtonPress wait_for_ctrl(void) {
	uint8_t prev_sec;			// Used for LED flashing
	uint16_t debounce;			// Used for debouncing

/* Wait for button tap while flashing LED to show ON state */
	rtc_restart();				// Restart the RTC for LED timing
	prev_sec = RTCSEC;
	debounce = 0x1000;
// Wait for button tap
	while (!ctrl_high()) {
		feed_watchdog();
		if (rtc_rdy()) {
// Only flash LED once every 2 seconds
			if (RTCSEC % 2 == 0 && RTCSEC != prev_sec) {
				LED1_DOT();		// Flash LED
				prev_sec = RTCSEC;
			}
		}
	}
	while (debounce--);			// Wait for debouncing

/* Wait until button is released or hold time (2 sec) is met */
	rtc_restart();				// Restart RTC
	uint8_t sec = RTCSEC;
	while (ctrl_high() && sec < 2) {
		feed_watchdog();
		if (rtc_rdy()) {
			sec = RTCSEC;		// Get new value
		}
	}

/* Turn off on button hold */
	if (sec >= 2) {
/* Turn on LED for 1 second to signal system turning off */
		LED1_ON();
		rtc_restart();
		while (RTCSEC < 1) {
			feed_watchdog();
		}
		return BUTTON_HOLD;		// System should turn off
	}

	return BUTTON_TAP;			// System should start logging
}

/*
 * Interrupt Service Routine triggered on Timer_A counter overflow
 * Increment high byte of timer (time_cont), using 3 bytes to keep time.
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void) {
	/* Increment high byte of timer */
	++time_cont;
	/* Clear timer interrupt flag */
	clear_timer_interrupt();
}

/*
 * Interrupt Service Routine triggered on Port 1 interrupt flag
 * This ISR handles 3 cases: accelerometer interrupt on new data, gyroscope
 * interrupt on new data, and CTRL button pressed down.
 * NOTE: This function uses the MSP430F5310 Real-Time Clock module
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
	if (button_interrupt_triggered()) {
		button_press_event();
		/* Clear CTRL button interrupt flag */
		/* TODO should we clear the flag before getting button pressed data? */
		clear_int_ctrl();
		/* Wake up from low power mode */
		LPM3_EXIT; /* TODO probably does nothing when not in low power mode */
		return;
	}
	if (accel_int()) {
		accel_sample_event();
		/* Clear accelerometer interrupt flag */
		/* TODO should we clear the flag before getting sample data? */
		clear_int_accel();
	}
	if (gyro_int()) {
		gyro_sample_event();
		/* Clear gyroscope interrupt flag */
		/* TODO should we clear the flag before getting sample data? */
		clear_int_gyro();
	}
}

void button_press_event(void) {
	// TODO should we make sure logging interrupts are disabled?
	disable_interrupts();
	/* Get button press */
	enum ButtonPress button_press = get_button_press(triple_tap_enabled);
	/* Put the button press data in the buffer */
	bool success = add_button_press(&button_press_buffer, button_press);
#ifdef DEBUG
	if (!success) {
		success = false;
	}
#endif
}

void accel_sample_event(void) {
	/* Get the timestamp */
	uint32_t timestamp = time_cont << 16 + TA0R;
	/* Let the timer interrupt run first if triggered */
	if (timer_interrupt_triggered()) {
		return;
	}
	/* Calculate the delta timestamp for sample data using previous sample's timestamp */
	uint32_t delta_time;
	if (timestamp_accel <= timestamp) {
		delta_time = timestamp - timestamp_accel;
	} else {
		delta_time = timestamp + (0x1000000 - timestamp_accel);
	}
#ifdef DEBUG
	if (delta_time > 80000 || delta_time < 60000) {
		delta_time++;
	}
#endif
	/* Update timestamp */
	timestamp_accel = timestamp;
	
	/* Put the accelerometer sample data in the buffer */
	bool success = add_accel_sample(&accel_sample_buffer, delta_time,
		read_addr_accel(ACCEL_OUTX_H), read_addr_accel(ACCEL_OUTX_L),
		read_addr_accel(ACCEL_OUTY_H), read_addr_accel(ACCEL_OUTY_L),
		read_addr_accel(ACCEL_OUTZ_H), read_addr_accel(ACCEL_OUTZ_L));
#ifdef DEBUG
	if (!success) {
		success = false;
	}
#endif
}

void gyro_sample_event(void) {
	// TODO
}

bool timer_interrupt_triggered(void) {
	if (TA0CCTL0 & (CCIFG)) {
		return true;
	}
	return false;
}

void clear_timer_interrupt(void) {
	TA0CCTL0 &= ~(CCIFG);
}

bool button_interrupt_triggered(void) {
	if (P1IV == P1IV_P1IFG1) {
		return true;
	}
	return false;
}

enum ButtonPress get_button_press(bool can_triple_tap) {
	enum ButtonPress first_button_press = wait_for_button_release();
	if (!can_triple_tap || first_button_press != BUTTON_TAP) {
		return first_button_press;
	}
	/* Check for second tap of triple tap (2 of 3) */
	rtc_restart();
	uint8_t sec = RTCSEC;
	/* User has a time window to initiate another button tap */
	while (!ctrl_high() && sec < BUTTON_TIME_WINDOW) {
		if (rtc_rdy()) {
			sec = RTCSEC;
		}
	}
	/* Button was not pressed again within the time window */
	if (!ctrl_high()) {
		return first_button_press;
	}
	enum ButtonPress second_button_press = wait_for_button_release();
	if (second_button_press != BUTTON_TAP) {
		return second_button_press;
	}
	/* Check for third tap of triple tap (3 of 3) */
	rtc_restart();
	sec = RTCSEC;
	/* User has a time window to initiate another button tap */
	while (!ctrl_high() && sec < BUTTON_TIME_WINDOW) {
		if (rtc_rdy()) {
			sec = RTCSEC;
		}
	}
	/* Button was not pressed again within the time window */
	if (!ctrl_high()) {
		return second_button_press;
	}
	enum ButtonPress third_button_press = wait_for_button_release();
	/* Triple tap achieved! */
	if (third_button_press == BUTTON_TAP) {
		return BUTTON_TRIPLE_TAP;
	}
	return third_button_press;
}

enum ButtonPress wait_for_button_release(void) {
	/* Wait for debouncing */
	for (uint16_t debounce = BUTTON_DEBOUNCE_TIME; debounce; --debounce);
	/* Wait until button is released or hold time is met */
	rtc_restart();
	uint8_t sec = RTCSEC;
	while (ctrl_high() && sec < BUTTON_HOLD_TIME) {
		feed_watchdog();
		/* Get time when RTC is ready */
		if (rtc_rdy()) {
			sec = RTCSEC;
		}
	}
	/* Button held */
	if (sec >= BUTTON_HOLD_TIME) {
		return BUTTON_HOLD;
	}
	return BUTTON_TAP;
}