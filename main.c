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
#include "samplebuffer.h"
#include "buttonbuffer.h"

/* Uncomment for easier debugging */
// TODO uncomment to test for release as well as
// add more feed_watchdog() functions
#define DEBUG

/* Update for new firmware versions */
#define FIRMWARE_NAME			"AG-1"
#define FIRMWARE_VERSION		"20131225"

/* Name of log files (max. 5 chars) */
#define FILE_NAME	"DATA"

/* DCO speed (MHz) */
#define CLOCK_SPEED		12

/* Infinite loop */
#define HANG()			for (;;);

/* Small delay before powering on components */
#define POWER_ON_DELAY() \
	for (uint8_t j = 0; j < CLOCK_SPEED; j++) { \
		for (uint16_t i = 0; i < 5000; i++) __no_operation(); \
	}

/* Delay between multiple LED flashes */
#define LED_FLASH_DELAY(DELAY) \
	for (uint8_t j = 0; j < CLOCK_SPEED; ++j) { \
		for (uint16_t i = 0; i < DELAY; ++i) __no_operation(); \
	}

/* Amount of time between LED flashes in seconds when idle */
enum { IDLE_FLASH_RATE = 4 };

/* Amount of time between LED flashes in seconds when logging */
enum { LOG_FLASH_RATE = 1 };

/* Amount of time between LED flashes in seconds when waiting for format action */
enum { FORMAT_FLASH_RATE = 1 };

/* Size of raw data buffers */
//enum { RAW_SAMPLE_BUFF_SIZE = 250 };
enum { RAW_SAMPLE_BUFF_SIZE = 217 };
//enum { RAW_SAMPLE_BUFF_SIZE = 149 };

/* Should be a multiple of SD card write block (512B) */
//enum { SD_SAMPLE_BUFF_SIZE = 512 };
enum { SD_SAMPLE_BUFF_SIZE = 1024 };
//enum { SD_SAMPLE_BUFF_SIZE = 2048 };

/* 1B */
enum { BUTTON_BUFF_SIZE = 1 };

/* Time (in cycles) to wait for button debouncing */
enum { BUTTON_DEBOUNCE_TIME = 0x1000 };

/* Time (in seconds) to detect a held button press */
enum { BUTTON_HOLD_TIME = 2 };

/* Time (in seconds) to detect a successive button press */
enum { BUTTON_TIME_WINDOW = 1 };

/* ASCII character used as delimiter for logger data */
enum { DELIMITER = ',' };

/* ASCII character used to indicate that the next character starts on a new line */
enum { NEW_LINE = '\n' };

/* Indicates the end of a character array */
enum { NULL_TERMINATOR = '\0' };

/* Possible states of the device */
enum DeviceState {
	OFF_STATE,
	IDLE_STATE,
	LOG_STATE,
	FORMAT_STATE
};

/* Data logger settings */
struct Logger {
	bool is_enabled;
	uint8_t range;
	uint8_t bandwidth;
};

/* Buffer of data to write to SD card */
// TODO rename to SdCardFile or something
struct SdCardBuffer {
	uint8_t buffer[SD_SAMPLE_BUFF_SIZE];
	/* Current index in buffer */
	uint16_t index;
	/* First cluster index */
	uint16_t start_cluster;
	/* Current cluster index */
	uint32_t cluster;
	/* Number of blocks */
	uint8_t block_num;
	/* Total bytes */
	uint32_t size;
};

/*
 * Function prototypes
 */
void start_watchdog(void);
void stop_watchdog(void);
void feed_watchdog(void);
void power_on_sd(void);
void power_off_sd(void);
void power_on_accelerometer(void);
void power_off_accelerometer(void);
void power_on_gyroscope(void);
void power_off_gyroscope(void);
void enable_button_pressing(bool enable_button_tap_flash, bool enable_triple_tap);
void enable_accelerometer_sampling(void);
void accelerometer_empty_read(void);
bool voltage_is_low(void);
/* Flash LED multiple times quickly to show "panic" */
void led_1_panic(void);
/* Flash LED dimly multiple times to signal low voltage */
void led_1_low_voltage(void);
/* Flash LED every chosen amount of seconds determined by the RTC */
bool flash_led_at_rate(uint8_t seconds);
/* Flash LED weakly */
void led_1_weak_flash(void);
/* Flash LED strongly */
void led_1_strong_flash(void);
/* Flash LED for a longer amount of time */
void led_1_long_flash(void);

void init(void);
void restart(void);
enum DeviceState idle(void);
enum DeviceState turn_off(void);
enum DeviceState start_logging(void);
enum DeviceState stop_logging(void);
enum DeviceState format_card(void);
enum DeviceState off_step(void);
enum DeviceState idle_step(void);
enum DeviceState log_step(void);
enum DeviceState format_step(void);
void init_sd_card(void);
void format_sd_card(void);
void new_sd_card_file(struct SdCardBuffer *const sd_card_buffer);
void add_firmware_info_to_sd_card_file(struct SdCardBuffer *const sd_card_buffer);
uint32_t get_block_offset(const struct SdCardBuffer *const sd_card_buffer);
bool add_value_to_buffer(struct SdCardBuffer *const sd_card_buffer, uint8_t value);
bool write_full_buffer_to_sd_card(struct SdCardBuffer *const sd_card_buffer);

/* Only write remainder of buffer for end of file */
bool write_remaining_buffer_to_sd_card(struct SdCardBuffer *const sd_card_buffer);
void get_config_settings(void);
bool button_press_event_handled(void);
bool sample_event_handled(void);
bool timer_interrupt_triggered(void);
void clear_timer_interrupt(void);
bool button_interrupt_triggered(void);
enum ButtonPress get_button_press(bool can_triple_tap);
enum ButtonPress wait_for_button_release(void);

/*
 * Define global variables
 */
#ifdef DEBUG
uint32_t debug_int = 0;
bool debug_hit = false;
#endif

/* High byte for continuous timer */
uint8_t time_cont;

/* Time of last sample for getting delta timestamp for acceleration data for new sample */
uint32_t timestamp_accel;

/* Buffer for samples */
struct SampleBuffer sample_buffer;

/* Samples for buffer */
struct Sample samples[RAW_SAMPLE_BUFF_SIZE];

/* Buffer for button presses */
struct ButtonPressBuffer button_press_buffer;

/* Button presses for buffer of button presses */
enum ButtonPress button_presses[BUTTON_BUFF_SIZE];

/* Whether the user can triple tap */
bool triple_tap_enabled;

/* Whether there is an indication of a button tap by flashing the LED */
bool button_tap_flash_enabled;

/* Information for SD FAT library */
struct fatstruct fatinfo;

/* Buffer for accelerometer sample data to write to SD card */
struct SdCardBuffer sd_buff;

/* Temporary variable for initializing the SD card using an SdCardBuffer's buffer */
uint8_t *data_sd;

/* Accelerometer settings */
struct Logger accelerometer;

/* Gyroscope settings */
struct Logger gyroscope;

/* So that the LED doesn't flash multiple times per second */
uint8_t prev_sec = 0;

/*
 * C++ version 0.4 char* style "itoa":
 * Original written by Lukas Chmela
 * Released under GPLv3.
 */
uint8_t* itoa(int32_t value, uint8_t* result);

uint8_t* itoa(int32_t value, uint8_t* result) {
	uint8_t* ptr = result, *ptr1 = result, tmp_char;
	int32_t tmp_value;
	uint8_t base = 10;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "9876543210123456789" [9 + (tmp_value - value * base)];
	} while (value);

	/* Apply negative sign */
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = NULL_TERMINATOR;
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

/* Adapted from the itoa function */
uint8_t* uitoa(uint32_t value, uint8_t* result);

uint8_t* uitoa(uint32_t value, uint8_t* result) {
	uint8_t* ptr = result, *ptr1 = result, tmp_char;
	uint32_t tmp_value;
	uint8_t base = 10;
	
	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "0123456789" [(tmp_value - value * base)];
	} while (value);
	
	*ptr-- = NULL_TERMINATOR;
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

/* Convert a 16 bit signed number from a two-byte array */
int16_t int8arr_to_int16(uint8_t *value);

int16_t int8arr_to_int16(uint8_t *value) {
	int16_t result = ((uint16_t)*value << 8) | *(value + 1);
	return result;
	
//	// TODO old method; see TODO below
//	int32_t result = (value[0] << 8) | value[1];
//	/* Check whether the number is negative by two's complement */
//	if (result >= 32768) {
//		// TODO why -0 instead of -32768? Might have to change function name...
//		result = -((~result + 1) & 0x7FFF);
//	}
//	return result;
}

/*
 * Convert a 32 bit unsigned number from a three-byte array where the high
 * byte is padding
 */
uint32_t int8arr_to_uint32(uint8_t *value) {
	uint32_t result = ((uint32_t)*value << 16) | ((uint16_t)*(value + 1) << 8) | *(value + 2);
	return result;
}

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

void power_on_sd(void) {
	feed_watchdog();
	power_on(SD_PWR);
	/* Needs a delay to complete powering-on */
	POWER_ON_DELAY();
	/* Initialize SD card */
	/* TODO may not necessarily need to init after each power-on */
	if (init_sd() != SD_SUCCESS) {
		/* Turn the LED on and hang to indicate failure */
		led_1_on();
		HANG();
	}
}

void power_off_sd(void) {
	power_off(SD_PWR);
}

void power_on_accelerometer(void) {
	feed_watchdog();
	power_on(ACCEL_PWR);
	/* Needs a delay to complete powering-on */
	POWER_ON_DELAY();
	/* Initialize accelerometer */
	/* TODO may not necessarily need to init after each power-on */
	if (!init_accel(accelerometer.range, accelerometer.bandwidth)) {
		/* Turn the LED on and hang to indicate failure */
		led_1_on();
		HANG();
	}
}

void power_off_accelerometer(void) {
	/* So accelerometer interrupt is low */
	accelerometer_empty_read();
	power_down_accel();
	power_off(ACCEL_PWR);
}

void power_on_gyroscope(void) {
	feed_watchdog();
	power_on(GYRO_PWR);
	/* Needs a delay to complete powering-on */
	POWER_ON_DELAY();
	/* Initialize gyroscope */
	/* TODO may not necessarily need to init after each power-on */
	if (!init_gyro(gyroscope.range, gyroscope.bandwidth)) {
		/* Turn the LED on and hang to indicate failure */
		led_1_on();
		HANG();
	}
}

void power_off_gyroscope(void) {
	power_down_gyro();
	power_off(GYRO_PWR);
}

void enable_button_pressing(bool enable_button_tap_flash, bool enable_triple_tap) {
	button_tap_flash_enabled = enable_button_tap_flash;
	triple_tap_enabled = enable_triple_tap;
	/* Clear button press buffer */
	clear_button_press_buffer(&button_press_buffer);
	/* Set button press interrupt to active to wait on enable_interrupts() */
	activate_ctrl_interrupt();
}

void enable_accelerometer_sampling(void) {
	activate_accel_interrupt();
	set_int_accel();
}

// TODO possibly use set_int_accel() / clear_int_accel() instead?
void accelerometer_empty_read(void) {
	read_addr_accel(ACCEL_OUTX_H);
	read_addr_accel(ACCEL_OUTX_L);
	read_addr_accel(ACCEL_OUTY_H);
	read_addr_accel(ACCEL_OUTY_L);
	read_addr_accel(ACCEL_OUTZ_H);
	read_addr_accel(ACCEL_OUTZ_L);
}

bool voltage_is_low(void) {
	uint16_t voltage = adc_read();
	if (voltage < VOLTAGE_THRSHLD) {
		/* Show low voltage with LED 1 */
		led_1_low_voltage();
		return true;
	}
	return false;
}

void led_1_panic(void) {
	led_1_off();
	for (uint8_t k = 0; k < 20; k++) {
		led_1_toggle();
		for (uint8_t j = 0; j < CLOCK_SPEED; j++) {
			for (uint16_t i = 0; i < 8000; i++);
		}
	}
}

void led_1_low_voltage(void) {
	for (uint8_t i = 0; i < 20; i++) {
		if (i % 2 == 0) {
			led_1_on();
			LED_FLASH_DELAY(170);
		} else {
			led_1_off();
			LED_FLASH_DELAY(10922);
		}
	}
	led_1_off();
}

bool flash_led_at_rate(uint8_t seconds) {
	if (rtc_rdy()) {
		if (RTCSEC % seconds == 0 && RTCSEC != 0 && RTCSEC != prev_sec) {
			prev_sec = RTCSEC;
			return true;
		}
	}
	return false;
}

void led_1_weak_flash(void) {
	led_1_on();
	LED_FLASH_DELAY(1000);
	led_1_off();
}

void led_1_strong_flash(void) {
	led_1_on();
	LED_FLASH_DELAY(10000);
	led_1_off();
}

void led_1_long_flash(void) {
	led_1_on();
	LED_FLASH_DELAY(60000);
	led_1_off();
}

/* Return int for compiler compatibility */
int main(void) {
	/* Initialize upon startup */
	init();
	/* Start in idle since resetting runs the firmware updater which runs this */
	enum DeviceState device_state = idle();
	/* Application loop */
	while (true) {
		feed_watchdog();
		switch(device_state) {
			case OFF_STATE:
				device_state = off_step();
				break;
			case IDLE_STATE:
				device_state = idle_step();
				break;
			case LOG_STATE:
				device_state = log_step();
				break;
			case FORMAT_STATE:
				device_state = format_step();
				break;
		}
	}
	return 0;
}

void init(void) {
	/* Construct data buffers */
	construct_sample_buffer(&sample_buffer, samples, RAW_SAMPLE_BUFF_SIZE);
	construct_button_press_buffer(&button_press_buffer, button_presses, BUTTON_BUFF_SIZE);
	/* Point pointer to buffer */
	data_sd = sd_buff.buffer;
	/* Watchdog timer is on by default */
	stop_watchdog();
	/* Set up and configure the clock */
	clock_config();
	/* Configure MCU pins */
	mcu_pin_config();
	/* Set up ADC */
	adc_config();
	/* Set up SPI for MCU */
	spi_config();
	/* Start the watchdog */
	start_watchdog();
	/* Initialize the SD card */
	init_sd_card();
	/* Deactivate all interrupts */
	deactivate_interrupts();
	/* Start the timer */
	timer_config();
}

void restart(void) {
	/* Trigger a brownout reset */
	brownout_reset();
}

enum DeviceState idle(void) {
	/* Make sure the LED is off */
	led_1_off();
	disable_interrupts();
	/*
	 * We don't want to waste time checking for triple taps when device
	 * is on as we don't have any features which require triple tapping
	 */
	enable_button_pressing(true, false);
	/* Set up the clock to flash the LED */
	rtc_restart();
	prev_sec = RTCSEC;
	enable_interrupts();
	return IDLE_STATE;
}

enum DeviceState turn_off(void) {
	/* Make sure the LED is off */
	led_1_off();
	disable_interrupts();
	/* We have features which require triple tapping when device is off */
	enable_button_pressing(false, true);
	enable_interrupts();
	return OFF_STATE;
}

enum DeviceState start_logging(void) {
	/* Turn on power to SD card */
	power_on_sd();
	/* Check for low voltage */
	if (voltage_is_low()) {
		restart();
	}
	/*  
	 * We parse the config file each time we want to start logging so the user doesn't
	 * have to restart the device manually each time they modify the config settings.
	 * NOTE if this ever proves too slow or too power hungry, we can do this in device
	 * turn on state.
	 */
	get_config_settings();
	disable_interrupts();
	enable_button_pressing(true, false);
	/* Power on logging devices and activate interrupts */
	/* Accelerometer is always turned on since we use its interrupt to grab samples */
	{
		power_on_accelerometer();
		enable_accelerometer_sampling();
	}
	if (gyroscope.is_enabled) {
		power_on_gyroscope();
	}
	new_sd_card_file(&sd_buff);
	/* Clear raw samples buffer */
	clear_sample_buffer(&sample_buffer);
	/* Reset timer */
	time_cont = 0;
	/* Reset time of last sample */
	timestamp_accel = 0;
	/* Set up the clock to flash the LED */
	rtc_restart();
	prev_sec = RTCSEC;
	/* Start capturing samples */
	enable_interrupts();
	return LOG_STATE;
}

enum DeviceState stop_logging(void) {
	/* Power off logging devices */
	{
		power_off_accelerometer();
	}
	if (gyroscope.is_enabled) {
		power_off_gyroscope();
	}
	/* Write final logger data in buffer and update the directory table */
	{
		write_remaining_buffer_to_sd_card(&sd_buff);
		/* Name of log file */
		uint8_t file_name[] = FILE_NAME;
		/* Get the number of the last log file */
		uint16_t file_num = get_file_num(sd_buff.buffer, &fatinfo, file_name);
		if (update_dir_table(sd_buff.buffer, 
									&fatinfo, 
									sd_buff.start_cluster, 
									sd_buff.size,
									file_name,
									file_num) != FAT_SUCCESS) {
#ifdef DEBUG
			HANG();
#endif
			// TODO logging error
		}
	}
	/* Turn off power to SD card since writing is complete */
	power_off_sd();
	return idle();
}

enum DeviceState format_card(void) {
	disable_interrupts();
	/* No triple tapping feature in this state */
	enable_button_pressing(true, false);
	enable_interrupts();
	return FORMAT_STATE;
}

enum DeviceState off_step(void) {
	/* Wait for button press in low power mode */
	enter_LPM();
	/* Button press happened, so continue */
	exit_LPM();
	/* Get the button press */
	enum ButtonPress button_press;
	bool success = remove_button_press(&button_press_buffer, &button_press);
	if (!success) {
#ifdef DEBUG
		HANG();
#endif
	} else {
		/* Change state based on button press */
		switch(button_press) {
			case BUTTON_TAP:
				/* Try again for a different type of button press */
				return turn_off();
			case BUTTON_HOLD:
				restart();
				break;
			case BUTTON_TRIPLE_TAP:
				/* Format the SD card */
				return format_card();
		}
	}
	return OFF_STATE;
}

enum DeviceState idle_step(void) {
	if (flash_led_at_rate(IDLE_FLASH_RATE)) {
		led_1_weak_flash();
	}
	/* Check for any button presses */
	if (button_press_buffer.count > 0) {
		enum ButtonPress button_press;
		bool success = remove_button_press(&button_press_buffer, &button_press);
		if (!success) {
#ifdef DEBUG
			HANG();
#endif
		} else {
			switch(button_press) {
				case BUTTON_TAP:
					return start_logging();
				case BUTTON_HOLD:
					return turn_off();
					break;
			}
		}
	}
	return IDLE_STATE;
}

enum DeviceState log_step(void) {
#ifndef DEBUG
	if (flash_led_at_rate(LOG_FLASH_RATE)) {
		led_1_strong_flash();
	}
#endif
	/* Process samples */
	// TODO refactor this to its own function but for now...
	/* Convert all current samples in raw buffer to ascii */
	uint16_t count = sample_buffer.count;
#ifdef DEBUG
	if (count == 0) {
		led_1_off();
	} else if (count == RAW_SAMPLE_BUFF_SIZE) {
		led_1_on();
	} else {
		led_1_toggle();
	}
#endif
	for (uint16_t i = 0; i < count; ++i) {
		/* Grab a raw accelerometer sample */
		struct Sample sample;
		bool success = remove_sample(&sample_buffer, &sample);
		if (!success) {
#ifdef DEBUG
			HANG();
#endif
		} else {
			// TODO dear god, refactor this...
			/* Sample is written on a new line */
			if (!add_value_to_buffer(&sd_buff, NEW_LINE)) {
				return stop_logging();
			}
			/* Convert delta time to ascii and put in SD card buffer */
			{
				int32_t delta_time = int8arr_to_uint32(sample.delta_time);
				/* Max timestamp value is 8 digits */
				uint8_t ascii_buffer[8];
				uitoa(delta_time, ascii_buffer);
				for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 8; ++i) {
					if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
						return stop_logging();
					}
				}
			}
			if (accelerometer.is_enabled) {
				/* Add delimiter */
				if (!add_value_to_buffer(&sd_buff, DELIMITER)) {
					return stop_logging();
				}
				/* Convert axes to ascii and put in SD card buffer */
				{
					/* Max axis value is 5 digits plus sign */
					uint8_t ascii_buffer[6];
					/* X-axis */
					{
						int16_t x_axis = int8arr_to_int16(sample.accel.x_axis);
						itoa(x_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_buff, DELIMITER)) {
						return stop_logging();
					}
					/* Y-axis */
					{
						int16_t y_axis = int8arr_to_int16(sample.accel.y_axis);
						itoa(y_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_buff, DELIMITER)) {
						return stop_logging();
					}
					/* Z-axis */
					{
						int16_t z_axis = int8arr_to_int16(sample.accel.z_axis);
						itoa(z_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
				}
			}
			if (gyroscope.is_enabled) {
				/* Add delimiter */
				if (!add_value_to_buffer(&sd_buff, DELIMITER)) {
					return stop_logging();
				}
				/* Convert axes to ascii and put in SD card buffer */
				{
					/* Max axis value is 5 digits plus sign */
					uint8_t ascii_buffer[6];
					/* X-axis */
					{
						int16_t x_axis = int8arr_to_int16(sample.gyro.x_axis);
						itoa(x_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_buff, DELIMITER)) {
						return stop_logging();
					}
					/* Y-axis */
					{
						int16_t y_axis = int8arr_to_int16(sample.gyro.y_axis);
						itoa(y_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_buff, DELIMITER)) {
						return stop_logging();
					}
					/* Z-axis */
					{
						int16_t z_axis = int8arr_to_int16(sample.gyro.z_axis);
						itoa(z_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_buff, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
				}
			}
		}
	}
	/* Check for any button presses */
	if (button_press_buffer.count > 0) {
		enum ButtonPress button_press;
		bool success = remove_button_press(&button_press_buffer, &button_press);
		if (!success) {
#ifdef DEBUG
			HANG();
#endif
		} else {
			switch(button_press) {
				case BUTTON_TAP:
				case BUTTON_HOLD:
					return stop_logging();
			}
		}
	}
	return LOG_STATE;
}

enum DeviceState format_step(void) {
	if (flash_led_at_rate(FORMAT_FLASH_RATE)) {
		led_1_weak_flash();
		LED_FLASH_DELAY(30000);
		led_1_weak_flash();
	}
	/* Check for any button presses */
	if (button_press_buffer.count > 0) {
		enum ButtonPress button_press;
		bool success = remove_button_press(&button_press_buffer, &button_press);
		if (!success) {
#ifdef DEBUG
			HANG();
#endif
		} else {
			/* Perform action based on button press */
			switch(button_press) {
				case BUTTON_TAP:
					return turn_off();
					break;
				case BUTTON_HOLD:
					format_sd_card();
					break;
			}
		}
	}
	return FORMAT_STATE;
}

void init_sd_card(void) {
	/* Turn on power to SD Card during initialization */
	power_on_sd();
	feed_watchdog();
	/* Find and read the FAT16 boot sector */
	if (valid_boot_sector(data_sd, &fatinfo) != FAT_SUCCESS) {
		/* Turn the LED on and hang to indicate failure */
		led_1_on();
		HANG();
	}
	feed_watchdog();
	/* Parse the FAT16 boot sector */
	if (parse_boot_sector(data_sd, &fatinfo) != FAT_SUCCESS) {
		/* Show failure with LED 1  */
		led_1_panic();
		/* Restart upon failure */
		restart();
	}
	/* Turn back off power to SD card since initialization is complete */
	power_off_sd();
}

void format_sd_card(void) {
	/* Turn on power to SD Card */
	power_on_sd();
	/* Check for low voltage */
	if (voltage_is_low()) {
		restart();
	}
	/* Format the SD card, using LED 1 to indicate it's being formatted */
	format_sd(data_sd, &fatinfo, led_1_on, led_1_toggle, led_1_off);
	restart();
}

void new_sd_card_file(struct SdCardBuffer *const sd_card_buffer) {
	sd_card_buffer->start_cluster = find_cluster(sd_card_buffer->buffer, &fatinfo);
	/* The SD card is full */
	if (!sd_card_buffer->start_cluster) {
#ifdef DEBUG
		HANG();
#endif
		// TODO SD card full
	}
	sd_card_buffer->index = 0;
	sd_card_buffer->cluster = sd_card_buffer->start_cluster;
	sd_card_buffer->block_num = 0;
	sd_card_buffer->size = 0;
	/* Firmware info */
	add_firmware_info_to_sd_card_file(sd_card_buffer);
	sd_card_buffer->buffer[sd_card_buffer->index++] = NEW_LINE;
	/* Sample rate */
	sd_card_buffer->buffer[sd_card_buffer->index++] = 's';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'a';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'm';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'p';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'l';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'e';
	sd_card_buffer->buffer[sd_card_buffer->index++] = '-';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'r';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'a';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 't';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'e';
	sd_card_buffer->buffer[sd_card_buffer->index++] = ':';
	sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
	/* Convert the sample rate to ascii */
	{
		uint8_t ascii_buffer[3];
		itoa(bandwidth_bits_to_hz_accel(accelerometer.bandwidth), ascii_buffer);
		for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 3; ++i) {
			sd_card_buffer->buffer[sd_card_buffer->index++] = ascii_buffer[i];
		}
	}
	sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'H';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'z';
	sd_card_buffer->buffer[sd_card_buffer->index++] = NEW_LINE;
	/* Range setting */
	if (accelerometer.is_enabled) {
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'a';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'c';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'c';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'e';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'l';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'r';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'a';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'n';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'g';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'e';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ':';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '+';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '/';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '-';
		sd_card_buffer->buffer[sd_card_buffer->index++] = range_bits_to_g_accel(accelerometer.range) + 0x30;
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'g';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '(';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '+';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '/';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '-';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '3';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '2';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '7';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '6';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '8';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ')';
		sd_card_buffer->buffer[sd_card_buffer->index++] = NEW_LINE;
	}
	if (gyroscope.is_enabled) {
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'g';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'y';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'r';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'o';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'r';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'a';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'n';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'g';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'e';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ':';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '+';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '/';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '-';
		/* Convert the range to ascii */
		{
			uint8_t ascii_buffer[4];
			itoa(range_bits_to_dps_gyro(gyroscope.range), ascii_buffer);
			for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 4; ++i) {
				sd_card_buffer->buffer[sd_card_buffer->index++] = ascii_buffer[i];
			}
		}
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'd';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'p';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 's';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '(';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '+';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '/';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '-';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '3';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '2';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '7';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '6';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '8';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ')';
		sd_card_buffer->buffer[sd_card_buffer->index++] = NEW_LINE;
	}
	/* delta-time units */
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'd';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 't';
	sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'u';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'n';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'i';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 't';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 's';
	sd_card_buffer->buffer[sd_card_buffer->index++] = ':';
	sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
	sd_card_buffer->buffer[sd_card_buffer->index++] = '8';
	sd_card_buffer->buffer[sd_card_buffer->index++] = '3';
	sd_card_buffer->buffer[sd_card_buffer->index++] = '.';
	sd_card_buffer->buffer[sd_card_buffer->index++] = '3';
	sd_card_buffer->buffer[sd_card_buffer->index++] = '3';
	sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'n';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 's';
	sd_card_buffer->buffer[sd_card_buffer->index++] = NEW_LINE;
	/* Column titles */
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'd';
	sd_card_buffer->buffer[sd_card_buffer->index++] = 't';
	if (accelerometer.is_enabled) {
		sd_card_buffer->buffer[sd_card_buffer->index++] = ',';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'a';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'c';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'c';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'e';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'l';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '(';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'x';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ',';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'y';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ',';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'z';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ')';
	}
	if (gyroscope.is_enabled) {
		sd_card_buffer->buffer[sd_card_buffer->index++] = ',';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'g';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'y';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'r';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'o';
		sd_card_buffer->buffer[sd_card_buffer->index++] = '(';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'x';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ',';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'y';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ',';
		sd_card_buffer->buffer[sd_card_buffer->index++] = 'z';
		sd_card_buffer->buffer[sd_card_buffer->index++] = ')';
	}
}

void add_firmware_info_to_sd_card_file(struct SdCardBuffer *const sd_card_buffer) {
	uint8_t name[] = FIRMWARE_NAME;
	uint8_t version[] = FIRMWARE_VERSION;
	/* Add firmware name */
	for (uint8_t i = 0; name[i] != NULL_TERMINATOR; ++i) {
		sd_card_buffer->buffer[sd_card_buffer->index++] = name[i];
	}
	sd_card_buffer->buffer[sd_card_buffer->index++] = ' ';
	/* Add firmware version */
	sd_card_buffer->buffer[sd_card_buffer->index++] = 'v';
	for (uint8_t i = 0; version[i] != NULL_TERMINATOR; ++i) {
		sd_card_buffer->buffer[sd_card_buffer->index++] = version[i];
	}
}

uint32_t get_block_offset(const struct SdCardBuffer *const sd_card_buffer) {
	uint32_t block_offset = sd_card_buffer->block_num;
	block_offset *= BLKSIZE;
	block_offset += get_cluster_offset(sd_card_buffer->cluster, &fatinfo);
	return block_offset;
}

bool add_value_to_buffer(struct SdCardBuffer *const sd_card_buffer, uint8_t value) {
	sd_card_buffer->buffer[sd_card_buffer->index++] = value;
	if (!write_full_buffer_to_sd_card(sd_card_buffer)) {
		return false;
	}
	return true;
}

bool write_full_buffer_to_sd_card(struct SdCardBuffer *const sd_card_buffer) {
	if (sd_card_buffer->index > SD_SAMPLE_BUFF_SIZE) {
		/* Something went very wrong... */
#ifdef DEBUG
		HANG();
#endif
		return false;
	}
	/* Buffer is full so write it to the SD card */
	if (sd_card_buffer->index == SD_SAMPLE_BUFF_SIZE) {
		/* Write entire buffer to SD card */
		uint32_t block_offset = get_block_offset(sd_card_buffer);
		uint8_t blocks = (uint16_t)SD_SAMPLE_BUFF_SIZE / BLKSIZE;
		if (write_multiple_block(sd_card_buffer->buffer, block_offset, blocks) != SD_SUCCESS) {
			/* Couldn't write blocks */
#ifdef DEBUG
			HANG();
#endif
			return false;
		}
		/* Prepare for writing next block */
		sd_card_buffer->size += SD_SAMPLE_BUFF_SIZE;
		sd_card_buffer->index = 0;
		sd_card_buffer->block_num += blocks;
		
		/* Cluster is full */
		if (!valid_block(sd_card_buffer->block_num, &fatinfo)) {
			/* Find another cluster */
			uint16_t next_cluster = find_cluster(sd_card_buffer->buffer, &fatinfo);
			if (!next_cluster) {
				/* Couldn't find another cluster; SD card is full */
#ifdef DEBUG
				HANG();
#endif
				return false;
			}
			/* Update the FAT */
			if (update_fat(sd_card_buffer->buffer, &fatinfo, sd_card_buffer->cluster * 2, next_cluster)) {
				/* Couldn't update FAT */
#ifdef DEBUG
				HANG();
#endif
				return false;
			}
			sd_card_buffer->cluster = next_cluster;
			sd_card_buffer->block_num = 0;
		}
	}
	return true;
}

bool write_remaining_buffer_to_sd_card(struct SdCardBuffer *const sd_card_buffer) {
	if (sd_card_buffer->index > SD_SAMPLE_BUFF_SIZE) {
		/* Something went very wrong... */
#ifdef DEBUG
		HANG();
#endif
		return false;
	}
	/* Write the remaining buffer data to SD card */
	if (sd_card_buffer->index > BLKSIZE) {
		/*
		 * Do a multi block write followed by a single block write if not all
		 * bytes were written: remaining bytes were not a multiple of BLKSIZE
		 */
		uint32_t block_offset = get_block_offset(sd_card_buffer);
		uint8_t blocks = sd_card_buffer->index / BLKSIZE;
		if (write_multiple_block(sd_card_buffer->buffer, block_offset, blocks) != SD_SUCCESS) {
			/* Couldn't write blocks */
#ifdef DEBUG
			HANG();
#endif
			return false;
		}
		uint16_t bytes_written = blocks * BLKSIZE;
		sd_card_buffer->size += bytes_written;
		sd_card_buffer->index = sd_card_buffer->index - bytes_written;
		sd_card_buffer->block_num += blocks;
		
		/* Now write the single block */
		if (sd_card_buffer->index > 0) {
			/* Place bytes at beginning of buffer */
			for (uint16_t i = 0; i < sd_card_buffer->index; ++i) {
				sd_card_buffer->buffer[i] = sd_card_buffer->buffer[bytes_written + i];
			}
			/* Write the block */
			block_offset = get_block_offset(sd_card_buffer);
			if (write_block(sd_card_buffer->buffer, block_offset, sd_card_buffer->index) != SD_SUCCESS) {
	#ifdef DEBUG
				HANG();
	#endif
				return false;
			}
			sd_card_buffer->size += sd_card_buffer->index;
		}
	} else {
		/* Write the single block */
		uint32_t block_offset = get_block_offset(sd_card_buffer);
		if (write_block(sd_card_buffer->buffer, block_offset, sd_card_buffer->index) != SD_SUCCESS) {
			/* Couldn't write block */
#ifdef DEBUG
			HANG();
#endif
			return false;
		}
		sd_card_buffer->size += sd_card_buffer->index;
	}
	return true;
}

void set_sample_rate(uint16_t bandwidth) {
	/* We don't support higher sample rates */
	if (bandwidth > 640) {
		bandwidth = 0;
	}
	accelerometer.bandwidth = bandwidth_bits_accel(bandwidth);
	/* 
	 * Set gyroscope's bandwidth slightly higher than the accelerometer so when
	 * we grab the next accelerometer sample, it's always a new gyroscope sample
	 */
	uint16_t bandwidth_hz = bandwidth_bits_to_hz_accel(accelerometer.bandwidth);
	if (bandwidth_hz == 40) {
		bandwidth = 100;
	} else if (bandwidth_hz == 160) {
		bandwidth = 200;
	} else if (bandwidth_hz == 640) {
		bandwidth = 800;
	}
	gyroscope.bandwidth = bandwidth_bits_gyro(bandwidth);
}

void set_range_accel(uint16_t range) {
	accelerometer.range = range_bits_accel(range);
}

void set_range_gyro(uint16_t range) {
	gyroscope.range = range_bits_gyro(range);
}

void set_disabled_accel(uint16_t disabled) {
	if (disabled == 1) {
		accelerometer.is_enabled = false;
	} else {
		accelerometer.is_enabled = true;
	}
}

void set_disabled_gyro(uint16_t disabled) {
	if (disabled == 1) {
		gyroscope.is_enabled = false;
	}	else {
		gyroscope.is_enabled = true;
	}
}

void get_config_settings(void) {
	/* Set default settings */
	accelerometer.is_enabled = true;
	accelerometer.range = DEFAULT_RANGE_ACCEL;
	accelerometer.bandwidth = DEFAULT_BANDWIDTH_ACCEL;
	gyroscope.is_enabled = true;
	gyroscope.range = DEFAULT_RANGE_GYRO;
	gyroscope.range = DEFAULT_BANDWIDTH_GYRO;
	/* Override defaults with settings from config file */
	struct Setting key_value_settings[4] = {
		{ .key = (uint8_t *)"sr", .set_value = set_sample_rate },
		{ .key = (uint8_t *)"ar", .set_value = set_range_accel },
		{ .key = (uint8_t *)"gr", .set_value = set_range_gyro }
	};
	struct Setting key_only_settings[2] = {
		{ .key = (uint8_t *)"disable_accel", .set_value = set_disabled_accel },
		{ .key = (uint8_t *)"disable_gyro", .set_value = set_disabled_gyro }
	};
	set_key_value_settings(key_value_settings, 4);
	set_key_only_settings(key_only_settings, 2);
	get_user_config(data_sd, &fatinfo);
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
 * This ISR handles 2 cases: accelerometer interrupt on new data
 * and CTRL button pressed down.
 * NOTE: This function uses the MSP430F5310 Real-Time Clock module
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
	if (button_interrupt_triggered()) {
		/* Clear the flag */
		clear_int_ctrl();
		if (button_press_event_handled()) {
			/* Wake up from low power mode; does nothing if not in low power mode */
			LPM3_EXIT;
			return;
		}
	}
	if (accel_int()) {
		//clear_int_accel(); // TODO unnecessary?
		if (!sample_event_handled()) {
			/* Trigger interrupt to try again to capture current sample */
			set_int_accel();
		}
	}
}

bool button_press_event_handled(void) {
	/* Deactivate interrupts to prevent additional button presses and end sampling */
	deactivate_interrupts();
	/* Get button press */
	enum ButtonPress button_press = get_button_press(triple_tap_enabled);
	/* Put the button press data in the buffer */
	bool success = add_button_press(&button_press_buffer, button_press);
	if (!success) {
#ifdef DEBUG
		HANG();
#endif
		return false;
	}
	/* Since we successfully received a button press, indicate with the LED */
	switch(button_press) {
		case BUTTON_TAP:
			if (button_tap_flash_enabled) {
				led_1_strong_flash();
			}
			break;
		case BUTTON_HOLD:
			led_1_long_flash();
			break;
		case BUTTON_TRIPLE_TAP:
			led_1_strong_flash();
			LED_FLASH_DELAY(10000);
			led_1_strong_flash();
			LED_FLASH_DELAY(10000);
			led_1_strong_flash();
			break;
	}
	return true;
}

bool sample_event_handled(void) {
	/* Get the timestamp */
	uint32_t timestamp = time_cont;
	timestamp <<= 16;
	timestamp += TA0R;
	/* Let the timer interrupt run first and then capture sample */
	if (timer_interrupt_triggered()) {
		debug_hit = true;
		return false;
	}
	debug_hit = false;
	/* Calculate the delta timestamp for sample data using previous sample's timestamp */
	uint32_t delta_time;
	if (timestamp_accel <= timestamp) {
		delta_time = timestamp - timestamp_accel;
	} else {
		delta_time = timestamp + (0x1000000 - timestamp_accel);
	}
	/* Convert delta time to 3 bytes */
	uint8_t delta_time_h = delta_time >> 16;
	uint8_t delta_time_m = delta_time >> 8;
	uint8_t delta_time_l = delta_time;
	/* Get accelerometer sample data */
	uint8_t accel_x_axis_h = 0;
	uint8_t accel_x_axis_l = 0;
	uint8_t accel_y_axis_h = 0;
	uint8_t accel_y_axis_l = 0;
	uint8_t accel_z_axis_h = 0;
	uint8_t accel_z_axis_l = 0;
	{
		accel_x_axis_h = read_addr_accel(ACCEL_OUTX_H);
		accel_x_axis_l = read_addr_accel(ACCEL_OUTX_L);
		accel_y_axis_h = read_addr_accel(ACCEL_OUTY_H);
		accel_y_axis_l = read_addr_accel(ACCEL_OUTY_L);
		accel_z_axis_h = read_addr_accel(ACCEL_OUTZ_H);
		accel_z_axis_l = read_addr_accel(ACCEL_OUTZ_L);
	}
	/* Get gyroscope sample data */
	uint8_t gyro_x_axis_h = 0;
	uint8_t gyro_x_axis_l = 0;
	uint8_t gyro_y_axis_h = 0;
	uint8_t gyro_y_axis_l = 0;
	uint8_t gyro_z_axis_h = 0;
	uint8_t gyro_z_axis_l = 0;
	if (gyroscope.is_enabled) {
		gyro_x_axis_h = read_addr_gyro(GYRO_OUTX_H);
		gyro_x_axis_l = read_addr_gyro(GYRO_OUTX_L);
		gyro_y_axis_h = read_addr_gyro(GYRO_OUTY_H);
		gyro_y_axis_l = read_addr_gyro(GYRO_OUTY_L);
		gyro_z_axis_h = read_addr_gyro(GYRO_OUTZ_H);
		gyro_z_axis_l = read_addr_gyro(GYRO_OUTZ_L);
	}
	/* Put the sample data in the buffer */
	bool success = add_sample(&sample_buffer, delta_time_h, delta_time_m, delta_time_l,
										accel_x_axis_h, accel_x_axis_l, accel_y_axis_h, accel_y_axis_l, accel_z_axis_h, accel_z_axis_l,
										gyro_x_axis_h, gyro_x_axis_l, gyro_y_axis_h, gyro_y_axis_l, gyro_z_axis_h, gyro_z_axis_l);
	if (!success) {
		/* The buffer is full */
#ifdef DEBUG
		++debug_int;
#endif
		return true;
	}
	/* Update timestamp only if sample was successfully added to buffer */
	timestamp_accel = timestamp;
	return true;
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