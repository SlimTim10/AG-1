/**
 * Written by Icewire Technologies.
 *
 *
 * The format for config.ini is as follows:
 *     Text after semicolons is considered a comment.
 *     A line that matches /^ *sr *= *[0-9]+ *$/ is used to set the sample rate.
 *         Valid bandwidth values: 40, 160, 640.
 *     A line that matches /^ *ar *= *[0-9]+ *$/ is used to set the range of the
 *         accelerometer. Valid range values: 2, 6.
 *     A line that matches /^ *gr *= *[0-9]+ *$/ is used to set the range of the
 *         gyroscope. Valid range values: 250, 500, 2000.
 *     A line that matches /^ *gs *= *[0-9]+ *$/ is used to set the sample rate of
 *         the gyroscope. Valid bandwidth values: 100, 200, 400, 800.
 *     A line that matches /^ *disable_gyro *$/ is used to disable logging for the
 *         gyroscope.
 *     A line that matches /^ *disable_accel *$/ is used to disable logging for the
 *         accelerometer.
 */

#include <msp430f5310.h>
#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "sdfat.h"
#include "LIS3LV02DL.h"
#include "L3G4200D.h"
#include "config.h"
#include "msp430f5310_extra.h"
#include "circuit.h"
#include "samplebuffer.h"
#include "buttonbuffer.h"
#include "const.h"
#include "macro.h"
#include "conversions.h"

/*
 * Define global debugging variables
 */
#ifdef DEBUG
static volatile bool debug_hit = false;
static volatile uint32_t debug_int = 0;
#endif

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
struct SdCardFile {
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
/* Note: perform an empty read before so we can clear P1.5 */
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
void init_sd_fat(void);
void format_sd_card(void);
void new_sd_card_file(struct SdCardFile *const sd_card_file);
void add_firmware_info_to_sd_card_file(struct SdCardFile *const sd_card_file);
uint32_t get_block_offset(const struct SdCardFile *const sd_card_file);
bool add_value_to_buffer(struct SdCardFile *const sd_card_file, uint8_t value);
bool write_full_buffer_to_sd_card(struct SdCardFile *const sd_card_file);

/* Only write remainder of buffer for end of file */
bool write_remaining_buffer_to_sd_card(struct SdCardFile *const sd_card_file);
void get_config_settings(void);
void timer_interrupt_event(void);
bool button_press_event_handled(void);
bool sample_event_handled(void);
bool timer_interrupt_triggered(void);
void clear_timer_interrupt(void);
bool button_interrupt_triggered(void);
enum ButtonPress get_button_press(bool can_triple_tap);
enum ButtonPress wait_for_button_release(void);

/* High byte for continuous timer */
volatile uint8_t time_cont;

/* Time of last sample for getting delta timestamp for acceleration data for new sample */
uint32_t timestamp_accel;

/* Buffer for samples */
struct SampleBuffer sample_buffer;

/* Samples for buffer */
volatile struct Sample samples[RAW_SAMPLE_BUFF_SIZE];

/* Buffer for button presses */
struct ButtonPressBuffer button_press_buffer;

/* Button presses for buffer of button presses */
volatile enum ButtonPress button_presses[BUTTON_BUFF_SIZE];

/* Whether the user can triple tap */
bool triple_tap_enabled;

/* Whether there is an indication of a button tap by flashing the LED */
bool button_tap_flash_enabled;

/* Information for SD FAT library */
struct fatstruct fatinfo;

/* Buffer for accelerometer sample data to write to SD card */
struct SdCardFile sd_file;

/* Temporary variable for initializing the SD card using an SdCardFile's buffer */
uint8_t *data_sd;

/* Accelerometer settings */
struct Logger accelerometer;

/* Gyroscope settings */
struct Logger gyroscope;

/* So that the LED doesn't flash multiple times per second */
uint8_t prev_sec = 0;

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
	data_sd = sd_file.buffer;
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
	feed_watchdog();
	/* Make sure the LED is off */
	led_1_off();
	disable_interrupts();
	feed_watchdog();
	/*
	 * We don't want to waste time checking for triple taps when device
	 * is on as we don't have any features which require triple tapping
	 */
	enable_button_pressing(true, false);
	feed_watchdog();
	/* Set up the clock to flash the LED */
	rtc_restart();
	prev_sec = RTCSEC;
	feed_watchdog();
	enable_interrupts();
	return IDLE_STATE;
}

enum DeviceState turn_off(void) {
	feed_watchdog();
	/* Make sure the LED is off */
	led_1_off();
	disable_interrupts();
	/* We have features which require triple tapping when device is off */
	feed_watchdog();
	enable_button_pressing(false, true);
	feed_watchdog();
	enable_interrupts();
	return OFF_STATE;
}

enum DeviceState start_logging(void) {
	feed_watchdog();
	/* Turn on power to SD card and read the FAT boot sector */
	power_on_sd();
	feed_watchdog();
	/* Check for low voltage */
	if (voltage_is_low()) {
		restart();
	}
	feed_watchdog();
	init_sd_fat();
	feed_watchdog();
	/*  
	 * We parse the config file each time we want to start logging so the user doesn't
	 * have to restart the device manually each time they modify the config settings.
	 * NOTE if this ever proves too slow or too power hungry, we can do this in device
	 * turn on state.
	 */
	get_config_settings();
	disable_interrupts();
	feed_watchdog();
	enable_button_pressing(true, false);
	/* Power on logging devices and activate interrupts */
	feed_watchdog();
	/* Accelerometer is always turned on since we use its interrupt to grab samples */
	{
		power_on_accelerometer();
		activate_accel_interrupt();
	}
	feed_watchdog();
	if (gyroscope.is_enabled) {
		power_on_gyroscope();
	}
	feed_watchdog();
	new_sd_card_file(&sd_file);
	feed_watchdog();
	/* Clear raw samples buffer */
	clear_sample_buffer(&sample_buffer);
	/* Reset timer */
	time_cont = 0;
	/* Reset time of last sample */
	timestamp_accel = 0;
	feed_watchdog();
	/* Set up the clock to flash the LED */
	rtc_restart();
	prev_sec = RTCSEC;
	feed_watchdog();
	/* Start capturing samples */
	enable_interrupts();
	/* Read accelerometer axes to get interrupt started */
	accelerometer_empty_read();
	return LOG_STATE;
}

enum DeviceState stop_logging(void) {
	feed_watchdog();
	/* Power off logging devices */
	{
		power_off_accelerometer();
	}
	feed_watchdog();
	if (gyroscope.is_enabled) {
		power_off_gyroscope();
	}
	feed_watchdog();
	/* Write final logger data in buffer and update the directory table */
	{
		write_remaining_buffer_to_sd_card(&sd_file);
		/* Name of log file */
		uint8_t file_name[] = FILE_NAME;
		/* Get the number of the last log file */
		uint16_t file_num = get_file_num(sd_file.buffer, &fatinfo, file_name);
		if (update_dir_table(sd_file.buffer, 
									&fatinfo, 
									sd_file.start_cluster, 
									sd_file.size,
									file_name,
									file_num) != FAT_SUCCESS) {
			/* Turn the LED on and hang to indicate failure */
			led_1_on();
			HANG();
		}
	}
	feed_watchdog();
	/* Turn off power to SD card since writing is complete */
	power_off_sd();
	return idle();
}

enum DeviceState format_card(void) {
	feed_watchdog();
	disable_interrupts();
	/* No triple tapping feature in this state */
	enable_button_pressing(true, false);
	feed_watchdog();
	enable_interrupts();
	return FORMAT_STATE;
}

enum DeviceState off_step(void) {
	/* Turn off the wdt before entering low power mode */
	stop_watchdog();
	/* Wait for button press in low power mode */
	enter_LPM();
	/* Button press happened, so continue */
	exit_LPM();
	/* Turn on the wdt after exiting low power mode */
	start_watchdog();
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
			}
		}
	}
	return IDLE_STATE;
}

enum DeviceState log_step(void) {
	/* Check for low voltage */
	if (voltage_is_low()) {
		return stop_logging();
	}
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
			if (!add_value_to_buffer(&sd_file, NEW_LINE)) {
				return stop_logging();
			}
			/* Convert delta time to ascii and put in SD card buffer */
			{
				int32_t delta_time = int8arr_to_uint32(sample.delta_time);
				/* Max timestamp value is 8 digits */
				uint8_t ascii_buffer[8];
				uitoa(delta_time, ascii_buffer);
				for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 8; ++i) {
					if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
						return stop_logging();
					}
				}
			}
			if (accelerometer.is_enabled) {
				/* Add delimiter */
				if (!add_value_to_buffer(&sd_file, DELIMITER)) {
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
							if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_file, DELIMITER)) {
						return stop_logging();
					}
					/* Y-axis */
					{
						int16_t y_axis = int8arr_to_int16(sample.accel.y_axis);
						itoa(y_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_file, DELIMITER)) {
						return stop_logging();
					}
					/* Z-axis */
					{
						int16_t z_axis = int8arr_to_int16(sample.accel.z_axis);
						itoa(z_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
				}
			}
			if (gyroscope.is_enabled) {
				/* Add delimiter */
				if (!add_value_to_buffer(&sd_file, DELIMITER)) {
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
							if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_file, DELIMITER)) {
						return stop_logging();
					}
					/* Y-axis */
					{
						int16_t y_axis = int8arr_to_int16(sample.gyro.y_axis);
						itoa(y_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
								return stop_logging();
							}
						}
					}
					/* Add delimiter */
					if (!add_value_to_buffer(&sd_file, DELIMITER)) {
						return stop_logging();
					}
					/* Z-axis */
					{
						int16_t z_axis = int8arr_to_int16(sample.gyro.z_axis);
						itoa(z_axis, ascii_buffer);
						for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 6; ++i) {
							if (!add_value_to_buffer(&sd_file, ascii_buffer[i])) {
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
				case BUTTON_HOLD:
					format_sd_card();
					break;
			}
		}
	}
	return FORMAT_STATE;
}

void init_sd_fat(void) {
	/* Find and read the FAT16 boot sector */
	if (valid_boot_sector(data_sd, &fatinfo) != FAT_SUCCESS) {
		/* Turn the LED on and hang to indicate failure */
		led_1_on();
		HANG();
	}
	/* Parse the FAT16 boot sector */
	if (parse_boot_sector(data_sd, &fatinfo) != FAT_SUCCESS) {
		/* Show failure with LED 1  */
		led_1_panic();
		/* Restart upon failure */
		restart();
	}
}

void format_sd_card(void) {
	feed_watchdog();
	/* Turn on power to SD card */
	power_on_sd();
	feed_watchdog();
	/* Check for low voltage */
	if (voltage_is_low()) {
		restart();
	}
	feed_watchdog();
	/* Try to read the boot sector so we can salvage the config file */
	if (valid_boot_sector(data_sd, &fatinfo) == FAT_SUCCESS &&
		parse_boot_sector(data_sd, &fatinfo) == FAT_SUCCESS) {
	} else {
		fat_defaults(&fatinfo);
	}
	/* Formatting takes a while so we need to stop the wdt */
	stop_watchdog();
	/* Format the SD card, using LED 1 to indicate it's being formatted */
	format_sd(data_sd, &fatinfo, led_1_on, led_1_toggle, led_1_off);
	restart();
}

void new_sd_card_file(struct SdCardFile *const sd_card_file) {
	sd_card_file->start_cluster = find_cluster(sd_card_file->buffer, &fatinfo);
	/* The SD card is full */
	if (!sd_card_file->start_cluster) {
		/* Turn the LED on and hang to indicate failure */
		led_1_on();
		HANG();
	}
	sd_card_file->index = 0;
	sd_card_file->cluster = sd_card_file->start_cluster;
	sd_card_file->block_num = 0;
	sd_card_file->size = 0;
	/* Firmware info */
	add_firmware_info_to_sd_card_file(sd_card_file);
	sd_card_file->buffer[sd_card_file->index++] = NEW_LINE;
//	feed_watchdog();
	/* Sample rate */
	sd_card_file->buffer[sd_card_file->index++] = 's';
	sd_card_file->buffer[sd_card_file->index++] = 'a';
	sd_card_file->buffer[sd_card_file->index++] = 'm';
	sd_card_file->buffer[sd_card_file->index++] = 'p';
	sd_card_file->buffer[sd_card_file->index++] = 'l';
	sd_card_file->buffer[sd_card_file->index++] = 'e';
	sd_card_file->buffer[sd_card_file->index++] = '-';
	sd_card_file->buffer[sd_card_file->index++] = 'r';
	sd_card_file->buffer[sd_card_file->index++] = 'a';
	sd_card_file->buffer[sd_card_file->index++] = 't';
	sd_card_file->buffer[sd_card_file->index++] = 'e';
	sd_card_file->buffer[sd_card_file->index++] = ':';
	sd_card_file->buffer[sd_card_file->index++] = ' ';
	/* Convert the sample rate to ascii */
	{
		uint8_t ascii_buffer[3];
		itoa(bandwidth_bits_to_hz_accel(accelerometer.bandwidth), ascii_buffer);
		for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 3; ++i) {
			sd_card_file->buffer[sd_card_file->index++] = ascii_buffer[i];
		}
	}
	sd_card_file->buffer[sd_card_file->index++] = ' ';
	sd_card_file->buffer[sd_card_file->index++] = 'H';
	sd_card_file->buffer[sd_card_file->index++] = 'z';
	sd_card_file->buffer[sd_card_file->index++] = NEW_LINE;
//	feed_watchdog();
	/* Range setting */
	if (accelerometer.is_enabled) {
		sd_card_file->buffer[sd_card_file->index++] = 'a';
		sd_card_file->buffer[sd_card_file->index++] = 'c';
		sd_card_file->buffer[sd_card_file->index++] = 'c';
		sd_card_file->buffer[sd_card_file->index++] = 'e';
		sd_card_file->buffer[sd_card_file->index++] = 'l';
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = 'r';
		sd_card_file->buffer[sd_card_file->index++] = 'a';
		sd_card_file->buffer[sd_card_file->index++] = 'n';
		sd_card_file->buffer[sd_card_file->index++] = 'g';
		sd_card_file->buffer[sd_card_file->index++] = 'e';
		sd_card_file->buffer[sd_card_file->index++] = ':';
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = '+';
		sd_card_file->buffer[sd_card_file->index++] = '/';
		sd_card_file->buffer[sd_card_file->index++] = '-';
		sd_card_file->buffer[sd_card_file->index++] = range_bits_to_g_accel(accelerometer.range) + 0x30;
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = 'g';
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = '(';
		sd_card_file->buffer[sd_card_file->index++] = '+';
		sd_card_file->buffer[sd_card_file->index++] = '/';
		sd_card_file->buffer[sd_card_file->index++] = '-';
		sd_card_file->buffer[sd_card_file->index++] = '3';
		sd_card_file->buffer[sd_card_file->index++] = '2';
		sd_card_file->buffer[sd_card_file->index++] = '7';
		sd_card_file->buffer[sd_card_file->index++] = '6';
		sd_card_file->buffer[sd_card_file->index++] = '8';
		sd_card_file->buffer[sd_card_file->index++] = ')';
		sd_card_file->buffer[sd_card_file->index++] = NEW_LINE;
	}
	if (gyroscope.is_enabled) {
		sd_card_file->buffer[sd_card_file->index++] = 'g';
		sd_card_file->buffer[sd_card_file->index++] = 'y';
		sd_card_file->buffer[sd_card_file->index++] = 'r';
		sd_card_file->buffer[sd_card_file->index++] = 'o';
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = 'r';
		sd_card_file->buffer[sd_card_file->index++] = 'a';
		sd_card_file->buffer[sd_card_file->index++] = 'n';
		sd_card_file->buffer[sd_card_file->index++] = 'g';
		sd_card_file->buffer[sd_card_file->index++] = 'e';
		sd_card_file->buffer[sd_card_file->index++] = ':';
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = '+';
		sd_card_file->buffer[sd_card_file->index++] = '/';
		sd_card_file->buffer[sd_card_file->index++] = '-';
		/* Convert the range to ascii */
		{
			uint8_t ascii_buffer[4];
			itoa(range_bits_to_dps_gyro(gyroscope.range), ascii_buffer);
			for (uint8_t i = 0; ascii_buffer[i] != NULL_TERMINATOR && i < 4; ++i) {
				sd_card_file->buffer[sd_card_file->index++] = ascii_buffer[i];
			}
		}
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = 'd';
		sd_card_file->buffer[sd_card_file->index++] = 'p';
		sd_card_file->buffer[sd_card_file->index++] = 's';
		sd_card_file->buffer[sd_card_file->index++] = ' ';
		sd_card_file->buffer[sd_card_file->index++] = '(';
		sd_card_file->buffer[sd_card_file->index++] = '+';
		sd_card_file->buffer[sd_card_file->index++] = '/';
		sd_card_file->buffer[sd_card_file->index++] = '-';
		sd_card_file->buffer[sd_card_file->index++] = '3';
		sd_card_file->buffer[sd_card_file->index++] = '2';
		sd_card_file->buffer[sd_card_file->index++] = '7';
		sd_card_file->buffer[sd_card_file->index++] = '6';
		sd_card_file->buffer[sd_card_file->index++] = '8';
		sd_card_file->buffer[sd_card_file->index++] = ')';
		sd_card_file->buffer[sd_card_file->index++] = NEW_LINE;
	}
	/* delta-time units */
	sd_card_file->buffer[sd_card_file->index++] = 'd';
	sd_card_file->buffer[sd_card_file->index++] = 't';
	sd_card_file->buffer[sd_card_file->index++] = ' ';
	sd_card_file->buffer[sd_card_file->index++] = 'u';
	sd_card_file->buffer[sd_card_file->index++] = 'n';
	sd_card_file->buffer[sd_card_file->index++] = 'i';
	sd_card_file->buffer[sd_card_file->index++] = 't';
	sd_card_file->buffer[sd_card_file->index++] = 's';
	sd_card_file->buffer[sd_card_file->index++] = ':';
	sd_card_file->buffer[sd_card_file->index++] = ' ';
	sd_card_file->buffer[sd_card_file->index++] = '8';
	sd_card_file->buffer[sd_card_file->index++] = '3';
	sd_card_file->buffer[sd_card_file->index++] = '.';
	sd_card_file->buffer[sd_card_file->index++] = '3';
	sd_card_file->buffer[sd_card_file->index++] = '3';
	sd_card_file->buffer[sd_card_file->index++] = ' ';
	sd_card_file->buffer[sd_card_file->index++] = 'n';
	sd_card_file->buffer[sd_card_file->index++] = 's';
	sd_card_file->buffer[sd_card_file->index++] = NEW_LINE;
//	feed_watchdog();
	/* Column titles */
	sd_card_file->buffer[sd_card_file->index++] = 'd';
	sd_card_file->buffer[sd_card_file->index++] = 't';
	if (accelerometer.is_enabled) {
		sd_card_file->buffer[sd_card_file->index++] = ',';
		sd_card_file->buffer[sd_card_file->index++] = 'a';
		sd_card_file->buffer[sd_card_file->index++] = 'c';
		sd_card_file->buffer[sd_card_file->index++] = 'c';
		sd_card_file->buffer[sd_card_file->index++] = 'e';
		sd_card_file->buffer[sd_card_file->index++] = 'l';
		sd_card_file->buffer[sd_card_file->index++] = '(';
		sd_card_file->buffer[sd_card_file->index++] = 'x';
		sd_card_file->buffer[sd_card_file->index++] = ',';
		sd_card_file->buffer[sd_card_file->index++] = 'y';
		sd_card_file->buffer[sd_card_file->index++] = ',';
		sd_card_file->buffer[sd_card_file->index++] = 'z';
		sd_card_file->buffer[sd_card_file->index++] = ')';
	}
	if (gyroscope.is_enabled) {
		sd_card_file->buffer[sd_card_file->index++] = ',';
		sd_card_file->buffer[sd_card_file->index++] = 'g';
		sd_card_file->buffer[sd_card_file->index++] = 'y';
		sd_card_file->buffer[sd_card_file->index++] = 'r';
		sd_card_file->buffer[sd_card_file->index++] = 'o';
		sd_card_file->buffer[sd_card_file->index++] = '(';
		sd_card_file->buffer[sd_card_file->index++] = 'x';
		sd_card_file->buffer[sd_card_file->index++] = ',';
		sd_card_file->buffer[sd_card_file->index++] = 'y';
		sd_card_file->buffer[sd_card_file->index++] = ',';
		sd_card_file->buffer[sd_card_file->index++] = 'z';
		sd_card_file->buffer[sd_card_file->index++] = ')';
	}
}

void add_firmware_info_to_sd_card_file(struct SdCardFile *const sd_card_file) {
	uint8_t name[] = FIRMWARE_NAME;
	uint8_t version[] = FIRMWARE_VERSION;
	/* Add firmware name */
	for (uint8_t i = 0; name[i] != NULL_TERMINATOR; ++i) {
		sd_card_file->buffer[sd_card_file->index++] = name[i];
	}
	sd_card_file->buffer[sd_card_file->index++] = ' ';
	/* Add firmware version */
	sd_card_file->buffer[sd_card_file->index++] = 'v';
	for (uint8_t i = 0; version[i] != NULL_TERMINATOR; ++i) {
		sd_card_file->buffer[sd_card_file->index++] = version[i];
	}
}

uint32_t get_block_offset(const struct SdCardFile *const sd_card_file) {
	uint32_t block_offset = sd_card_file->block_num;
	block_offset *= BLKSIZE;
	block_offset += get_cluster_offset(sd_card_file->cluster, &fatinfo);
	return block_offset;
}

bool add_value_to_buffer(struct SdCardFile *const sd_card_file, uint8_t value) {
	sd_card_file->buffer[sd_card_file->index++] = value;
	if (!write_full_buffer_to_sd_card(sd_card_file)) {
		return false;
	}
	return true;
}

bool write_full_buffer_to_sd_card(struct SdCardFile *const sd_card_file) {
	if (sd_card_file->index > SD_SAMPLE_BUFF_SIZE) {
		/* Something went very wrong... */
#ifdef DEBUG
		HANG();
#endif
		return false;
	}
	/* Buffer is full so write it to the SD card */
	if (sd_card_file->index == SD_SAMPLE_BUFF_SIZE) {
		/* Write entire buffer to SD card */
		uint32_t block_offset = get_block_offset(sd_card_file);
		uint8_t blocks = (uint16_t)SD_SAMPLE_BUFF_SIZE / BLKSIZE;
		if (write_multiple_block(sd_card_file->buffer, block_offset, blocks) != SD_SUCCESS) {
			/* Couldn't write blocks */
#ifdef DEBUG
			HANG();
#endif
			return false;
		}
		/* Prepare for writing next block */
		sd_card_file->size += SD_SAMPLE_BUFF_SIZE;
		sd_card_file->index = 0;
		sd_card_file->block_num += blocks;
		
		/* Cluster is full */
		if (!valid_block(sd_card_file->block_num, &fatinfo)) {
			/* Find another cluster */
			uint16_t next_cluster = find_cluster(sd_card_file->buffer, &fatinfo);
			if (!next_cluster) {
				/* Couldn't find another cluster; SD card is full */
#ifdef DEBUG
				HANG();
#endif
				return false;
			}
			/* Update the FAT */
			if (update_fat(sd_card_file->buffer, &fatinfo, sd_card_file->cluster * 2, next_cluster)) {
				/* Couldn't update FAT */
#ifdef DEBUG
				HANG();
#endif
				return false;
			}
			sd_card_file->cluster = next_cluster;
			sd_card_file->block_num = 0;
		}
	}
	return true;
}

bool write_remaining_buffer_to_sd_card(struct SdCardFile *const sd_card_file) {
	if (sd_card_file->index > SD_SAMPLE_BUFF_SIZE) {
		/* Something went very wrong... */
#ifdef DEBUG
		HANG();
#endif
		return false;
	}
	/* Write the remaining buffer data to SD card */
	if (sd_card_file->index > BLKSIZE) {
		/*
		 * Do a multi block write followed by a single block write if not all
		 * bytes were written: remaining bytes were not a multiple of BLKSIZE
		 */
		uint32_t block_offset = get_block_offset(sd_card_file);
		uint8_t blocks = sd_card_file->index / BLKSIZE;
		if (write_multiple_block(sd_card_file->buffer, block_offset, blocks) != SD_SUCCESS) {
			/* Couldn't write blocks */
#ifdef DEBUG
			HANG();
#endif
			return false;
		}
		uint16_t bytes_written = blocks * BLKSIZE;
		sd_card_file->size += bytes_written;
		sd_card_file->index = sd_card_file->index - bytes_written;
		sd_card_file->block_num += blocks;
		
		/* Now write the single block */
		if (sd_card_file->index > 0) {
			/* Place bytes at beginning of buffer */
			for (uint16_t i = 0; i < sd_card_file->index; ++i) {
				sd_card_file->buffer[i] = sd_card_file->buffer[bytes_written + i];
			}
			/* Write the block */
			block_offset = get_block_offset(sd_card_file);
			if (write_block(sd_card_file->buffer, block_offset, sd_card_file->index) != SD_SUCCESS) {
	#ifdef DEBUG
				HANG();
	#endif
				return false;
			}
			sd_card_file->size += sd_card_file->index;
		}
	} else {
		/* Write the single block */
		uint32_t block_offset = get_block_offset(sd_card_file);
		if (write_block(sd_card_file->buffer, block_offset, sd_card_file->index) != SD_SUCCESS) {
			/* Couldn't write block */
#ifdef DEBUG
			HANG();
#endif
			return false;
		}
		sd_card_file->size += sd_card_file->index;
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
	timer_interrupt_event();
}

void timer_interrupt_event(void) {
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
		bool success = button_press_event_handled();
		/* Deactivate interrupts to prevent additional button presses and end sampling */
		deactivate_interrupts();
		if (success) {
			/* Wake up from low power mode; does nothing if not in low power mode */
			LPM3_EXIT;
		}
		/* Clear the button interrupt flag */
		clear_int_ctrl();
	}
	if (accel_int()) {
		/* Accelerometer interrupt flag is cleared when axes are read */
		/* Keep trying to handle the event until successful */
		while (!sample_event_handled());
		/* Clear the accelerometer interrupt flag */
		clear_int_accel();
	}
}

bool button_press_event_handled(void) {
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
//#ifdef DEBUG
//		debug_hit = true;
//#endif
		/* Run the timer interrupt event */
		timer_interrupt_event();
		return false;
	}
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
	/* Get accelerometer sample data (which also clears the accel interrupt flag) */
	uint8_t accel_x_axis_h = read_addr_accel(ACCEL_OUTX_H);
	uint8_t accel_x_axis_l = read_addr_accel(ACCEL_OUTX_L);
	uint8_t accel_y_axis_h = read_addr_accel(ACCEL_OUTY_H);
	uint8_t accel_y_axis_l = read_addr_accel(ACCEL_OUTY_L);
	uint8_t accel_z_axis_h = read_addr_accel(ACCEL_OUTZ_H);
	uint8_t accel_z_axis_l = read_addr_accel(ACCEL_OUTZ_L);
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
	if (success) {
		/* Update timestamp only if sample was successfully added to buffer */
		timestamp_accel = timestamp;
	} else {
//#ifdef DEBUG
//		++debug_int;
//#endif
	}
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