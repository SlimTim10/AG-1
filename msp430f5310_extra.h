/**
 * Written by Tim Johns.
 */

#ifndef _MSPLIB_H
#define _MSPLIB_H

/* Threshold voltage for device operation = 3.0 V */
#define VOLTAGE_THRSHLD		0x0267

void enter_LPM(void);
void exit_LPM(void);
void wdt_config(void);
void wdt_stop(void);
void adc_config(void);
uint16_t adc_read(void);
void clock_config(void);
void rtc_restart(void);
uint8_t rtc_rdy(void);
void enable_interrupts(void);
void disable_interrupts(void);
void brownout_reset(void);
void timer_config(void);

#endif