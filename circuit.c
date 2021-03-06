/**
 * Written by Tim Johns.
 *
 * The function bodies in this file are specific to the current circuit design.
 */

#ifndef _CIRCUITLIB_C
#define _CIRCUITLIB_C

#include <msp430f5310.h>
#include <stdint.h>
#include "circuit.h"

/*----------------------------------------------------------------------------*/
/* Configure microcontroller pin selections									  */
/*----------------------------------------------------------------------------*/
void mcu_pin_config(void) {
	P1DIR &= ~BIT1;		// input: CTRL button
	P1DIR |= BIT3;		// output: LED1
// output: LDO regulators
	P6DIR |= SD_PWR | ACCEL_PWR | GYRO_PWR;
	P1DIR &= ~BIT5;		// input: Accelerometer **a LIS3LV02DL INT1 interrupt
	P1DIR &= ~BIT7;		// input: Gyroscope **g interrupt L3G4200D INT2
	P6SEL |= BIT3;		// Select ADC on P6.3 (technically not GPIO)
}

/*----------------------------------------------------------------------------*/
/* Select XIN and XOUT options in microcontroller pins						  */
/*----------------------------------------------------------------------------*/
void mcu_xt_pins(void) {
// Select XIN on P5.4
	P5SEL |= BIT4;
// Select XOUT on P5.5
	P5SEL |= BIT5;
}

/*----------------------------------------------------------------------------*/
/* Return true iff CTRL is high (button is pressed down)					  */
/*----------------------------------------------------------------------------*/
uint8_t ctrl_high(void) {
	return (P1IN & BIT1);
}

/*----------------------------------------------------------------------------*/
/* Configure interrupts for accelerometer and gyroscope connections			  */
/*----------------------------------------------------------------------------*/
/* DEP
void interrupt_config(void) {
	P1IE = 0;						// Clear enabled interrupts on P1
	P1IE |= BIT5;					// P1.5 interrupt enabled for accelerometer
	P1IES &= ~BIT5;					// P1.5 edge select: low-to-high transition
	P1IE |= BIT7;					// P1.7 interrupt enabled for gyroscope
	P1IES &= ~BIT7;					// P1.7 edge select: low-to-high transition
	P1IE |= BIT1;					// P1.1 interrupt enabled for CTRL button
	P1IES &= ~BIT1;					// P1.1 edge select: low-to-high transition
	P1IFG = 0x0000;					// Clear all pending interrupt Flags
}
*/

void deactivate_interrupts(void) {
	/* Reset interrupts on P1 */
	P1IE = 0;
	/* Clear all pending interrupt Flags */
	P1IFG = 0x0000;
}

void activate_accel_interrupt(void) {
	/* P1.5 interrupt enabled for accelerometer */
	P1IE |= BIT5;
	/* P1.5 edge select: low-to-high transition */
	P1IES &= ~BIT5;
}

void activate_gyro_interrupt(void) {
	/* P1.7 interrupt enabled for gyroscope */
	P1IE |= BIT7;
	/* P1.7 edge select: low-to-high transition */
	P1IES &= ~BIT7;
}

void activate_ctrl_interrupt(void) {
	/* P1.1 interrupt enabled for CTRL button */
	P1IE |= BIT1;
	/* P1.1 edge select: low-to-high transition */
	P1IES &= ~BIT1;
}

/*----------------------------------------------------------------------------*/
/* Set interrupt flag for accelerometer (P1.5)								  */
/*----------------------------------------------------------------------------*/
void set_int_accel(void) {
	P1IFG |= BIT5;
}

/*----------------------------------------------------------------------------*/
/* Clear interrupt flag for accelerometer (P1.5)							  */
/*----------------------------------------------------------------------------*/
void clear_int_accel(void) {
	P1IFG &= ~BIT5;
}

/*----------------------------------------------------------------------------*/
/* Set interrupt flag for gyroscope (P1.7)									  */
/*----------------------------------------------------------------------------*/
void set_int_gyro(void) {
	P1IFG |= BIT7;
}

/*----------------------------------------------------------------------------*/
/* Clear interrupt flag for gyroscope (P1.7)								  */
/*----------------------------------------------------------------------------*/
void clear_int_gyro(void) {
	P1IFG &= ~BIT7;
}

/*----------------------------------------------------------------------------*/
/* Clear interrupt flag for CTRL button (P1.1)								  */
/*----------------------------------------------------------------------------*/
void clear_int_ctrl(void) {
	P1IFG &= ~BIT1;
}

/*----------------------------------------------------------------------------*/
/* Enable LDO regulator controlled by MCU pin 6.n							  */
/*----------------------------------------------------------------------------*/
void power_on(uint8_t n) {
	P6OUT |= n;
}

/*----------------------------------------------------------------------------*/
/* Disable LDO regulator controlled by MCU pin 6.n							  */
/*----------------------------------------------------------------------------*/
void power_off(uint8_t n) {
	P6OUT &= ~n;
}

/*----------------------------------------------------------------------------*/
/* Turn off all MCU SPI outputs												  */
/*----------------------------------------------------------------------------*/
void mcu_spi_off(void) {
	P4SEL = 0x00;				// Unselect SPI bus function
	P4OUT = 0x00;				// All slaves SPI bus set low
	P4DIR = 0xFF;				// P4 output direction
	P1OUT &= (~BIT4) & (~BIT6);	// Accelerometer CS and gyroscope CS
}

/*----------------------------------------------------------------------------*/
/* Turn LED light 1 on (P1.3)												  */
/*----------------------------------------------------------------------------*/
void led_1_on(void) {
	P1OUT |= BIT3;
}

/*----------------------------------------------------------------------------*/
/* Turn LED light 1 off (P1.3)												  */
/*----------------------------------------------------------------------------*/
void led_1_off(void) {
	P1OUT &= ~BIT3;
}

/*----------------------------------------------------------------------------*/
/* Turn LED light 1 on if off, or off if on (P1.3)							  */
/*----------------------------------------------------------------------------*/
void led_1_toggle(void) {
	P1OUT ^= BIT3;
}

#endif