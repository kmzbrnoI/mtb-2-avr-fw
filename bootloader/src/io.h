#ifndef _IO_H_
#define _IO_H_

/* Raw input-output operation of bootloader of MTB-UNI v4 module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

#define INPUT_BUTTON PINB2

static inline void io_init() {
	DDRC |= 0x01; // LED PC0 (red)
	DDRB |= 0x03; // LED PB0 (green), PB1 (blue)
	PORTB |= 0x04; // button pull-up

	DDRD |= 0x14; // testpad, UART direction
	PORTD &= ~(1 << PD2); // uart IN
	PORTC |= (1 << PC0); // led RED off
}

static inline uint8_t io_get_addr_raw() {
	// TODO
	return 1;
}

static inline void io_led_red_on() { PORTC &= ~(1 << PC0); }
static inline void io_led_red_off() { PORTC |= (1 << PC0); }
static inline void io_led_green_on() { PORTB |= (1 << PB0); }
static inline void io_led_green_off() { PORTB &= ~(1 << PB0); }
static inline void io_led_blue_on() { PORTB |= (1 << PB1); }
static inline void io_led_blue_off() { PORTB &= ~(1 << PB1); }

static inline void io_led_red(bool state) {
	if (state)
		io_led_red_on();
	else
		io_led_red_off();
}

static inline void io_led_green(bool state) {
	if (state)
		io_led_green_on();
	else
		io_led_green_off();
}

static inline void io_led_blue(bool state) {
	if (state)
		io_led_blue_on();
	else
		io_led_blue_off();
}

static inline bool io_led_red_state() { return !((PORTC >> PC0) & 0x1); }
static inline bool io_led_green_state() { return (PORTB >> PB0) & 0x1; }
static inline bool io_led_blue_state() { return (PORTB >> PB1) & 0x1; }

static inline void io_led_red_toggle() { io_led_red(!io_led_red_state()); }
static inline void io_led_green_toggle() { io_led_green(!io_led_green_state()); }
static inline void io_led_blue_toggle() { io_led_blue(!io_led_blue_state()); }

static inline bool io_button() { return (PINB >> 2) & 0x1; }

static inline void uart_out() { PORTD |= (1 << PD2); }
static inline void uart_in() { PORTD &= ~(1 << PD2); }

#endif
