#ifndef _IO_H_
#define _IO_H_

/* Raw input-output operation of MTB-UNI v4 module.
 * This unit does not perform any debouncing nor any complicated IO stuff.
 * It just abstracts IO with nice & fast functions.
 */

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

// Defines below are informative only, they are basically never used.
// If you change IO pins mapping, look into ‹io.c› and change approprite
// functions.

#define NO_INPUTS 16
#define NO_OUTPUTS 16
#define NO_INPUTS_ADDR 8

#define INPUT_BUTTON PINB2

void io_init();

bool io_get_input_raw(uint8_t inum);
uint16_t io_get_inputs_raw();

static inline uint8_t io_get_addr_raw() {
	// TODO
	return 1;
}

void io_set_output_raw(uint8_t onum, bool state);
void io_set_outputs_raw(uint16_t state);
void io_set_outputs_raw_mask(uint16_t state, uint16_t mask);
uint16_t io_get_outputs_raw();

bool io_get_output_raw(uint8_t onum);

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

static inline void io_testpad_set(bool state) {
	if (state)
		PORTD |= (1 << PD4);
	else
		PORTD &= ~(1 << PD4);
}

static inline bool io_testpad_get() { return (PORTD & PD2); }
static inline void io_testpad_toggle() { io_testpad_set(!io_testpad_get()); }

static inline void uart_out() { PORTD |= (1 << PD2); }
static inline void uart_in() { PORTD &= ~(1 << PD2); }

#endif
