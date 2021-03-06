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

#define PIN_LED_RED PC0
#define PIN_LED_GREEN PB0
#define PIN_LED_BLUE PB1

// Warning: button pin conflict with SPI SS
#define PIN_BUTTON PD5
#define PIN_TEST_PAD PD4

#define PIN_OUTPUT_SET PD3
#define PIN_INPUT_SHIFT PD7
#define PIN_OUTPUTS_DISABLE PD6

#define PIN_IR_MA PC3
#define PIN_IR_MB PC2
#define PIN_IR_PULSE PC1

#define PIN_UART_RX PD0
#define PIN_UART_TX PD1
#define PIN_UART_DIR PD2

#define IR_CHANNELS 4

void io_init();

bool io_get_input_raw(uint8_t inum);
uint16_t io_get_inputs_raw();

uint8_t io_get_addr_raw();

void io_set_output_raw(uint8_t onum, bool state);
void io_set_outputs_raw(uint16_t state);
void io_set_outputs_raw_mask(uint16_t state, uint16_t mask);
uint16_t io_get_outputs_raw();

bool io_get_output_raw(uint8_t onum);

static inline void io_led_red_on() { PORTC &= ~(1 << PIN_LED_RED); }
static inline void io_led_red_off() { PORTC |= (1 << PIN_LED_RED); }
static inline void io_led_green_on() { PORTB |= (1 << PIN_LED_GREEN); }
static inline void io_led_green_off() { PORTB &= ~(1 << PIN_LED_GREEN); }
static inline void io_led_blue_on() { PORTB |= (1 << PIN_LED_BLUE); }
static inline void io_led_blue_off() { PORTB &= ~(1 << PIN_LED_BLUE); }

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

static inline bool io_led_red_state() { return !((PORTC >> PIN_LED_RED) & 0x1); }
static inline bool io_led_green_state() { return (PORTB >> PIN_LED_GREEN) & 0x1; }
static inline bool io_led_blue_state() { return (PORTB >> PIN_LED_BLUE) & 0x1; }

static inline void io_led_red_toggle() { io_led_red(!io_led_red_state()); }
static inline void io_led_green_toggle() { io_led_green(!io_led_green_state()); }
static inline void io_led_blue_toggle() { io_led_blue(!io_led_blue_state()); }

static inline bool io_button() { return (PIND >> PIN_BUTTON) & 0x1; }

static inline void io_testpad_set(bool state) {
	if (state)
		PORTD |= (1 << PIN_TEST_PAD);
	else
		PORTD &= ~(1 << PIN_TEST_PAD);
}

static inline bool io_testpad_get() { return ((PORTD >> PIN_TEST_PAD) & 0x1); }
static inline void io_testpad_toggle() { io_testpad_set(!io_testpad_get()); }

void uart_out();
void uart_in();

void io_shift_update();
void io_shift_load();

extern bool inputs_disabled;

static inline void io_ir_pulse(bool state) {
	// true = IR active, false = IR inactive
	if (state)
		PORTC &= ~(1 << PIN_IR_PULSE);
	else
		PORTC |= (1 << PIN_IR_PULSE);
}

static inline void io_ir_channel(uint8_t channel) {
	if (channel & 1)
		PORTC |= (1 << PIN_IR_MA);
	else
		PORTC &= ~(1 << PIN_IR_MA);

	if (channel & 2)
		PORTC |= (1 << PIN_IR_MB);
	else
		PORTC &= ~(1 << PIN_IR_MB);
}

static inline void outputs_enable() { PORTD &= ~(1 << PIN_OUTPUTS_DISABLE); }
static inline void outputs_disable() { PORTD |= (1 << PIN_OUTPUTS_DISABLE); }

#endif
