#ifndef _IO_H_
#define _IO_H_

/* Raw input-output operation of bootloader of MTB-UNI v4 module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

#define PIN_LED_RED PC0
#define PIN_LED_GREEN PB0
#define PIN_LED_BLUE PB1

#define PIN_BUTTON PD5
#define PIN_TEST_PAD PD4
#define PIN_INPUT_SHIFT PD7
#define PIN_OUTPUTS_DISABLE PD6

#define PIN_UART_RX PD0
#define PIN_UART_TX PD1
#define PIN_UART_DIR PD2

void io_init();

uint8_t io_get_addr_raw();

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

void uart_out();
void uart_in();

static inline void outputs_enable() { PORTD &= ~(1 << PIN_OUTPUTS_DISABLE); }
static inline void outputs_disable() { PORTD |= (1 << PIN_OUTPUTS_DISABLE); }

void io_shift_update();

#endif
