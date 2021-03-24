#include <avr/io.h>

#include "io.h"
#include "common.h"

void io_init() {
	DDRC |= 0x01; // LED PC0 (red)
	DDRB |= 0x03; // LED PB0 (green), PB1 (blue)
	PORTB |= 0x04; // button pull-up

	DDRD |= 0x14; // testpad, UART direction
	uart_in();
	io_led_red_off(); // red LED is off in logical one
}

bool io_get_input_raw(uint8_t inum) {
	return (io_get_inputs_raw() >> inum) & 0x1;
}

uint16_t io_get_inputs_raw() {
	return 0; // TODO
}

void io_set_output_raw(uint8_t onum, bool state) {
	// TODO
}

void io_set_outputs_raw(uint16_t state) {
	io_set_outputs_raw_mask(state, 0xFFFF);
}

void io_set_outputs_raw_mask(uint16_t state, uint16_t mask) {
	// TODO
	/*state = (state & mask) | (io_get_outputs_raw() & (~mask));
	uint8_t low = state & 0xFF;
	uint8_t high = (state >> 8) & 0xFF;
	PORTD = bit_reverse(low);
	PORTC = bit_reverse(high);*/
}

uint16_t io_get_outputs_raw() {
	return 0; // TODO
}

bool io_get_output_raw(uint8_t onum) {
	return (io_get_outputs_raw() >> onum) & 0x1;
}
