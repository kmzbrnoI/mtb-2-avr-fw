#include <avr/io.h>
#include <avr/delay.h>

#include "io.h"
#include "common.h"

volatile uint16_t _outputs = 0;
volatile uint16_t _inputs = 0;
volatile uint8_t _address = 0;
bool uart_out_high = false;

bool inputs_disabled = false;

void io_init() {
	// Reset because could load from bootloader
	SPCR = 0;
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;

	PORTC |= (1 << PIN_IR_MA); // read inputs-disabled jumper, wait for input to settle
	DDRB |= (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE); // LED PB0 (green), PB1 (blue)
	DDRC |= (1 << PIN_LED_RED); // LED PC0 (red)
	PORTD |= (1 << PIN_BUTTON); // button pull-up

	_delay_us(50); // wait for inputs to load

	// With transistor (active low) = pulled up
	// Without transistor (active high) = pulled down
	uart_out_high = !((PIND >> PIN_UART_DIR) & 0x1);

	uart_in();
	DDRD |= (1 << PIN_TEST_PAD) | (1 << PIN_UART_DIR); // testpad, UART direction
	io_led_red_off(); // red LED is off in logical one

	DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2); // MOSI & SCK & SS out
	PORTB |= (1 << PB4); // pull-up on MISO just for sure
	outputs_disable();
	DDRD |= (1 << PIN_OUTPUT_SET) | (1 << PIN_INPUT_SHIFT) | (1 << PIN_OUTPUTS_DISABLE);

	SPCR = (1 << SPE) | (1 << MSTR); // enable SPI, SPI master, frequency=f_osc/4

	inputs_disabled = !((PINC >> PIN_IR_MA) & 0x1);
	if (!inputs_disabled) {
		// Configure IR outputs
		PORTC &= ~(1 << PIN_IR_MA);
		DDRC |= (1 << PIN_IR_MA) | (1 << PIN_IR_MB) | (1 << PIN_IR_PULSE);
		PORTC |= (1 << PIN_IR_PULSE); // idle high
	}
}

bool io_get_input_raw(uint8_t inum) {
	return (io_get_inputs_raw() >> inum) & 0x1;
}

uint16_t io_get_inputs_raw() {
	return _inputs;
}

void io_set_output_raw(uint8_t onum, bool state) {
	if (state)
		_outputs |= (1 << onum);
	else
		_outputs &= ~(1 << onum);
}

void io_set_outputs_raw(uint16_t state) {
	io_set_outputs_raw_mask(state, 0xFFFF);
}

void io_set_outputs_raw_mask(uint16_t state, uint16_t mask) {
	_outputs = (state & mask) | (_outputs & (~mask));
}

uint16_t io_get_outputs_raw() {
	return _outputs;
}

bool io_get_output_raw(uint8_t onum) {
	return (io_get_outputs_raw() >> onum) & 0x1;
}

uint8_t io_get_addr_raw() {
	return _address;
}

static inline uint8_t switch_bits_03(uint8_t data) {
	uint8_t res = data & 0xF6;
	res |= ((data >> 3) & 1);
	res |= ((data&1) << 3);
	return res;
}

void io_shift_update() {
	// Typical time of this function: 10 us

	uint16_t buf_inputs = 0;
	uint8_t read;

	// Input load
	PORTD |= (1 << PIN_INPUT_SHIFT);
	__asm__("nop");
	__asm__("nop");

	SPDR = 0;
	read = SPDR;
	while (!(SPSR & (1<<SPIF)));
	read = SPDR;
	buf_inputs = switch_bits_03(read);

	SPDR = _outputs & 0xFF;
	while (!(SPSR & (1<<SPIF)));
	read = SPDR;
	buf_inputs |= switch_bits_03(read) << 8;

	SPDR = _outputs >> 8;
	while (!(SPSR & (1<<SPIF)));
	read = SPDR;
	_address = ~switch_bits_03(read);

	// Output set
	PORTD |= (1 << PIN_OUTPUT_SET);
	__asm__("nop");
	__asm__("nop");
	PORTD &= ~(1 << PIN_OUTPUT_SET);

	PORTD &= ~(1 << PIN_INPUT_SHIFT);

	_inputs = buf_inputs;
}

void io_shift_load() {
	PORTD &= ~(1 << PIN_INPUT_SHIFT);
	__asm__("nop");
	__asm__("nop");
	PORTD |= (1 << PIN_INPUT_SHIFT); // disable loading now, data stays loaded in shifts
}

void uart_out() {
	if (uart_out_high)
		PORTD |= (1 << PIN_UART_DIR);
	else
		PORTD &= ~(1 << PIN_UART_DIR);
}

void uart_in() {
	if (uart_out_high)
		PORTD &= ~(1 << PIN_UART_DIR);
	else
		PORTD |= (1 << PIN_UART_DIR);
}
