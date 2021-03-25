#include "io.h"

uint8_t _address;

void io_init() {
	DDRC |= (1 << PIN_LED_RED); // LED PC0 (red)
	DDRB |= (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE); // LED PB0 (green), PB1 (blue)
	PORTD |= (1 << PIN_BUTTON); // button pull-up

	DDRD |= (1 << PIN_TEST_PAD) | (1 << PIN_UART_DIR); // testpad, UART direction
	PORTD &= ~(1 << PIN_UART_DIR); // uart IN
	io_led_red_off();

	DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2); // MOSI & SCK & SS out
	PORTB |= (1 << PB4); // pull-up on MISO just for sure
	DDRD |= (1 << PIN_INPUT_SHIFT);
	DDRD |= (1 << PIN_INPUT_SHIFT) | (1 << PIN_OUTPUTS_DISABLE);

	SPCR = (1 << SPE) | (1 << MSTR); // enable SPI, SPI master, frequency=f_osc/4

	outputs_disable();
}

static inline uint8_t switch_bits_03(uint8_t data) {
	uint8_t res = data & 0xF6;
	res |= ((data >> 3) & 1);
	res |= ((data&1) << 3);
	return res;
}

void io_shift_update() {
	// Typical time of this function: 10 us
	uint8_t read;

	// Input load
	PORTD |= (1 << PIN_INPUT_SHIFT);
	__asm__("nop");
	__asm__("nop");

	SPDR = 0;
	while (!(SPSR & (1<<SPIF)));
	read = SPDR;

	SPDR = 0;
	while (!(SPSR & (1<<SPIF)));
	read = SPDR;

	SPDR = 0;
	while (!(SPSR & (1<<SPIF)));
	read = SPDR;

	_address = ~switch_bits_03(read);
	PORTD &= ~(1 << PIN_INPUT_SHIFT);
}

uint8_t io_get_addr_raw() {
	return _address;
}
