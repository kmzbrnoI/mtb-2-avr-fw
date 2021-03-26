#include <util/delay.h>
#include "io.h"

uint8_t _address;
bool uart_out_high = false;

void io_init() {
	// Reset
	SPCR = 0;
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;

	DDRB |= (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE); // LED PB0 (green), PB1 (blue)
	DDRC |= (1 << PIN_LED_RED); // LED PC0 (red)
	PORTD |= (1 << PIN_BUTTON); // button pull-up

	_delay_us(50); // wait for inputs to load

	// With transistor (active low) = pulled up
	// Without transistor (active high) = pulled down
	uart_out_high = !((PIND >> PIN_UART_DIR) & 0x1);

	uart_in();
	DDRD |= (1 << PIN_TEST_PAD) | (1 << PIN_UART_DIR); // testpad, UART direction
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
