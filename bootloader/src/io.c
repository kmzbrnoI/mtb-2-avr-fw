#include <util/delay.h>
#include "io.h"

uint8_t _address;
bool uart_out_high = false;

static inline void io_reset() {
	SPCR = 0;
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
}

void io_init() {
	io_reset();

	DDRB |= (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE);
	DDRC |= (1 << PIN_LED_RED);
	PORTD |= (1 << PIN_BUTTON) | (1 << PIN_UART_RX); // pull-up

	_delay_us(50); // wait for inputs to load

	// With transistor (active low) = pulled up
	// Without transistor (active high) = pulled down
	uart_out_high = !((PIND >> PIN_UART_DIR) & 0x1);

	uart_in();
	DDRD |= (1 << PIN_TEST_PAD) | (1 << PIN_UART_DIR) | (1 << PIN_UART_TX);
	io_led_red_off();

	DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2); // MOSI & SCK & SS out
	PORTB |= (1 << PB4); // pull-up on MISO just for sure
	DDRD |= (1 << PIN_INPUT_SHIFT);
	DDRD |= (1 << PIN_INPUT_SHIFT) | (1 << PIN_OUTPUTS_DISABLE);

	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL); // enable SPI, SPI master, frequency=f_osc/4

	outputs_disable();
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
	read = ~SPDR;

	// Address is connected to shift's input almost randomly... :(
	_address = ((read & 0xF0) >> 2) | ((read & 0x8) >> 3) | ((read & 0x4) >> 1)
		| ((read & 0x2) << 5) | ((read & 0x1) << 7);

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
