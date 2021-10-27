#include <avr/io.h>
#include <avr/interrupt.h>
#include "diag.h"

mcusr_t mcusr;
error_flags_t error_flags = {0};
mtbbus_warn_flags_t mtbbus_warn_flags = {0};
mtbbus_warn_flags_t mtbbus_warn_flags_old = {0};
volatile uint16_t vcc_voltage = 0;

void vcc_init_measure() {
	// Initialize VCC measurement.
	// This function sets ADC properties. No change should be done to these
	// registers in any other function in future!
	ADMUX |= (1 << REFS0); // AVCC with external capacitor at AREF pin
	ADMUX |= 0xE; // measure internal 1V1 band-gap reference
	ADCSRA |= (1 << ADIE) | (1 << ADEN); // enable ADC interrupt, enable ADC
	ADCSRA |= 0x5; // prescaler 32×
}

void vcc_start_measure() {
	// Measure VCC voltage.
	ADCSRA |= (1 << ADSC); // start conversion
}

ISR(ADC_vect) {
	uint16_t value = ADCL;
	value |= (ADCH << 8);
	vcc_voltage = value;
}
