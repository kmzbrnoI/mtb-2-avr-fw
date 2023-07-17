#include <stddef.h>

#include "ir.h"
#include "io.h"
#include "inputs.h"

#define SINGLE_IR_PERIOD 50 // 2500 us

uint16_t ir_state = 0;
bool ir_shift_disable = false;
void (*volatile on_shift_scanned)() = NULL;
volatile size_t pwri = 0;
volatile uint8_t inputs_active = 0;
volatile bool ir_debounce_to_update = false;

static inline void _shift_scanned();

void ir_update_50us() {
	static size_t counter = 0;

	if (counter == 0) {
		// Activate output
		io_ir_channel(pwri);
		io_ir_pulse(true);
		ir_shift_disable = true;

	} else if (counter == 1) {
		io_ir_pulse(false);

		// Load inputs state to shifts right now
		io_shift_load();
		ir_shift_disable = false;
		on_shift_scanned = _shift_scanned; // Next scan will trigger our event

	} else if (counter == 10) {
		uint8_t inputs = ((~io_get_inputs_raw()) >> (pwri*4)) & 0xF;
		ir_state &= ~(0xF << (pwri*4)); // clear old state
		ir_state |= (inputs_active & (~inputs) & 0xF) << (pwri*4);
	}

	counter++;
	if (counter >= SINGLE_IR_PERIOD) {
		counter = 0;
		pwri++;
		if (pwri == IR_CHANNELS) {
			pwri = 0;
			ir_debounce_to_update = true;
			// Directly calling ir_debounce_update() takes long time.
			// So long time that MTBbus receive interrupt is missed
		}
	}
}

void _shift_scanned() {
	inputs_active = ((~io_get_inputs_raw()) >> (pwri*4)) & 0xF;
}
