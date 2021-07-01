#include <stddef.h>
#include "inputs.h"
#include "io.h"
#include "config.h"
#include "ir.h"

volatile uint16_t inputs_logic_state = 0;
volatile uint16_t inputs_debounced_state = 0;
volatile uint16_t inputs_old = 0;
volatile bool inputs_scanned = false;

bool btn_pressed = false;

#define DEBOUNCE_THRESHOLD 20 // 10 ms
#define IR_DEBOUNCE_THRESHOLD 5 // 5 IR ticks
volatile uint8_t _inputs_debounce_counter[NO_INPUTS] = {0, };
volatile uint8_t _inputs_fall_counter[NO_INPUTS] = {0, };
volatile uint8_t _btn_debounce_counter = 0;
volatile uint8_t _first_scan_counter = 0;

static void _inputs_button_debounce_update();

void inputs_debounce_update() {
	uint16_t state = ~io_get_inputs_raw();
	uint16_t irs = config_ir_inputs;

	if (_first_scan_counter < 2*DEBOUNCE_THRESHOLD) {
		_first_scan_counter++;
		if (_first_scan_counter == 2*DEBOUNCE_THRESHOLD)
			inputs_scanned = true;
	}

	for (size_t i = 0; i < NO_INPUTS; i++) {
		if ((irs&1) == 0) {
			if (state & 0x01) {
				if (_inputs_debounce_counter[i] < DEBOUNCE_THRESHOLD) {
					_inputs_debounce_counter[i]++;
					// If you need evens in future, you must enter if below only if
					// inputs_debounced_state[i] == 0
					if (_inputs_debounce_counter[i] == DEBOUNCE_THRESHOLD) {
						inputs_debounced_state |= (1 << i);
						inputs_logic_state |= (1 << i); // no fall
						if (_inputs_fall_counter[i] > 0)
							_inputs_fall_counter[i] = 0; // stop falling counter
					}
				}
			} else {
				if (_inputs_debounce_counter[i] > 0) {
					_inputs_debounce_counter[i]--;
					// If you need evens in future, you must enter if below only if
					// inputs_debounced_state[i] == 1
					if (_inputs_debounce_counter[i] == 0) {
						inputs_debounced_state &= ~(1 << i);
						_inputs_fall_counter[i] = input_delay(i)*10;
						if (_inputs_fall_counter[i] == 0)
							inputs_logic_state &= ~(1 << i);
					}
				}
			}
		}

		state >>= 1;
		irs >>= 1;
	}

	_inputs_button_debounce_update();
}

void ir_debounce_update() {
	uint16_t state = ir_state;
	uint16_t irs = config_ir_inputs;

	for (size_t i = 0; i < NO_INPUTS; i++) {
		if (irs&1) {
			if (state & 0x01) {
				if (_inputs_debounce_counter[i] < IR_DEBOUNCE_THRESHOLD) {
					_inputs_debounce_counter[i]++;
					// If you need evens in future, you must enter if below only if
					// inputs_debounced_state[i] == 0
					if (_inputs_debounce_counter[i] == IR_DEBOUNCE_THRESHOLD) {
						inputs_debounced_state |= (1 << i);
						inputs_logic_state |= (1 << i); // no fall
						if (_inputs_fall_counter[i] > 0)
							_inputs_fall_counter[i] = 0; // stop falling counter
					}
				}
			} else {
				if (_inputs_debounce_counter[i] > 0) {
					_inputs_debounce_counter[i]--;
					// If you need evens in future, you must enter if below only if
					// inputs_debounced_state[i] == 1
					if (_inputs_debounce_counter[i] == 0) {
						inputs_debounced_state &= ~(1 << i);
						_inputs_fall_counter[i] = input_delay(i)*10;
						if (_inputs_fall_counter[i] == 0)
							inputs_logic_state &= ~(1 << i);
					}
				}
			}
		}

		state >>= 1;
		irs >>= 1;
	}
}

void inputs_fall_update() {
	for (size_t i = 0; i < NO_INPUTS; i++) {
		if (_inputs_fall_counter[i] > 0) {
			_inputs_fall_counter[i]--;
			if (_inputs_fall_counter[i] == 0)
				inputs_logic_state &= ~(1 << i);
		}
	}
}

static void _inputs_button_debounce_update() {
	bool state = io_button();
	if (state & 0x01) {
		if (_btn_debounce_counter > 0) {
			_btn_debounce_counter--;
			if ((_btn_debounce_counter == 0) && (btn_pressed)) {
				btn_pressed = false;
				btn_on_depressed();
			}
		}
	} else {
		if (_btn_debounce_counter < DEBOUNCE_THRESHOLD) {
			_btn_debounce_counter++;
			if ((_btn_debounce_counter == DEBOUNCE_THRESHOLD) && (!btn_pressed)) {
				btn_pressed = true;
				btn_on_pressed();
			}
		}
	}
}
