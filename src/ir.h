#ifndef _IR_H_
#define _IR_H_

#include <stdbool.h>
#include <stdint.h>

void ir_update_50us(void);

extern uint16_t ir_state;
extern bool ir_shift_disable;
extern volatile bool ir_debounce_to_update;
extern void (*volatile on_shift_scanned)(void);

#endif
