#ifndef _IR_H_
#define _IR_H_

#include <stdbool.h>
#include <stdint.h>

void ir_update_50us();

extern uint16_t ir_state;
extern bool ir_shift_disable;
extern void (*on_shift_scanned)();

#endif