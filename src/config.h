#ifndef _CONFIG_H_
#define _CONFIG_H_

/* General configuration of MTB-UNI module
 */

#include <stdint.h>
#include <stdbool.h>
#include "io.h"

extern uint8_t config_safe_state[NO_OUTPUTS];
extern uint8_t config_inputs_delay[NO_INPUTS/2];
extern bool config_write;
extern uint8_t config_mtbbus_speed;
extern uint16_t config_ir_inputs;
extern uint8_t config_ir_support;

// Warning: these functions take long time to execute
void config_load(void);
void config_save(void);
void config_save_ir_support(void);

void config_boot_fwupgd(void);
void config_boot_normal(void);

void config_int_wdrf(bool value);
bool config_is_int_wdrf(void);

uint16_t config_bootloader_version(void);
uint8_t config_mcusr(void);

uint8_t input_delay(uint8_t input);

static inline bool is_ir_input(uint8_t input) {
	return (config_ir_inputs >> input) & 1;
}

static inline bool is_ir_input_in_section(uint8_t input) {
	// Whether there is any IR input in section of 4 inputs
	// (transistors are in groups of 4 inputs).
	return (config_ir_inputs >> (4*input)) & 0x0F;
}

#define CONFIG_MODULE_TYPE_IR 0x10
#define CONFIG_MODULE_TYPE_NONIR 0x11
#define CONFIG_MODULE_TYPE (config_ir_support ? CONFIG_MODULE_TYPE_IR : CONFIG_MODULE_TYPE_NONIR)
#define CONFIG_FW_MAJOR 1
#define CONFIG_FW_MINOR 4
#define CONFIG_PROTO_MAJOR 4
#define CONFIG_PROTO_MINOR 1

#define CONFIG_BOOT_FWUPGD 0x01
#define CONFIG_BOOT_NORMAL 0x00

#endif
