/* Main source file of MTB-UNI v4 CPU's ATmega128 firmware.
 */

#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "common.h"
#include "io.h"
#include "scom.h"
#include "outputs.h"
#include "config.h"
#include "inputs.h"
#include "ir.h"
#include "diag.h"
#include "../lib/mtbbus.h"
#include "../lib/crc16modbus.h"

///////////////////////////////////////////////////////////////////////////////
// Function prototypes

int main();
static inline void init();
void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len);
void mtbbus_send_ack();
void mtbbus_send_inputs(uint8_t message_code);
void mtbbus_send_error(uint8_t code);
static inline void leds_update();
void goto_bootloader();
static inline void update_mtbbus_polarity();
void led_red_ok();
static bool is_ir_support_measure();
void mtbbus_free();
static inline void on_initialized();
static inline bool mtbbus_addressed();
static inline void btn_short_press();
static inline void btn_long_press();
static inline void autodetect_mtbbus_speed();
static inline void autodetect_mtbbus_speed_stop();
void mtbbus_auto_speed_next();
static inline void mtbbus_auto_speed_received();
void send_diag_value(uint8_t i);

///////////////////////////////////////////////////////////////////////////////
// Defines & global variables

#define LED_GR_ON 5
#define LED_GR_OFF 2
volatile uint8_t led_gr_counter = 0;

#define LED_RED_OK_ON 40
#define LED_RED_OK_OFF 20
#define LED_RED_ERR_ON 100
#define LED_RED_ERR_OFF 50
volatile uint8_t led_red_counter = 0;

volatile bool beacon = false;
volatile bool inputs_debounce_to_update = false;
volatile bool scom_to_update = false;
volatile bool outputs_changed_when_setting_scom = false;

#define LED_BLUE_BEACON_ON 100
#define LED_BLUE_BEACON_OFF 50
volatile uint8_t led_blue_counter = 0;

__attribute__((used, section(".fwattr"))) struct {
	uint8_t no_pages;
	uint16_t crc;
} fwattr;

volatile bool initialized = false;
volatile uint8_t _init_counter = 0;
#define INIT_TIME 50 // 500 ms

#define MTBBUS_TIMEOUT_MAX 100 // 1 s
volatile uint8_t mtbbus_timeout = MTBBUS_TIMEOUT_MAX; // increment each 10 ms

volatile uint8_t btn_press_time = 0;

volatile bool mtbbus_auto_speed_in_progress = false;
volatile uint8_t mtbbus_auto_speed_timer = 0;
volatile uint8_t mtbbus_auto_speed_last;
#define MTBBUS_AUTO_SPEED_TIMEOUT 20 // 200 ms

///////////////////////////////////////////////////////////////////////////////

int main() {
	init();

	while (true) {
		if (scom_to_update) {
			outputs_changed_when_setting_scom = false;
			scom_update();
			scom_to_update = false;
			if (outputs_changed_when_setting_scom)
				outputs_apply_state();
		}
		if (inputs_debounce_to_update) {
			inputs_debounce_update();
			inputs_debounce_to_update = false;
		}
		if (ir_debounce_to_update) {
			ir_debounce_update();
			ir_debounce_to_update = false;
		}

		if (config_write) {
			config_save();
			config_write = false;
		}

		if (!ir_shift_disable) {
			io_shift_update();
			outputs_enable();
			if (on_shift_scanned != NULL) {
				on_shift_scanned();
				on_shift_scanned = NULL;
			}
		}

		wdt_reset();
		_delay_us(50);
	}
}

static inline void init() {
	// Disable watchdog
	cli();
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0x00;

	// Check reset flags

	uint16_t bootloader_version = config_bootloader_version();
	if ((bootloader_version >= 0x0102) && (bootloader_version != 0xFFFF))
		mcusr.all = config_mcusr();
	else
		mcusr.all = MCUSR;
	MCUSR = 0;

	if (config_is_int_wdrf()) {
		mcusr.bits.wdrf = false;
		config_int_wdrf(false);
	}
	mcusr.bits.borf = false; // brownout detects basically all power-on resets
	mtbbus_warn_flags.all = (mcusr.all >> 1) & 0x07;

	io_init();
	io_led_red_on();
	io_led_green_on();
	io_led_blue_on();
	scom_init();

	// Setup timer 0 @ 20 kHz (period 100 us)
	TCCR0A = (1 << WGM01); // CTC mode
	TCCR0B = (1 << CS01); // CTC mode, prescaler 8×
	TIMSK0 = (1 << OCIE0A); // enable compare match interrupt
	OCR0A = 91;

	// Setup timer 1 @ 100 Hz (period 10 ms)
	TCCR1B = (1 << WGM12) | (1 << CS11); // CRC mode, 8× prescaler
	TIMSK1 = (1 << OCIE1A); // enable compare match interrupt
	OCR1A = 18430;

	config_load();
	outputs_set_full(config_safe_state);

	io_shift_update();

	uint8_t _mtbbus_addr = io_get_addr_raw();
	error_flags.bits.addr_zero = (_mtbbus_addr == 0);
	mtbbus_init(_mtbbus_addr, config_mtbbus_speed);
	mtbbus_on_receive = mtbbus_received;
	mtbbus_on_free = mtbbus_free;

	update_mtbbus_polarity();

	if (config_ir_support > 1) {
		// measure ir cupport automatically
		config_ir_support = is_ir_support_measure();
		config_save_ir_support();
	}

	diag_init();

	mtbbus_warn_flags_old = 0xFF; // causes report of change to PC
	wdt_enable(WDTO_250MS);
	sei(); // enable interrupts globally
}

static inline void on_initialized() {
	io_led_red_off();
	io_led_green_off();
	io_led_blue_off();
	initialized = true;
}

void mtbbus_free() {}

ISR(TIMER0_COMPA_vect) {
	// Timer 1 @ 20 kHz (period 50 us)
	static size_t counter = 0;

	if ((!inputs_disabled) && (config_ir_support))
		ir_update_50us();

	counter++;
	if (counter >= 10) { // 2 kHz (500 us)
		inputs_debounce_to_update = true;
		counter = 0;
	}
}

ISR(TIMER1_COMPA_vect) {
	// Timer 3 @ 100 Hz (period 10 ms)
	if ((TCNT1H > 0) & (TCNT1H < OCR1AH))
		mtbbus_warn_flags.bits.missed_timer = true;

	scom_to_update = true;
	outputs_update();
	inputs_fall_update();
	leds_update();

	if (_init_counter < INIT_TIME) {
		_init_counter++;
		if (_init_counter == INIT_TIME)
			on_initialized();
	}

	if (mtbbus_timeout < MTBBUS_TIMEOUT_MAX)
		mtbbus_timeout++;

	if ((btn_pressed) && (btn_press_time < 100)) {
		btn_press_time++;
		if (btn_press_time >= 100)
			btn_long_press();
	}

	if (mtbbus_auto_speed_in_progress) {
		mtbbus_auto_speed_timer++;
		if (mtbbus_auto_speed_timer >= MTBBUS_AUTO_SPEED_TIMEOUT)
			mtbbus_auto_speed_next();
	}

	{
		static uint8_t diag_timer = 0;
		diag_timer++;
		if (diag_timer >= DIAG_UPDATE_PERIOD) {
			diag_update();
			diag_timer = 0;
		}
	}

}

///////////////////////////////////////////////////////////////////////////////

static inline void leds_update() {
	if (led_gr_counter > 0) {
		led_gr_counter--;
		if (led_gr_counter == LED_GR_OFF)
			io_led_green_off();
	}

	bool led_red_flashing = error_flags.all;

	if (led_red_counter > 0) {
		led_red_counter--;
		if (((!led_red_flashing) && (led_red_counter == LED_RED_OK_OFF)) ||
			((led_red_flashing) && (led_red_counter == LED_RED_ERR_OFF)))
			io_led_red_off();
	}
	if ((led_red_flashing) && (led_red_counter == 0)) {
		led_red_counter = LED_RED_ERR_ON;
		io_led_red_on();
	}

	if (led_blue_counter > 0) {
		led_blue_counter--;
		if (led_blue_counter == LED_BLUE_BEACON_OFF)
			io_led_blue_off();
	}
	if ((beacon) && (led_blue_counter == 0)) {
		led_blue_counter = LED_BLUE_BEACON_ON;
		io_led_blue_on();
	}
}

void led_red_ok() {
	if (led_red_counter == 0) {
		led_red_counter = LED_RED_OK_ON;
		io_led_red_on();
	}
}

///////////////////////////////////////////////////////////////////////////////

void btn_on_pressed() {
	btn_press_time = 0;
}

void btn_on_depressed() {
	if (btn_press_time < 100) // < 1 s
		btn_short_press();
}

static inline void btn_short_press() {
	if (mtbbus_auto_speed_in_progress) {
		autodetect_mtbbus_speed_stop();
		return;
	}

	uint8_t _mtbbus_addr = io_get_addr_raw();
	error_flags.bits.addr_zero = (_mtbbus_addr == 0);
	mtbbus_addr = _mtbbus_addr;
	if (mtbbus_addr != 0)
		led_red_ok();
	update_mtbbus_polarity();
}

static inline void btn_long_press() {
	if (!mtbbus_addressed())
		autodetect_mtbbus_speed();
}

///////////////////////////////////////////////////////////////////////////////

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) {
	if (!initialized)
		return;

	error_flags.bits.bad_mtbbus_polarity = false;
	if (led_gr_counter == 0) {
		io_led_green_on();
		led_gr_counter = LED_GR_ON;
	}
	_delay_us(2);

	mtbbus_timeout = 0;
	if (mtbbus_auto_speed_in_progress)
		mtbbus_auto_speed_received();

	if ((!broadcast) && (command_code == MTBBUS_CMD_MOSI_MODULE_INQUIRY) && (data_len >= 1)) {
		static bool last_input_changed = false;
		static bool last_diag_changed = false;
		static bool first_scan = true;
		bool last_ok = data[0] & 0x01;

		if ((inputs_logic_state != inputs_old) || (last_input_changed && !last_ok) || (first_scan)) {
			// Send inputs changed
			last_input_changed = true;
			first_scan = false;
			mtbbus_send_inputs(MTBBUS_CMD_MISO_INPUT_CHANGED);
			inputs_old = inputs_logic_state;
		} else {
			last_input_changed = false;

			if ((mtbbus_warn_flags.all != mtbbus_warn_flags_old.all) || (last_diag_changed && !last_ok)) {
				last_diag_changed = true;
				mtbbus_warn_flags_old = mtbbus_warn_flags;
				send_diag_value(MTBBUS_DV_STATE);
			} else {
				mtbbus_send_ack();
			}
		}

	} else if ((command_code == MTBBUS_CMD_MOSI_INFO_REQ) && (!broadcast)) {
		uint16_t bootloader_ver = config_bootloader_version();
		mtbbus_output_buf[0] = 9;
		mtbbus_output_buf[1] = MTBBUS_CMD_MISO_MODULE_INFO;
		mtbbus_output_buf[2] = CONFIG_MODULE_TYPE;
		mtbbus_output_buf[3] = (mtbbus_warn_flags.all > 0) << 2;
		mtbbus_output_buf[4] = CONFIG_FW_MAJOR;
		mtbbus_output_buf[5] = CONFIG_FW_MINOR;
		mtbbus_output_buf[6] = CONFIG_PROTO_MAJOR;
		mtbbus_output_buf[7] = CONFIG_PROTO_MINOR;
		mtbbus_output_buf[8] = bootloader_ver >> 8;
		mtbbus_output_buf[9] = bootloader_ver & 0xFF;
		mtbbus_send_buf_autolen();

	} else if ((command_code == MTBBUS_CMD_MOSI_SET_CONFIG) && (data_len >= 24) && (!broadcast)) {
		for (size_t i = 0; i < NO_OUTPUTS; i++)
			config_safe_state[i] = data[i];
		for (size_t i = 0; i < NO_OUTPUTS/2; i++)
			config_inputs_delay[i] = data[NO_OUTPUTS+i];
		if (data_len >= 26) {
			config_ir_inputs = data[24] << 8;
			config_ir_inputs |= data[25];
		} else {
			config_ir_inputs = 0;
		}
		config_write = true;
		mtbbus_send_ack();

	} else if ((command_code == MTBBUS_CMD_MOSI_GET_CONFIG) && (!broadcast)) {
		mtbbus_output_buf[0] = 25;
		mtbbus_output_buf[1] = MTBBUS_CMD_MISO_MODULE_CONFIG;
		for (size_t i = 0; i < NO_OUTPUTS; i++)
			mtbbus_output_buf[2+i] = config_safe_state[i];
		for (size_t i = 0; i < NO_OUTPUTS/2; i++)
			mtbbus_output_buf[2+NO_OUTPUTS+i] = config_inputs_delay[i];
		data[25] = config_ir_inputs >> 8;
		data[26] = config_ir_inputs & 0xFF;
		mtbbus_send_buf_autolen();

	} else if ((command_code == MTBBUS_CMD_MOSI_BEACON) && (data_len >= 1)) {
		beacon = data[0];
		if (!broadcast)
			mtbbus_send_ack();

	} else if ((command_code == MTBBUS_CMD_MOSI_GET_INPUT) && (!broadcast)) {
		mtbbus_send_inputs(MTBBUS_CMD_MISO_INPUT_STATE);

	} else if ((command_code == MTBBUS_CMD_MOSI_SET_OUTPUT) && (data_len >= 4) && (!broadcast)) {
		outputs_set_zipped(data, data_len);
		outputs_changed_when_setting_scom = true;

		mtbbus_output_buf[0] = data_len+1;
		mtbbus_output_buf[1] = MTBBUS_CMD_MISO_OUTPUT_SET;
		for (size_t i = 0; i < data_len; i++)
			mtbbus_output_buf[2+i] = data[i];
		mtbbus_send_buf_autolen();

	} else if (command_code == MTBBUS_CMD_MOSI_RESET_OUTPUTS) {
		outputs_set_full(config_safe_state);

		if (!broadcast)
			mtbbus_send_ack();

	} else if ((command_code == MTBBUS_CMD_MOSI_CHANGE_ADDR) && (data_len >= 1) && (!broadcast)) {
		mtbbus_send_error(MTBBUS_ERROR_UNSUPPORTED_COMMAND);

	} else if ((command_code == MTBBUS_CMD_MOSI_CHANGE_SPEED) && (data_len >= 1)) {
		config_mtbbus_speed = data[0];
		config_write = true;
		mtbbus_set_speed(data[0]);

		if (!broadcast)
			mtbbus_send_ack();

	} else if ((command_code == MTBBUS_CMD_MOSI_FWUPGD_REQUEST) && (data_len >= 1) && (!broadcast)) {
		config_boot_fwupgd();
		mtbbus_on_sent = &goto_bootloader;
		mtbbus_send_ack();

	} else if (command_code == MTBBUS_CMD_MOSI_REBOOT) {
		if (broadcast) {
			goto_bootloader();
		} else {
			mtbbus_on_sent = &goto_bootloader;
			mtbbus_send_ack();
		}

	} else if ((command_code == MTBBUS_CMD_MOSI_SPECIFIC) && (data_len >= 2)) {
		if (data[0] == 0x01) {
			if (data[1] > 1)
				config_ir_support = is_ir_support_measure();
			else
				config_ir_support = data[1];
			config_write = true;
			if (!broadcast)
				mtbbus_send_ack();
			led_red_ok();
		} else {
			mtbbus_send_error(MTBBUS_ERROR_UNKNOWN_COMMAND);
		}

	} else if ((command_code == MTBBUS_CMD_MOSI_DIAG_VALUE_REQ) && (data_len >= 1)) {
		send_diag_value(data[0]);

	} else {
		if (!broadcast)
			mtbbus_send_error(MTBBUS_ERROR_UNKNOWN_COMMAND);
	}
}

// Warning: functions below don't check mtbbus_can_fill_output_buf(), bacause
// they should be called ONLY from mtbbus_received event (as MTBbus is
// request-response based bus).

void mtbbus_send_ack() {
	mtbbus_output_buf[0] = 1;
	mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ACK;
	mtbbus_send_buf_autolen();
}

void mtbbus_send_inputs(uint8_t message_code) {
	mtbbus_output_buf[0] = 3;
	mtbbus_output_buf[1] = message_code;
	mtbbus_output_buf[2] = (inputs_logic_state >> 8) & 0xFF;
	mtbbus_output_buf[3] = inputs_logic_state & 0xFF;
	mtbbus_send_buf_autolen();
}

void mtbbus_send_error(uint8_t code) {
	mtbbus_output_buf[0] = 2;
	mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ERROR;
	mtbbus_output_buf[2] = code;
	mtbbus_send_buf_autolen();
}

///////////////////////////////////////////////////////////////////////////////

void goto_bootloader() {
	config_int_wdrf(true);
	wdt_enable(WDTO_15MS);
	while (true);
}

static inline void update_mtbbus_polarity() {
	error_flags.bits.bad_mtbbus_polarity = !((PIND >> PIN_UART_RX) & 0x1);
}

///////////////////////////////////////////////////////////////////////////////

bool is_ir_support_measure() {
	DDRC |= (1 << PIN_IR_PULSE);
	PORTC &= ~(1 << PIN_IR_PULSE);
	_delay_us(1); // wait for capacitor to discharge

	DDRC &= ~(1 << PIN_IR_PULSE);
	PORTC |= (1 << PIN_IR_PULSE); // enable pull-up, slowly charge capacitor up
	_delay_us(3);

	bool is = !((PINC >> PIN_IR_PULSE) & 0x1);

	// restore output
	PORTC &= ~(1 << PIN_IR_PULSE);
	DDRC |= (1 << PIN_IR_PULSE);

	return is;
}

///////////////////////////////////////////////////////////////////////////////

static inline bool mtbbus_addressed() {
	return mtbbus_timeout < MTBBUS_TIMEOUT_MAX;
}

///////////////////////////////////////////////////////////////////////////////

static inline void autodetect_mtbbus_speed() {
	io_led_blue_on();
	mtbbus_auto_speed_in_progress = true;
	mtbbus_auto_speed_last = 0; // relies on first speed 38400 kBd = 0x01
	mtbbus_auto_speed_next();
}

void mtbbus_auto_speed_next() {
	mtbbus_auto_speed_timer = 0;
	mtbbus_auto_speed_last++; // relies on continuous interval of speeds
	if (mtbbus_auto_speed_last > MTBBUS_SPEED_115200)
		mtbbus_auto_speed_last = 1;
	mtbbus_set_speed(mtbbus_auto_speed_last);
}

static inline void mtbbus_auto_speed_received() {
	mtbbus_auto_speed_in_progress = false;
	config_mtbbus_speed = mtbbus_auto_speed_last;
	config_write = true;
	io_led_blue_off();
}

static inline void autodetect_mtbbus_speed_stop() {
	if (mtbbus_auto_speed_in_progress) {
		mtbbus_auto_speed_in_progress = false;
		io_led_blue_off();
	}
}

///////////////////////////////////////////////////////////////////////////////

void send_diag_value(uint8_t i) {
	mtbbus_output_buf[1] = MTBBUS_CMD_MISO_DIAG_VALUE;
	mtbbus_output_buf[2] = i;

	switch (i) {
	case MTBBUS_DV_VERSION:
		mtbbus_output_buf[0] = 2+1;
		mtbbus_output_buf[3] = 0x10;
		break;

	case MTBBUS_DV_STATE:
		mtbbus_output_buf[0] = 2+1;
		mtbbus_output_buf[3] = (mtbbus_warn_flags.all > 0) << 1;
		break;

	case MTBBUS_DV_UPTIME:
		mtbbus_output_buf[0] = 2+4;
		mtbbus_output_buf[3] = (uptime_seconds >> 24);
		mtbbus_output_buf[4] = (uptime_seconds >> 16) & 0xFF;
		mtbbus_output_buf[5] = (uptime_seconds >> 8) & 0xFF;
		mtbbus_output_buf[6] = (uptime_seconds) & 0xFF;
		break;

	case MTBBUS_DV_WARNINGS:
		mtbbus_warn_flags_old = mtbbus_warn_flags;
		mtbbus_output_buf[0] = 2+1;
		mtbbus_output_buf[3] = mtbbus_warn_flags.all;
		break;

	case MTBBUS_DV_VMCU:
		mtbbus_output_buf[0] = 2+2;
		mtbbus_output_buf[3] = vcc_voltage >> 8;
		mtbbus_output_buf[4] = vcc_voltage & 0xFF;
		break;

	case MTBBUS_DV_TEMPMCU:
		mtbbus_output_buf[0] = 2+4;
		mtbbus_output_buf[3] = mcu_temperature >> 8;
		mtbbus_output_buf[4] = mcu_temperature & 0xFF;
		mtbbus_output_buf[5] = ts_offset;
		mtbbus_output_buf[6] = ts_gain;
		break;

	default:
		mtbbus_output_buf[0] = 2+0;
		mtbbus_warn_flags_old = mtbbus_warn_flags;
	}

	mtbbus_send_buf_autolen();
}

///////////////////////////////////////////////////////////////////////////////
