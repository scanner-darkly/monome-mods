#include <stdio.h>
#include "delay.h"
#include "compiler.h"
#include "preprocessor.h"
#include "board.h"
#include "print_funcs.h"
#include "intc.h"
#include "pm.h"
#include "gpio.h"

int main(void)
{
	// Switch main clock from internal RC to external Oscillator 0
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

	init_dbg_rs232(FOSC0);

	gpio_enable_gpio_pin(AVR32_PIN_PB00);

	print_dbg("\r\n\nstart");

	while (true) {
		delay_ms(250);
		print_dbg(".");
		gpio_tgl_gpio_pin(AVR32_PIN_PB00);
	}
}
