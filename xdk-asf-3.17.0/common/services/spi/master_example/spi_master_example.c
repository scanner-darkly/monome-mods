/*****************************************************************************
 *
 * \file
 *
 * \brief Example of usage of the SPI Master Mode Basic Services.
 *
 ******************************************************************************/

/**
 * Copyright (c) 2010-2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*! \mainpage
 * \section intro Introduction
 * This example demonstrates how to use the SPI Master Mode Basic Services.
 *
 * \section files Main Files
 * - spi_master_example.c: port drivers example application.
 * - conf_board.h: board initialization process configuration
 * - conf_spi_master_example.h: board-specific mapping of resources used by this example
 * - spi_master.c: Part Specific SPI Master Mode Implementation
 *
 * \section spiapiinfo services/basic/spi API
 * The spi API can be found \ref spi_master.h "here".
 *
 * \section deviceinfo Device Info
 * All AVR devices can be used. This example has been tested
 * with the following setup:
 *   - Evaluation kits with a dataflash connecting through a SPI interface.
 * To make this example work on STK600 define AT45DB041 in config file.
 *
 * \section exampledescription Description of the example
 *   - Send "Read Status" command to the dataflash.
 *   - Read back the status of the dataflash.
 *   - If status "ok", both \ref SPI_EXAMPLE_LED_PIN_EXAMPLE_1 and \ref SPI_EXAMPLE_LED_PIN_EXAMPLE_2 are 'on'
 *     else \ref SPI_EXAMPLE_LED_PIN_EXAMPLE_1 is 'on' and \ref SPI_EXAMPLE_LED_PIN_EXAMPLE_2 is 'off'.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */


#include "compiler.h"
#include "preprocessor.h"
#include "board.h"
#include "gpio.h"
#include "sysclk.h"
#include "delay.h"
#include "spi.h"

#define SPI_EXAMPLE (&AVR32_SPI)

int main(void)
{
	sysclk_init();

	int i=0;

	// board_init();

	sysclk_enable_pba_module(SYSCLK_SPI);

	// spi_reset(SPI_EXAMPLE);
	// spi_set_master_mode(SPI_EXAMPLE);
	// spi_disable_modfault(SPI_EXAMPLE);
	// spi_disable_loopback(SPI_EXAMPLE);
	// spi_set_chipselect(SPI_EXAMPLE,(1 << AVR32_SPI_MR_PCS_SIZE) - 1);
	// spi_disable_variable_chipselect(SPI_EXAMPLE);
	// spi_disable_chipselect_decoding(SPI_EXAMPLE);
	// spi_set_delay(SPI_EXAMPLE,0);

	// spi_set_chipselect_delay_bct(SPI_EXAMPLE,0,0);
	// spi_set_chipselect_delay_bs(SPI_EXAMPLE,0,0);
	// spi_set_bits_per_transfer(SPI_EXAMPLE,0, 8);
	// spi_set_baudrate_register(SPI_EXAMPLE,0, getBaudDiv(1000000, sysclk_get_peripheral_bus_hz(SPI_EXAMPLE)));
	// spi_enable_active_mode(SPI_EXAMPLE,0);
	// spi_set_mode(SPI_EXAMPLE,0,SPI_MODE_0);

	  static const gpio_map_t SPI_GPIO_MAP = {
    {AT45DBX_SPI_SCK_PIN,  AT45DBX_SPI_SCK_FUNCTION },
    {AT45DBX_SPI_MISO_PIN, AT45DBX_SPI_MISO_FUNCTION},
    {AT45DBX_SPI_MOSI_PIN, AT45DBX_SPI_MOSI_FUNCTION},
    {AT45DBX_SPI_NPCS0_PIN,  AT45DBX_SPI_NPCS0_FUNCTION },
    // {AT45DBX_SPI_NPCS1_PIN,  AT45DBX_SPI_NPCS1_FUNCTION },
  };

  // Assign GPIO to SPI.
  gpio_enable_module(SPI_GPIO_MAP, sizeof(SPI_GPIO_MAP) / sizeof(SPI_GPIO_MAP[0]));

  spi_options_t spiOptions = {
    .reg = 0,
    .baudrate = 1000000,
    .bits = 8,
    .trans_delay = 0,
    .spck_delay = 0,
    .stay_act = 1,
    .spi_mode = 0,
    .modfdis = 1
  };


  // Initialize as master.
  spi_initMaster(SPI_EXAMPLE, &spiOptions);
  // Set SPI selection mode: variable_ps, pcs_decode, delay.
  spi_selectionMode(SPI_EXAMPLE, 0, 0, 0);
  // Enable SPI module.
  spi_enable(SPI_EXAMPLE);

  // spi_setupChipReg( SPI, &spiOptions, FPBA_HZ );
  spi_setupChipReg(SPI_EXAMPLE, &spiOptions, sysclk_get_pba_hz() );

	spi_enable(SPI_EXAMPLE);


	while (true)
	{
		i++;
		delay_ms(10);
			// status = spi_at45dbx_mem_check();
		spi_selectChip(SPI_EXAMPLE,0);
		spi_put(SPI_EXAMPLE,i);
		spi_unselectChip(SPI_EXAMPLE,0);

	}
}

