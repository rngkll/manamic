/*
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/spi.h>

#define LED_DISCO_PORT      GPIOD
#define LED_DISCO_GREEN_PIN GPIO12
#define LED_DISCO_LRED_PIN  GPIO13
#define LED_DISCO_RED_PIN   GPIO14
#define LED_DISCO_BLUE_PIN  GPIO15


static void clock_setup(void);
static void gpio_setup(void);
//static uint16_t read_adc_naiive(uint8_t channel);
//static void adc_setup(uint32_t adc, uint8_t channel, uint8_t time);
static void is2_setup(void);

//-----------------------------------------------------
int main(void)
{
   int i;
   //int j=0;
   clock_setup();
   gpio_setup();
	is2_setup();




   /* Blink the LED (PC8) on the board. */
   while (1) {
      /*
      printf("tick: %d: adc0= %u, target adc1=%d, adc1=%d\n",
            j++, input_adc0, target, input_adc1);
      */
		/* Using API function gpio_toggle(): */
		/*gpio_toggle(LED_DISCO_PORT, LED_DISCO_GREEN_PIN |
			   LED_DISCO_LRED_PIN |
				LED_DISCO_RED_PIN |
				LED_DISCO_BLUE_PIN);*/   /* LED on/off */
		//start recording

		for (i = 0; i < 9000000; i++) {   /* Wait a bit. */
			__asm__("nop");
		}

		gpio_clear(LED_DISCO_PORT, LED_DISCO_RED_PIN);
	
	}

   return 0;
}

//------------------------------------------------------
static void clock_setup(void)
{
   rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

   /* Enable GPIOD clock. */ 
   rcc_periph_clock_enable(RCC_GPIOD);

   /* Enable SPI2 Periph and gpio clocks */
   rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);
}

//------------------------------------------------------
static void gpio_setup(void)
{
   /* Using API functions: */
   gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
                          GPIO_PUPD_NONE,
                          GPIO12 | GPIO13 | GPIO14 | GPIO15);
   
   /* Setup adc ports */
   gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
   gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	gpio_set_af(GPIOB, GPIO_AF1, GPIO10);

}

//-------------------------------------------------------
static void is2_setup(void)
{
	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(SPI2);

	/* Set up SPI in Master mode with:
	 * Clock baud rate: 1/64 of peripheral clock frequency
	 * Clock polarity: Idle High
	 * Clock phase: Data valid on 2nd clock pulse
	 * Data frame format: 8-bit
	 * Frame format: MSB First
	 */
	spi_init_master(SPI2,
		SPI_CR1_BAUDRATE_FPCLK_DIV_64,
		SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_2,
		SPI_CR1_DFF_16BIT,
		SPI_CR1_MSBFIRST);
	 
	/* Enable SPI1 periph. */
	spi_enable(SPI2);

	/*
	 * Set NSS management to software.
	 * Note:
	 * Setting nss high is very important, even if we are controlling the GPIO
	 * ourselves this bit needs to be at least set to 1, otherwise the spi
	 * peripheral will not send any data out.
	 */
	spi_enable_software_slave_management(SPI2);
	spi_set_nss_high(SPI2);
			 
	/* Enable SPI1 periph. */
	spi_enable(SPI1);
}

