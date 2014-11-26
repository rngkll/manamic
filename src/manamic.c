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
static void record(void);

//-----------------------------------------------------
int main(void)
{
   int i;
   //int j=0;
   clock_setup();
   gpio_setup();




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
		record();

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

   /* Enable ADC clock */
   //rcc_periph_clock_enable(RCC_ADC1);
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
static void record(void)
{
	
	//spi_clean_disable(SPI2);
   //set clock
   rcc_periph_clock_enable(RCC_SPI2);
	spi_set_baudrate_prescaler(SPI2, SPI_CR1_BR_FPCLK_DIV_32);
	gpio_set(LED_DISCO_PORT, LED_DISCO_RED_PIN);

}

//static void mic_init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR)
static void mic_init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{
	/*Enable PLLI2S clock*/
	rcc_periph_clock_enable(RCC_SPI2);

	clock_scale_t rccclkinit; 

	/* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	if ((AudioFreq & 0x7) == 0)
	{
		/* Audio frequency multiple of 8 (8/16/32/48/96/192)*/
		/* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz */
		/* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz */
		#define RCC_PERIPHCLK_I2S             ((uint32_t)0x00000001)
		uint32_t PeriphClockSelection = RCC_PERIPHCLK_I2S
		uint8_t plli2sn = 192;
		uint8_t plli2sn = 8;
		rcc_clock_setup_hse_3v3(&rccclkinit);
	}




