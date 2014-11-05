/*
 * This file is part of the libopencm3 project.
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
#include <libopencm3/stm32/dac.h>

#define LED_DISCO_PORT      GPIOD
#define LED_DISCO_GREEN_PIN GPIO12
#define LED_DISCO_LRED_PIN  GPIO13
#define LED_DISCO_RED_PIN   GPIO14
#define LED_DISCO_BLUE_PIN  GPIO15


static void clock_setup(void);
static void gpio_setup(void);
static void adc_setup(void);
//static void dac_setup(void);
static uint16_t read_adc_naiive(uint8_t channel);

//-----------------------------------------------------
int main(void)
{
   int i, j=0;
   clock_setup();
   gpio_setup();
   adc_setup();
  // dac_setup();

   gpio_set(LED_DISCO_PORT, LED_DISCO_LRED_PIN);



   /* Blink the LED (PC8) on the board. */
   while (1) {
      /*
      uint16_t input_adc0 = read_adc_naiive(0);
      uint16_t target = input_adc0 / 2;
      uint16_t input_adc1 = read_adc_naiive(1);
      */

      /* Using API function gpio_toggle(): */
      gpio_toggle(LED_DISCO_PORT, LED_DISCO_GREEN_PIN |
                                  LED_DISCO_LRED_PIN |
                                  LED_DISCO_RED_PIN |
                                  LED_DISCO_BLUE_PIN);   /* LED on/off */

      for (i = 0; i < 9000000; i++) {   /* Wait a bit. */
         __asm__("nop");
      }

      /*
      printf("tick: %d: adc0= %u, target adc1=%d, adc1=%d\n",
            j++, input_adc0, target, input_adc1);
      */
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
   rcc_periph_clock_enable(RCC_ADC1);
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


}

//------------------------------------------------------
static void adc_setup(void)
{
   adc_off(ADC1);
   adc_disable_scan_mode(ADC1);
   adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);

   adc_power_on(ADC1);

}

//------------------------------------------------------
static uint16_t read_adc_naiive(uint8_t channel)
{
   uint8_t channel_array[16];
   channel_array[0] = channel;
   adc_set_regular_sequence(ADC1, 1, channel_array);
   adc_start_conversion_regular(ADC1);
   while (!adc_eoc(ADC1));
   uint16_t reg16 = adc_read_regular(ADC1);
   return reg16;
}

//------------------------------------------------------
/*static void dac_setup(void)
{
   gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);
   dac_disable(CHANNEL_2);
   dac_disable_waveform_generation(CHANNEL_2);
   dac_enable(CHANNEL_2);
   dac_set_trigger_source(DAC_CR_TSEL2_SW);
}*/

