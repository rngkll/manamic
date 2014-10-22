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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define LED_DISCO_PORT GPIOD
#define LED_DISCO_GREEN_PIN GPIO12
#define LED_DISCO_LRED_PIN GPIO13
#define LED_DISCO_RED_PIN GPIO14
#define LED_DISCO_BLUE_PIN GPIO15

static void clock_setup(void)
{
   rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

   /* Enable GPIOD clock. */ 
   rcc_periph_clock_enable(RCC_GPIOD);
}

static void gpio_setup(void)
{
   /* Set LED_DISCO_GREEN_PIN (in GPIO port D) to 'output push-pull'. */
   /* Manually: */
   // LED_DISCO_GREEN_PORT_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((8 - 8) * 4) + 2));
   // LED_DISCO_GREEN_PORT_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((8 - 8) * 4));
   /* Using API functions: */
   gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
                           GPIO_PUPD_NONE,
                           GPIO12 | GPIO13 | GPIO14 | GPIO15);

}



int main(void)
{
   int i;
   clock_setup();
   gpio_setup();

   gpio_set(LED_DISCO_PORT, LED_DISCO_LRED_PIN);

   /* Blink the LED (PC8) on the board. */
   while (1) {


      /* Using API function gpio_toggle(): */
      gpio_toggle(LED_DISCO_PORT, LED_DISCO_GREEN_PIN |
                                  LED_DISCO_LRED_PIN |
                                  LED_DISCO_RED_PIN |
                                  LED_DISCO_BLUE_PIN);   /* LED on/off */

      for (i = 0; i < 9000000; i++) {   /* Wait a bit. */
         __asm__("nop");
      }
   }

   return 0;
}
