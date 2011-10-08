/*
 * gpio_func.h
 *
 * Defines all pseudo functions to control GPIO-based hardware (LEDs, Switches).
 * Function prototypes of init_hardware, cpld_config and external watchdog.
 *
 *  Created on: 21.03.2009
 *      Author: Jan Alte, DO1FJN
 *
 *
 * This file is part of the DV-modem firmware (DVfirmware).
 * For general information about DVfirmware see "dv-main.c".
 *
 * DVfirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The DVfirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */


#ifndef GPIO_FUNC_H_
#define GPIO_FUNC_H_

typedef enum {
  CPLDdisabled, CPLDcfg_ambeact, CPLDcfg_codec, CPLDcfg_ambepas
} tcpldmode;

#ifndef DEBUG
#define SLEEP()			__asm__ __volatile__ ("sleep 0");
#define FREEZE()		__asm__ __volatile__ ("sleep 1");
#else
#define SLEEP()
#define FREEZE()
#endif

// LED's
#define leds_off()	{ gpio0_set(RED_LED); gpio0_set(GREEN_LED); }
#define leds_red()	{ gpio0_clr(RED_LED); gpio0_set(GREEN_LED); }
#define leds_green()	{ gpio0_set(RED_LED); gpio0_clr(GREEN_LED); }
#define leds_yellow()	{ gpio0_clr(RED_LED); gpio0_clr(GREEN_LED); }

void	gpio0_set(unsigned int pin);
void	gpio0_clr(unsigned int pin);
void	gpio0_tgl(unsigned int pin);

unsigned int gpio0_readpin(unsigned int pin);
unsigned int gpio0_readovr(unsigned int pin);

// cpld_operate() löst RESET des AD73311|AMBE2020 (beide nur bei AMBE-mode).
//void cpld_operate(void);

void	watchdog(void);

// Initialisierung der CPU, GPIOs (Functions) und General Clocks
void	init_hardware(void);

// Einschalten / Abschalten der USB-Peripherie (Power + PLL1 + Generic-Clock)
int	init_usb_hardware(void);
void	exit_usb_hardware(void);


#endif // GPIO_FUNC_H_
