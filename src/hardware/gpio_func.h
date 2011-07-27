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

#ifndef DVATRX
// Enable connection between DAC & Modulator
#define enable_afout()		gpio0_clr(MODEN_PIN)
#define disable_afout()		gpio0_set(MODEN_PIN)

// RS232-Force-Off-Pin
#define rs232_force_off()	gpio0_clr(RS232OFF_PIN)
#define rs232_auto_on()		gpio0_set(RS232OFF_PIN)

// LED's
#define leds_off()	{ gpio0_set(RED_LED); gpio0_set(GREEN_LED); }
#define leds_red()	{ gpio0_clr(RED_LED); gpio0_set(GREEN_LED); }
#define leds_green()	{ gpio0_set(RED_LED); gpio0_clr(GREEN_LED); }
#define leds_yellow()	{ gpio0_clr(RED_LED); gpio0_clr(GREEN_LED); }

#else

#define enable_afout()		{ }
#define disable_afout()		{ }

#define rs232_force_off()	{ }
#define rs232_auto_on()		{ }

#endif


/*#if (defined DEBUG)
#define SetDebug0()	gpio0_set(DBG_PIN0)
#define ClrDebug0()	gpio0_clr(DBG_PIN0)
#define SetDebug1()	gpio0_set(DBG_PIN1)
#define ClrDebug1()	gpio0_clr(DBG_PIN1)
#else*/
#define SetDebug0()	{}
#define ClrDebug0()	{}
#define SetDebug1()	{}
#define ClrDebug1()	{}
//#endif



// cpld_start() übernimmt die Konfiguration (per cpld_config() eingestellt)
// und löst Reset von AMBE und AD73311 (je nach Mode).
#ifdef HW09PT
void cpld_start(void);
void cpld_stop(void);
#else
#define cpld_start()		gpio0_clr(RUNCFG_PIN)
#define cpld_stop()		gpio0_set(RUNCFG_PIN)
#endif
//void gpio_init(void);
//void gpio_enable_ssc(void);

void	gpio0_set(unsigned int pin);
void	gpio0_clr(unsigned int pin);
void	gpio0_tgl(unsigned int pin);

unsigned int gpio0_readpin(unsigned int pin);
unsigned int gpio0_readovr(unsigned int pin);

// cpld_config() setzt CPLD in den Config-Modus (AMBE2020 und Codec sind im Reset).
// Bei CPLDdisabled ist zudem der MCK abgetrennt (Stromsparen)
void	cpld_config(tcpldmode mode);

// cpld_operate() löst RESET des AD73311|AMBE2020 (beide nur bei AMBE-mode).
//void cpld_operate(void);

void	watchdog(void);

// Initialisierung der CPU, GPIOs (Functions) und General Clocks
void	init_hardware(void);

// Einschalten / Abschalten der USB-Peripherie (Power + PLL1 + Generic-Clock)
int	init_usb_hardware(void);
void	exit_usb_hardware(void);


#endif // GPIO_FUNC_H_
