/*
 * gpio_func.c
 *
 * API functions to control CPLD and LEDs via general pin i/o.
 * Includes "init_hardware()" to setup Mainclock and configure GPIO-Pins and
 * a watchdog function (toggle external WD-Line)
 *
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
 *
 *  Report:
 *  2010-01-16	Ausgemistet, Disable SLIP by Default
 *  2011-08-30	Watchdog uses a PortB Pin on Target DVRPTR
 *
 */


#include "gpio_func.h"
#include "hw_defs.h"
#include "compiler.h"


__inline void gpio0_set(unsigned int pin) {
  AVR32_GPIO.port[0].ovrs = 1 << pin;
}

__inline void gpio0_clr(unsigned int pin) {
  AVR32_GPIO.port[0].ovrc = 1 << pin;
}

__inline void gpio0_tgl(unsigned int pin) {
  AVR32_GPIO.port[0].ovrt = 1 << pin;
}

__inline unsigned int gpio0_readpin(unsigned int pin) {
  return (AVR32_GPIO.port[0].pvr & (1 << pin));
}

__inline unsigned int gpio0_readovr(unsigned int pin) {
  return (AVR32_GPIO.port[0].ovr & (1 << pin));
}

__inline void gpio1_set(unsigned int pin) {
  AVR32_GPIO.port[1].ovrs = 1 << (pin&0x1F);
}

__inline void gpio1_clr(unsigned int pin) {
  AVR32_GPIO.port[1].ovrc = 1 << (pin&0x1F);
}

__inline void gpio1_tgl(unsigned int pin) {
  AVR32_GPIO.port[1].ovrt = 1 << (pin&0x1F);
}

__inline unsigned int gpio1_readpin(unsigned int pin) {
  return (AVR32_GPIO.port[1].pvr & (1 << (pin&0x1F)));
}



__inline void watchdog(void) {
#ifdef DVRPTR
  if (Get_system_register(AVR32_COUNT)&0x01000000) {
    gpio1_set(WATCHDOG_PIN);	// toggle external Watchdog in slow freq.
  } else {
    gpio1_clr(WATCHDOG_PIN);
  }
#else
  if (Get_system_register(AVR32_COUNT)&0x01000000) {
    gpio0_set(WATCHDOG_PIN);	// toggle external Watchdog in slow freq.
  } else {
    gpio0_clr(WATCHDOG_PIN);
  }
#endif
}



void init_hardware(void) {
  // 1. Enable OSC0 with 12MHz Crystal
  AVR32_PM.oscctrl0 = (7<<AVR32_PM_OSCCTRL0_MODE_OFFSET)
    |(OSC0STARTUPVALUE<<AVR32_PM_OSCCTRL0_STARTUP_OFFSET);
  AVR32_PM.mcctrl = AVR32_PM_MCCTRL_OSC0EN_MASK;	// slow-clock + OSC0 on
  // 2. Program PLL
  AVR32_PM.pll[0] = 0x2009010d;
  // (MUL=10/DIV=1)
  // 3. Init GPIO-Pins
  AVR32_GPIO.port[0].puer  = GPIO0_PULLUP;
  AVR32_GPIO.port[0].ovr   = GPIO0_OVRINIT;
  AVR32_GPIO.port[0].oder  = GPIO0_ODER;		// Output-Enable
  AVR32_GPIO.port[0].pmr0  = GPIO0_PMR0;
  AVR32_GPIO.port[0].pmr1  = GPIO0_PMR1;
  AVR32_GPIO.port[0].gperc = GPIO0_DISABLE_MASK;	// Disable GPIO0 control
  AVR32_GPIO.port[1].puer  = GPIO1_PULLUP;		// *** PortB (CFG, SSC, RS232) ***
  AVR32_GPIO.port[1].ovr   = GPIO1_OVRINIT;
  AVR32_GPIO.port[1].oder  = GPIO1_ODER;		// Output-Enable
  AVR32_GPIO.port[1].pmr0  = GPIO1_PMR0;
  AVR32_GPIO.port[1].pmr1  = GPIO1_PMR1;
  AVR32_GPIO.port[1].gperc = GPIO1_DISABLE_MASK;
  // 3. Set Waitstate for Flash (Clock > 30MHz)
  AVR32_FLASHC.fcr |= AVR32_FLASHC_FCR_FWS_MASK;
  // 4. Wait PLL stabilize; pm_wait_for_pll0_locked(pm);
  while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_LOCK0_MASK));
  // 5. Switch Mainclock to PLL0
  AVR32_PM.mcctrl  = AVR32_PM_MCSEL_PLL0|AVR32_PM_MCCTRL_OSC0EN_MASK;
  AVR32_PM.pbamask = PBAMASK_NEEDED;
  AVR32_PM.pbbmask = PBBMASK_NEEDED;
}



#define PLL_96MHZ_VALUE	0x2007010d		// 96MHz/2 -> 48MHz for USB

int init_usb_hardware(void) {
  if (AVR32_PM.pll[1] == PLL_96MHZ_VALUE) {
    if (AVR32_PM.poscsr & AVR32_PM_POSCSR_LOCK1_MASK) {
      AVR32_PM.gcctrl[AVR32_PM_GCLK_USBB] = AVR32_PM_GCCTRL_CEN_MASK |	\
        AVR32_PM_GCCTRL_PLLSEL_MASK | AVR32_PM_GCCTRL_OSCSEL_MASK;
      return TRUE;
    } // fi PLL1 is locked
  } else {
    AVR32_PM.pbbmask = PBBMASK_NEEDED | PBBMASK_USBB;	// Enable Power for USB Module
    AVR32_PM.pll[1]  = PLL_96MHZ_VALUE;
  }
  return FALSE;
}


void exit_usb_hardware(void) {
  AVR32_PM.gcctrl[AVR32_PM_GCLK_USBB] = 0;	// Turn off generic clock
  AVR32_PM.pll[1] = 0;				// Turn Off PLL1
  AVR32_PM.pbbmask = PBBMASK_NEEDED;		// Disable Power for USB Module
}


