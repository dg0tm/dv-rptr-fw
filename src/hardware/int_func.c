/*
 * int_func.c
 *
 * implements a simple EIC API to use external interrupt sources.
 * A simple idle timer (keep AVR32 µC alive to toggle external watchdog via main-loop)
 * is also a part of this file.
 *
 *
 *  Created on: 20.03.2009
 *      Author: DO1FJN, Jan Alte
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


#include "int_func.h"
#include "hw_defs.h"

#include <compiler.h>

#define IDLE_TIMER		AVR32_TC.channel[IDLE_TIMER_CH]
#define	IDLE_TIMER_IRQ		(AVR32_TC_IRQ0+IDLE_TIMER_CH)

#define IDLE_PERIOD		CALC_CLOCKS_FROM(25)	// 25ms

/*! \name External Interrupt Controller Functions
 */
//! @{


void eic_enableint(unsigned int intnum, __int_handler handler, unsigned int int_lev, teictrigmode mode) {
  if (intnum < 8) {
    switch (mode) {
    case EICm_FALLING_EDGE:
      AVR32_EIC.edge &= ~(1<<intnum);
      AVR32_EIC.mode &= ~(1<<intnum);
      break;
    case EICm_RAISING_EDGE:
      AVR32_EIC.edge |= (1<<intnum);
      AVR32_EIC.mode &= ~(1<<intnum);
      break;
    case EICm_LOW_LEVEL:
      AVR32_EIC.level &= ~(1<<intnum);
      AVR32_EIC.mode |= (1<<intnum);
      break;
    case EICm_HIGH_LEVEL:
      AVR32_EIC.level |= (1<<intnum);
      AVR32_EIC.mode |= (1<<intnum);
      break;
    }
    AVR32_EIC.en = (1 << intnum);
    INTC_register_interrupt(handler, AVR32_EIC_IRQ_0+intnum, int_lev);
    AVR32_EIC.ier = (1 << intnum); 	// Int scharf
  }
}


void eic_disableint(unsigned int intnum) {
  if (intnum < 8) {
    AVR32_EIC.idr = (1 << intnum);
    AVR32_EIC.dis = (1 << intnum);
  } // fi
}


void eic_reenableint(unsigned int intnum) {
  if (intnum < 8) {
    AVR32_EIC.en = (1 << intnum);
    AVR32_EIC.ier = (1 << intnum);
  } // fi
}

void eic_changemode(unsigned int intnum, teictrigmode mode) {
  if (intnum < 8) {
    int ena_int = AVR32_EIC.imr & (1 << intnum);
    eic_disableint(intnum);
    switch (mode) {
    case EICm_FALLING_EDGE:
      AVR32_EIC.edge &= ~(1<<intnum);
      AVR32_EIC.mode &= ~(1<<intnum);
      break;
    case EICm_RAISING_EDGE:
      AVR32_EIC.edge |= (1<<intnum);
      AVR32_EIC.mode &= ~(1<<intnum);
      break;
    case EICm_LOW_LEVEL:
      AVR32_EIC.level &= ~(1<<intnum);
      AVR32_EIC.mode |= (1<<intnum);
      break;
    case EICm_HIGH_LEVEL:
      AVR32_EIC.level |= (1<<intnum);
      AVR32_EIC.mode |= (1<<intnum);
      break;
    }
    if (ena_int) eic_reenableint(intnum);
  }
}


void eic_clrline(unsigned int intnum) {
  Bool global_interrupt_enabled;
  // Clear line line_number
  if (intnum < 8) {
    global_interrupt_enabled = Is_global_interrupt_enabled();
    if (global_interrupt_enabled) Disable_global_interrupt();
    AVR32_EIC.icr = (1 << intnum);
    AVR32_EIC.isr;
    if (global_interrupt_enabled) Enable_global_interrupt();
  } // fi
}


void eic_setfilter(unsigned int intnum, int filtered) {
  if (intnum < 8) {
    if (filtered) {
      AVR32_EIC.filter |= (1 << intnum);
    } else {
      AVR32_EIC.filter &= ~(1 << intnum);
    }
  }
}

__inline int eic_is_raising_edge(unsigned int intnum) {
  return (AVR32_EIC.edge&(1<<intnum));
}

//! @}


/*! \name IDLE Timer Function
 */
//! @{

tidleproc idle_handler;


INTERRUPT_FUNC idle_timer_func(void) {	// Weckt den µC aus Sleep, Watchdog in Mainloop
  IDLE_TIMER.sr;        // Read Status to acknowledge INT
  if (idle_handler != NULL) idle_handler();
}



void idle_timer_stop(void) {
  IDLE_TIMER.idr = 0xFF;
  IDLE_TIMER.ccr = AVR32_TC_CLKDIS_MASK;
  IDLE_TIMER.sr;
}


void idle_timer_custom(tidleproc handler, unsigned short clocks) {
  idle_timer_stop();
  // *** Timer: Periodisch, Up bis RC-Compare, AutoTrg ***
  IDLE_TIMER.cmr = AVR32_TC_NONE << AVR32_TC_BSWTRG_OFFSET |
      AVR32_TC_NONE << AVR32_TC_BEEVT_OFFSET |
      AVR32_TC_NONE << AVR32_TC_BCPC_OFFSET |
      AVR32_TC_NONE << AVR32_TC_BCPB_OFFSET |
      AVR32_TC_NONE << AVR32_TC_ASWTRG_OFFSET |
      AVR32_TC_NONE << AVR32_TC_AEEVT_OFFSET |
      AVR32_TC_NONE << AVR32_TC_ACPC_OFFSET |
      AVR32_TC_NONE << AVR32_TC_ACPA_OFFSET |
      1 << AVR32_TC_WAVE_OFFSET |
      AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET |	// Up, RC-Trigger resets counter
      FALSE << AVR32_TC_ENETRG_OFFSET |
      AVR32_TC_EEVT_TIOB_INPUT << AVR32_TC_EEVT_OFFSET |
      AVR32_TC_EEVTEDG_NO_EDGE << AVR32_TC_EEVTEDG_OFFSET |
      FALSE << AVR32_TC_CPCDIS_OFFSET |
      FALSE << AVR32_TC_CPCSTOP_OFFSET |
      AVR32_TC_BURST_NOT_GATED << AVR32_TC_BURST_OFFSET |
      0 << AVR32_TC_CLKI_OFFSET |
      AVR32_TC_TCCLKS_TIMER_CLOCK5 << AVR32_TC_TCCLKS_OFFSET;	// MASTERCLK/128
  INTC_register_interrupt(&idle_timer_func, IDLE_TIMER_IRQ, AVR32_INTC_INT0);
  idle_handler   = handler;
  IDLE_TIMER.rc  = clocks;
  IDLE_TIMER.ier = AVR32_TC_CPCS_MASK;	// Counter Compare-C
  IDLE_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}


void idle_timer_start(void) {
  idle_timer_stop();
  // *** Timer: Periodisch, Up bis RC-Compare, AutoTrg ***
  IDLE_TIMER.cmr = AVR32_TC_NONE << AVR32_TC_BSWTRG_OFFSET |
    AVR32_TC_NONE << AVR32_TC_BEEVT_OFFSET |
    AVR32_TC_NONE << AVR32_TC_BCPC_OFFSET |
    AVR32_TC_NONE << AVR32_TC_BCPB_OFFSET |
    AVR32_TC_NONE << AVR32_TC_ASWTRG_OFFSET |
    AVR32_TC_NONE << AVR32_TC_AEEVT_OFFSET |
    AVR32_TC_NONE << AVR32_TC_ACPC_OFFSET |
    AVR32_TC_NONE << AVR32_TC_ACPA_OFFSET |
    1 << AVR32_TC_WAVE_OFFSET |
#ifdef IDLE_PERIOD
    AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET |		// Up, RC-Trigger resets counter
#else
    AVR32_TC_WAVSEL_UP_NO_AUTO << AVR32_TC_WAVSEL_OFFSET |      // Up, ohne RC-Trigger
#endif
    FALSE << AVR32_TC_ENETRG_OFFSET |
    AVR32_TC_EEVT_TIOB_INPUT << AVR32_TC_EEVT_OFFSET |
    AVR32_TC_EEVTEDG_NO_EDGE << AVR32_TC_EEVTEDG_OFFSET |
    FALSE << AVR32_TC_CPCDIS_OFFSET |
    FALSE << AVR32_TC_CPCSTOP_OFFSET |
    AVR32_TC_BURST_NOT_GATED << AVR32_TC_BURST_OFFSET |
    0 << AVR32_TC_CLKI_OFFSET |
    AVR32_TC_TCCLKS_TIMER_CLOCK5 << AVR32_TC_TCCLKS_OFFSET;     // MASTERCLK/128
  INTC_register_interrupt(&idle_timer_func, IDLE_TIMER_IRQ, AVR32_INTC_INT0);
  idle_handler   = NULL;
#ifdef IDLE_PERIOD
  IDLE_TIMER.rc  = IDLE_PERIOD;
  IDLE_TIMER.ier = AVR32_TC_CPCS_MASK;	// Counter Compare-C
#else
  IDLE_TIMER.ier = AVR32_TC_COVFS_MASK; // Counter Overflow
#endif
  IDLE_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}



//! @}
