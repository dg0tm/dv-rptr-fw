/*
 * Standard DV-RPTR Firmware
 * -------------------------
 * This project is firmware of the AVR32 DV-RPTR, developed by J. Alte.
 * It processes various converting modes for digitized speech signals.
 * See www.digisolutions.de (german only) for more information about DV-Modem.
 *
 * Copyright (c) 2011 by Jan Alte, DO1FJN
 *
 *
 * rptr_main.c (main source file of DVfirmware)
 *
 *  Created on: 2011-07-26 (developed from DV-Modem)
 *      Author: Jan Alte, DO1FJN
 *
 *
 * This firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * All source files of DVfirmware are written in UTF-8 charset.
 *
 * Report:
 * 2011-07-26 V0.01  Erstes Firmware-Release
 *
 *
 * ToDo:
 * ALL!!! This Version is only a sum of files, copied from DV-Firmware to
 * build a new (smaller) project for DV-RPTR
 */


#include "defines.h"
#include "rptr_func.h"
#include "slowdata.h"

#include "gpio_func.h"

#include "usb_func.h"

#include "intc.h"
#include "compiler.h"

// *** Globale Variablen ***


//tOPmode		OpMode	  = OpUnknown;




int main(void) {
  Disable_global_interrupt();		// Disable all interrupts.

  // *** Initialization-Section ***
  init_hardware();
  INTC_init_interrupts();		// Initialize interrupt vectors.

  //device_hw_Init();
  usb_init();				// Enable VBUS-Check

  // *** Initialierung der verschiedenen Parameter ***
  dstar_init_data();			// Default Header "noCall"

  Enable_global_interrupt();		// Enable all interrupts.

  //device_Startup();

  while (TRUE) {			// *** Main-Loop ***
    SLEEP();				// erst einmal dösen bis zu einem IRQ
    watchdog();
    // USB-Funktion
    usb_handler();

  } // ehliw MAINloop
}

// MAIN Ende

