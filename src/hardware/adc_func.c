/*
 * adc_func.c
 *
 * implements basic adc-functions (useing internal ADC of AVR32 µC)
 *
 *  Created on: 09.03.2009
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


#include "adc_func.h"
#include "hw_defs.h"

#include <compiler.h>


#define AVR32_ADC_INTPRIO	AVR32_INTC_INT0



__inline void adc_exit(void) {
  AVR32_ADC.idr  = 0xFFFFFFFF;
  AVR32_ADC.cr   = AVR32_ADC_SWRST_MASK;
  AVR32_ADC.chdr = 0xFF;			// Disable all CH
}


void adc_init(unsigned char channel_mask) {
  adc_exit();
  AVR32_ADC.cher = channel_mask;
  AVR32_ADC.mr   = (ADC_SHTIM_VAL<<AVR32_ADC_SHTIM_OFFSET) |
    (ADC_STARTUP_VAL<<AVR32_ADC_STARTUP_OFFSET) |
    (((MASTERCLOCK/ADC_CLOCK+1)/2-1)<<AVR32_ADC_PRESCAL_OFFSET);
}



void adc_sethandler(__int_handler handler, unsigned char channel_mask) {
  AVR32_ADC.idr = channel_mask;
  AVR32_ADC.chsr;
  if (handler != NULL) {
    INTC_register_interrupt(handler, AVR32_ADC_IRQ, AVR32_ADC_INTPRIO);
    AVR32_ADC.ier = channel_mask;
  } //fi valid func
}

