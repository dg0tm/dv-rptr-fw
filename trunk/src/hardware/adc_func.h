/*
 * adc_func.h
 *
 * defines basic adc-functions (useing internal ADC of AVR32 µC)
 *
 *  Created on: 10.03.2009
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


#ifndef ADC_FUNJC_H_
#define ADC_FUNJC_H_

#include "intc.h"		// handler def


#define ADC_CLOCK		4000000L	// max. 5MHz allowd for full 10bit resolution

#define ADC_REFERENCEVAL	(512-0)		// später Messung ohne Signal

#define ADC_SIGNEDFS(x)		((x<<6)-0x8000)

// ADC-Channel Definitionen:
#define HFIN			AVR32_ADC.cdr7
#define HFDATA_INT_MASK		AVR32_ADC_IER_EOC7_MASK
#define RSSI_IN                 AVR32_ADC.cdr6
#define RSSI_INT_MASK		AVR32_ADC_IER_EOC6_MASK
#define HFIN_CHANNEL            0x80
#define RSSI_CHANNEL            0x40

#define RSSI_CH_ENABLED()	(AVR32_ADC.chsr&RSSI_CHANNEL)
#define HFIN_CH_ENABLED()	(AVR32_ADC.chsr&HFIN_CHANNEL)


// Timing
#define ADC_SHTIM_VAL		0x03	//= 1�s Sample&Hold Time (min 600ns needed) (value max 0x0F)
#define ADC_STARTUP_VAL		0x1F


void	adc_init(unsigned char channel_mask);
void	adc_exit(void);

#define adc_startconversion()	(AVR32_ADC.cr = AVR32_ADC_START_MASK)

#define adc_int_enable(x)	(AVR32_ADC.ier = x)


// Enable RSSI_CHANNEL, other ad-channels unchanged
#define adc_enable_rssi()	(AVR32_ADC.cher = RSSI_CHANNEL)
// Disable RSSI_CHANNEL, other ad-channels unchanged
#define adc_disable_rssi()	(AVR32_ADC.chdr = RSSI_CHANNEL)

// Enable RSSI_CHANNEL, other ad-channels unchanged
#define adc_enable_hfin()	(AVR32_ADC.cher = HFIN_CHANNEL)
// Disable RSSI_CHANNEL, other ad-channels unchanged
#define adc_disable_hfin()	(AVR32_ADC.chdr = HFIN_CHANNEL)


void	adc_sethandler(__int_handler handler, unsigned char channel_mask);


#endif
