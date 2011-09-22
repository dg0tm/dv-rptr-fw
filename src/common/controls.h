/*
 * controls.h
 *
 *  Created on: 30.08.2011
 *      Author: Jan Alte, DO1FJN
 *
 *
 * This file is part of the DV-RPTR firmware (DVfirmware).
 * For general information about DVfirmware see "rptr-main.c".
 *
 * DVfirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The DVfirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONTROLS_H_
#define CONTROLS_H_

// TWI (I²C) addresses (see datasheets)

#ifdef DVATRX
#define TWI_POTI_MODDEMOD_ADR	0x28	// A0 Pin Low
#define TWI_POTI_MICRO_ADR	0x29	// A0 Pin high
#define TWI_DAC_ADR		0x58	// Addr 0x38 (L Type) not possible, used in minikey module

#define TWI_MAX5387_REGA	0x11
#define TWI_MAX5387_REGB	0x12
#endif

#ifdef DVRPTR
#if DAC_USE_L_TYPE
#define TWI_DAC_ADR		0x38	// L-Type
#else
#define TWI_DAC_ADR		0x58	// M-Type (default)
#endif
#endif


#define TWI_DAC_POWERUP		0x0C	// Power-Up both channels
#define TWI_DAC_POWERDOWN	0x0F	// Power-Down, terminated outs 100k to GND


void set_dac_power_mode(unsigned char mde);

void set_chA_level(unsigned char level);	// set reference voltage on DAC ChA (FSK)
void set_chB_level(unsigned char level);	// set reference voltage on DAC ChB (AFSK)

unsigned char get_chA_level(void);
unsigned char get_chB_level(void);


#endif // CONTROLS_H_
