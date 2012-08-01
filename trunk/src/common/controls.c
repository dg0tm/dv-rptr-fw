/*
 * controls.c
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
 *
 * Report:
 * 2012-01-22 JA Bugfix: chA_level() vs. dac_power_mode() using same memory don't work in some cases
 *            -> using new reg_write() operation of twi_func module
 */

#include "controls.h"
#include "twi_func.h"

#include "compiler.h"


typedef struct PACKED_DATA {
  U8 reg;
  U8 value;
} ttwi_control;

ttwi_control chA_lvl;
ttwi_control chB_lvl;


void dac_twidone(tTWIresult TWIres, unsigned int transfered_bytes) {
  if ((TWIres == TWIok)&&(transfered_bytes)) {
    reg_write(TWI_DAC_ADR, 0xE0, 0x00, 1, NULL);
  }
}


void set_dac_power_mode(unsigned char mde) {
  reg_blocking_write(TWI_DAC_ADR, 0xF0, mde, 1, NULL);
}


void set_chA_level(unsigned char level) {
#ifdef DVATRX
  chA_lvl.reg   = TWI_MAX5387_REGB;
  chA_lvl.value = level;
  twi_write(TWI_POTI_MODDEMOD_ADR, (const char *)&chA_lvl.reg, 2, NULL);
#endif
#ifdef DVRPTR
  chA_lvl.reg = 0x40 | (level >> 4);	// 8bits to DAC-Out B (0x10)
  chA_lvl.value = level << 4;
  twi_blocking_write(TWI_DAC_ADR, (const char *)&chA_lvl.reg, 2, dac_twidone);
#endif
}


void set_chB_level(unsigned char level) {
#ifdef DVRPTR
  chB_lvl.reg = 0x50 | (level >> 4);	// 8bits to DAC-Out B (0x10)
  chB_lvl.value = level << 4;
  twi_blocking_write(TWI_DAC_ADR, (const char *)&chB_lvl.reg, 2, dac_twidone);
#endif
}


unsigned char get_chA_level(void) {
#ifdef DVATRX
  return (chA_lvl.value);
#endif
#ifdef DVRPTR
  return (chA_lvl.reg << 4)|(chA_lvl.value>>4);
#endif
}


unsigned char get_chB_level(void) {
#ifdef DVRPTR
  return (chB_lvl.reg << 4)|(chB_lvl.value>>4);
#else
  return 0;
#endif
}

