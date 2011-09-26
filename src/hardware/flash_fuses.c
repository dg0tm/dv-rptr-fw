/*
 * flash_fuses.c
 *
 *  Created on: 24.09.2011
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


#include "flash_fuses.h"
#include "compiler.h"


#define FLASHKEY_MASK		(AVR32_FLASHC_FCMD_KEY_KEY<<AVR32_FLASHC_FCMD_KEY)

#define	flash_is_busy()		((AVR32_FLASHC.fsr & AVR32_FLASHC_FSR_FRDY_MASK)==0)
#define flash_error()		(AVR32_FLASHC.fsr & (AVR32_FLASHC_FSR_LOCKE_MASK|AVR32_FLASHC_FSR_PROGE_MASK))

static U32	flash_errstatus;


void wait_flash(void) {
  while (flash_is_busy());
}



void flash_cmd(unsigned char cmd, int page_no) {
  wait_flash();
  if (page_no >= 0) {
    AVR32_FLASHC.fcmd = FLASHKEY_MASK | (page_no<<AVR32_FLASHC_FCMD_PAGEN) | (cmd & 0x3F);
  } else {
    AVR32_FLASHC.fcmd = (AVR32_FLASHC.fcmd&AVR32_FLASHC_FCMD_PAGEN_MASK)|FLASHKEY_MASK | (cmd & 0x3F);
  }
  flash_errstatus = flash_error();
}



void erase_fusebit(int gpfusebit) {
  flash_cmd(AVR32_FLASHC_FCMD_CMD_EGPB, gpfusebit & 0x3F);
}


void write_fusebit(int gpfusebit) {
  flash_cmd(AVR32_FLASHC_FCMD_CMD_WGPB, gpfusebit & 0x3F);
}


void write_fusebyte(int gpfusebyteno, unsigned char value) {
  if (gpfusebyteno < 8)
    flash_cmd(AVR32_FLASHC_FCMD_CMD_PGPFB, ((U32)value<<3) | gpfusebyteno);
}


char flash_check4err(void) {
  if (flash_errstatus) {
    AVR32_FLASHC.fcr &= ~AVR32_FLASHC_FCR_FRDY_MASK;
    return FALSE;
  }
  return TRUE;
}

