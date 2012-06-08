/*
 * eeblockrec.c
 *
 *  Created on: 04.07.2011
 *      Author: Jan Alte, DO1FJN
 *
 * EEProm Block Record provides a simple standard to separate a eeprom memory
 * in up to 7 blocks. Once created, the internal format of each block can be
 * changed w/o any effects to the others. Keep in mind, that block-sizes are
 * fixed wirh the first creation of a EMBR. Keep enougth reserve for growing
 * structures...
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



#include "eeblockrec.h"
#include "eeprom.h"
#include "crc.h"

#include "hardware/hw_defs.h"	// MASTERCLOCK
#include "hardware/gpio_func.h"	// SLEEP() and watchdog()

#include <string.h>

#define READ_EMBR_TIMEOUT	(MASTERCLOCK/50)	// 20ms
#define WRITE_EMBR_TIMEOUT	(MASTERCLOCK/50)	// 20ms



void create_empty_embr(tEMBR *dest, unsigned short storage_size) {
  memset(dest, 0xFF, sizeof(tEMBR));	// clear all
  memcpy(dest->embr_name, EMBRID, 4);	// set own name
  dest->memsize = storage_size;
}


U8 find_free_block(const tEMBR *dest) {
  static const char freeid[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
  U8 idx;
  for (idx=0; idx<7; idx++) {
    if ( memcmp(dest->block[idx].ident, freeid, 4) == 0) {
      return idx;
    }
  }
  return 255;
}


U32 calc_next_ofs(const tEMBR *dest, U8 no_of_used_blocks) {
  U16 highest_ofs = 0;
  U16 highest_len = sizeof(tEMBR);
  U8 idx;
  if (no_of_used_blocks==0)
    return sizeof(tEMBR);
  else for (idx=0; idx<no_of_used_blocks; idx++) {
    if ( (dest->block[idx].length < 0xFFFF) ) {	// have valid data in current block?
      if (highest_ofs < dest->block[idx].offset) {
        highest_ofs = dest->block[idx].offset;
        highest_len = dest->block[idx].length;
      } // fi
    } // fi a valid block
  } // rof all used
  return highest_ofs + highest_len;
}


tEMBRresult add_embr_block(tEMBR *dest, char blockname[4], unsigned short len) {
  U32 memofs;
  U8 block_nr = find_free_block(dest);
  if (block_nr < 7) {
    memofs = calc_next_ofs(dest, block_nr);
    if ( (memofs + len) > dest->memsize ) return EMBRnospaceleft;
    memcpy(dest->block[block_nr].ident, blockname, 4);
    dest->block[block_nr].offset = memofs;
    dest->block[block_nr].length = len;
    return EMBRok;
  } else
    return EMBRnofreeblock;
}



tEMBRitem *get_block(tEMBR *embr, char blockname[4]) {
  U8 idx;
  for (idx=0; idx<7; idx++) {
    if ( memcmp(embr->block[idx].ident, blockname, 4) == 0) {
      return &embr->block[idx];
    }
  }
  return NULL;
}



tEMBRresult read_embr(tEMBR *dest, unsigned int eeprom_addr) {
  U32 starttime, currtime;
  char eerd_id = eeprom_read((char *)dest, sizeof(tEMBR), eeprom_addr);
  starttime = Get_system_register(AVR32_COUNT);
  while (eeprom_status(eerd_id) == eeBUSYREAD) {
    SLEEP();			// wait here until some IRQ evens occurs
    currtime = Get_system_register(AVR32_COUNT);
    if ((currtime-starttime) > READ_EMBR_TIMEOUT) return EMBRtimeout;
  } // ehliw
  switch (eeprom_status(eerd_id)) {
  case eeDONE:		// readin done
    break;
  case eeTIMEOUT:	// no EEProm - device not respond
    return EMBRnoeeprom;
  default:
    return EMBRerror;
  }
  if (memcmp(dest->embr_name, EMBRID, 4) != 0 ) return eeINVALIDID;
  if (crc_ccitt((char *)dest, sizeof(tEMBR)) != 0) return EMBRcrcerr;
  return EMBRok;
}


tEMBRresult write_embr(tEMBR *src, unsigned int eeprom_addr) {
  U32 starttime, currtime;
  char eewr_id;
  append_crc_ccitt((char *)src, sizeof(tEMBR));
  eewr_id   = eeprom_write((char *)src, sizeof(tEMBR), eeprom_addr);
  starttime = Get_system_register(AVR32_COUNT);
  while (eeprom_status(eewr_id) == eeBUSYWRITE) {
    SLEEP();			// wait here until some IRQ evens occurs
    currtime = Get_system_register(AVR32_COUNT);
    if ((currtime-starttime) > WRITE_EMBR_TIMEOUT) return EMBRtimeout;
  } // ehliw
  switch (eeprom_status(eewr_id)) {
  case eeDONE:		// readin done
    break;
  case eeTIMEOUT:	// no EEProm - device not respond
    return EMBRnoeeprom;
  default:
    return EMBRerror;
  }
  return EMBRok;
}
