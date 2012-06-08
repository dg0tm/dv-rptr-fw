/*
 * storage.c
 *
 *  Created on: 25.04.2012
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
 * Report:
 * 2012-05-18	first working version, w/o call-back on write-done
 *
 * ToDo:
 * - call-back handler for write done (mainfct use it for a ACK-message2PC)
 */

#include "storage.h"
#include "defines.h"
#include "compiler.h"

#include "crc.h"
#include "gpio_func.h"
#include "eeprom.h"
#include "eeblockrec.h"

#include <string.h>

#define CONFIG_EE_SIZE	256

typedef struct {
  unsigned short	adr;
  unsigned char		len;
} tconfig_block;



static tEMBR	embr_int;	// Master Block Record internal EEProm

static char	cfg0_buffer[CONFIG_EE_SIZE];

tconfig_block	cfg0_block[8];	// adresses of every config block

volatile char	cfg0_wid;	// ID for writing to CFG0 buffer


bool eeprom_waitread(char *dest, U16 len, U32 addr) {
  char id;
  id = eeprom_read(dest, len, addr);
  while (eeprom_status(id)==eeBUSYREAD) SLEEP();
  return eeprom_status(id)==eeDONE;
}


bool format_internal_eeprom(void) {
  // use one EMBR for the hole memory only:
  create_empty_embr(&embr_int, EE_DV_SIZE-sizeof(tEMBR));
  add_embr_block(&embr_int, "CFG0", 256);	// 256 Bytes for config data
  if (write_embr(&embr_int, EE_DV_BASE) ==  EMBRok) {
    // store all Configs...
  } else return false;
  return true;
}


void update_cfg0_blocks(void) {
  unsigned int CAdr = 0;
  while (CAdr < sizeof(cfg0_buffer)) {
    // store CAdr
    if ((cfg0_buffer[CAdr] & 0xF8) == 0xC0) {
      cfg0_block[cfg0_buffer[CAdr] & 0x07].adr = CAdr;
      cfg0_block[cfg0_buffer[CAdr] & 0x07].len = cfg0_buffer[CAdr+1]+4;
      CAdr += cfg0_buffer[CAdr+1] + 4;	// Cx LL <data> <crc>
    } else break;
  }
}


bool load_internal_eeprom(void) {
  tEMBRitem *cfg0;
  tEMBRresult res;
  // 1st: reading EMBR on Address 0x0000 of internal EEProm:
  res = read_embr(&embr_int, EE_DV_BASE);
  memset(cfg0_block, 0, sizeof(cfg0_block));
  // 2nd: reading blocks, if EMBR is ok
  if (res == EMBRok) {
    cfg0 = get_block(&embr_int, "CFG0");
    if (cfg0 != NULL) {	// have configs
      if (cfg0->length > CONFIG_EE_SIZE) cfg0->length = CONFIG_EE_SIZE;
      if (eeprom_waitread(cfg0_buffer, cfg0->length, EE_DV_BASE+cfg0->offset)) {
	update_cfg0_blocks();
	return true;
      } // fi EERD
    } // fi have CFG0
  } // fi have EMBR
  return false;
}


bool have_configs(void) {
  return get_block(&embr_int, "CFG0") != NULL;
}


__inline char *get_cfg0_buffer(void) {
  return cfg0_buffer;
}


char *get_loaded_config(unsigned char nr) {
  char *result = NULL;
  if ((nr < 8) && (cfg0_block[nr].len > 0)) {
    if (crc_ccitt(&cfg0_buffer[cfg0_block[nr].adr], cfg0_block[nr].len) == 0)
      result = &cfg0_buffer[cfg0_block[nr].adr];
  }
  return result;
}


bool save_cfg0_eeprom(unsigned int used_length) {
  tEMBRitem *cfg0;
  update_cfg0_blocks();
  cfg0 = get_block(&embr_int, "CFG0");
  if (cfg0 == NULL) {	// no CFG0 block defined...
    // in this case: format eeprom (creates a new EMBR with a CFG0 entry)
    if (format_internal_eeprom())
      cfg0 = get_block(&embr_int, "CFG0");
    if (cfg0 == NULL)
      return false;	// if something goes wrong - return.
  } // fi (now cfg0 is defined)
  if (used_length > cfg0->length) return false;	// no space availabe
  cfg0_wid = eeprom_write(cfg0_buffer, used_length, EE_DV_BASE + cfg0->offset);
  return cfg0_wid != EEPROM_QUEUE_BUSY;
}

