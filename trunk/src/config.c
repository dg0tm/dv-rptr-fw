/*
 * config.c
 *
 *  Load/Save functions for all configs and esp. C0 block (MAIN config)
 *
 *  Created on: 22.05.2012
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR firmware (RPTRfirmware).
 * For general information about RPTRfirmware see "rptr-main.c".
 *
 * RPTRfirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The RPTRfirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 * Report:
 * 2012-06-30	Config C6 added
 */

#include "config.h"
#include "defines.h"
#include "hw_defs.h"		// PTT-Macro
#include "compiler.h"

#include "dac_func.h"
#include "gmsk_func.h"		// configuration
#include "controls.h"
#include "transceiver.h"
#include "rptr_func.h"		// realtime-handler part of HF I/O

#include "TLV320AIC.h"		// config function analog frontend
#include "dongle.h"
#include "storage.h"		// load/save eeprom data high level functions
#include "crc.h"

#include "stdmodem.h"		// statusCTRL

#include <string.h>


t_config_0 CONFIG_C0 = {
    0x00,			// flags
    750 * 256 / V_Ref,		// modulation voltage peak-peak ~ 0.75V
    GMSK_STDTXDELAY<<8		// little endian! Std-Value 120ms
};

U32		configs_loaded;

extern U32	dgl_capabilities;



// *** configuration ***
/*! \name MAIN configuration routines
 */
//! @{

void cfg_apply_c0(void) {
  U16 txd;
  // Receive Inversion
  gmsk_demodulator_invert(CONFIG_C0.flags & C0FLAG_RXINVERS);
  // Transmitter Inversion
  gmsk_set_mod_hub(((CONFIG_C0.flags & C0FLAG_TXINVERS)?-1:1) * GMSK_DEFAULT_BW);
  // Select Channel A or B from Dual DAC (FSK/AFSK on connector)
  dac_set_active_ch((CONFIG_C0.flags & C0FLAG_CHAN_B)?1:0);
  // adjust modulation level
  set_chA_level(CONFIG_C0.mod_level);
  set_chB_level(CONFIG_C0.mod_level);
  // setup delay before modulation
  txd = swap16(CONFIG_C0.txdelay);
  if (txd > MAX_ALLOWED_TXDELAY) txd = MAX_ALLOWED_TXDELAY;
  gmsk_set_txdelay(txd);
}


char *cfg_read_c0(char *config_buffer) {
  config_buffer[0] = 0xC0;			// identifier byte
  config_buffer[1] = sizeof(CONFIG_C0);		// length
  memcpy(config_buffer + 2, &CONFIG_C0, sizeof(CONFIG_C0));
  return config_buffer + 2 + sizeof(CONFIG_C0);
}


void cfg_write_c0(const char *config_data) {
  memcpy(&CONFIG_C0, config_data, sizeof(CONFIG_C0));
  cfg_apply_c0();
  status_control &= ~STA_NOCONFIG_MASK;		// clear no-config bit
}



bool config_setup(const char *config_data, int len) {
  if (len == 0) return false;
  while (len > 0) {
    U8 block_len = config_data[1];
    switch (config_data[0]) {
    case 0xC0:
      if (block_len==sizeof(CONFIG_C0)) cfg_write_c0(config_data+2); else return false;
      break;
    case 0XC1:	// internal transceiver (QRG)
      trx_standby();
      if (block_len==CONFIG_C1_SIZE) cfg_write_c1(config_data+2); else return false;
      if (status_control & STA_RXENABLE_MASK) trx_receive();	// Enable receiving...
      break;
    case 0XC2:	// Dongle Routing, MyCall, listen-mode-flags
      if (block_len==CONFIG_C2_SIZE) cfg_write_c2(config_data+2); else return false;
      break;
    case 0XC3:	// 20 char CText for dongle transmissions
      if (block_len==CONFIG_C3_SIZE) cfg_write_c3(config_data+2); else return false;
      break;
    case 0XC4:	// AMBE analog frontend configuration 8bytes
      if (block_len==CONFIG_C4_SIZE) cfg_write_c4(config_data+2); else return false;
      break;
    case 0XC5:	// AMBE AGC and DRC config 12bytes
      if (block_len==CONFIG_C5_SIZE) cfg_write_c5(config_data+2); else return false;
      break;
    case 0XC6:	// AMBE NF-Codec Filter-Coeffs (IIR + 5xBQ-Blocks or 1x25tap FIR; 56bytes)
      if (block_len==CONFIG_C6_SIZE) cfg_write_c6(config_data+2); else return false;
      break;
    default:
      // ignore other config blocks
      break;
    } // hctiews
    if ((block_len-2) > len) {	// get next block
      len -= block_len + 2;
      config_data += block_len + 2;
    } else len = 0;
  } // ehliw all blocks
  return true;
}



#define HIGHEST_SUPPORTED_CFG	6

void load_configs_from_eeprom(void) {
  char *cfg_buffer;
  U8   nr;
  configs_loaded = 0x0000;
  if (have_configs()) for (nr=0; nr <= HIGHEST_SUPPORTED_CFG; nr++ ) {
    cfg_buffer = get_loaded_config(nr);
    if (cfg_buffer != NULL) {
      configs_loaded |= (1 << nr);
      config_setup(cfg_buffer, cfg_buffer[1]+2);
    }
  } // fi/rof a CFG0 block exists
}


bool is_config_loaded(char cfg_id) {
  if ((cfg_id & 0xF0) != 0xC0) return false;
  return (configs_loaded & (1<<(cfg_id&0x0F))) != 0;
}



bool save_configs(void) {
  char *cfg_block = get_cfg0_buffer();
  char *nextblock;
  // first build a new (full) CFG0 buffer with all configs:
  nextblock = cfg_read_c0(cfg_block);
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  nextblock = cfg_read_c1(cfg_block);
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  nextblock = cfg_read_c2(cfg_block);
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  nextblock = cfg_read_c3(cfg_block);
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  nextblock = cfg_read_c4(cfg_block);	// analog frontend
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  nextblock = cfg_read_c5(cfg_block);	// AGC+DRC
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  nextblock = cfg_read_c6(cfg_block);	// NF-Codec Filter
  append_crc_ccitt(cfg_block, nextblock-cfg_block+2);
  cfg_block = nextblock + 2;
  // 2nd save new loaded block (only real length)
  return save_cfg0_eeprom(cfg_block - get_cfg0_buffer());
}

//! @}
