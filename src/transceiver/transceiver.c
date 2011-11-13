/*
 * tranceiver.c
 *
 *  Created on: 13.11.2011
 *      Author: Jan Alte, DO1FJN
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

#include "transceiver.h"

#include "rda1846.h"		// RDA1846 based transceiver module

#include "compiler.h"
#include "defines.h"

#include <string.h>


ttrx_function	trx_standby;
ttrx_function	trx_receive;
ttrx_function	trx_transmit;
ttrx_setfreq	trx_setfreq;


void no_function(void) { }


unsigned int set_freq_dummy(unsigned int rxf, unsigned int txf) {
  return 0;
}


unsigned int trx_init(void) {
  U32 result;
  trx_standby  = no_function;
  trx_receive  = no_function;
  trx_transmit = no_function;
  trx_setfreq  = set_freq_dummy;
  // search a transceiver on this board...
  // first: test RDA1846 available...
  result = rda_init();
  if (result & TRXCAP_AVAIL) {
    trx_standby  = rda_standby;
    trx_receive  = rda_receive;
    trx_transmit = rda_transmit;
    trx_setfreq  = rda_setfreq;
    return result;
  }
  // 2nd: test DG8FAC board available...
  // not here!
  return 0;
}



// configuration routines - physical config
typedef struct PACKED_DATA {
  unsigned int	rx_freq;	// !!! little endian here
  unsigned int	tx_freq;	// !!! little endian here
  unsigned char	flags;		// bandwidth...
  unsigned char rsvrd[3];
} t_config_1;

t_config_1 CONFIG_C1;



char *cfg_read_c1(char *config_buffer) {
  config_buffer[0] = 0xC1;			// identifier byte
  config_buffer[1] = sizeof(CONFIG_C1);		// length
  memcpy(config_buffer + 2, &CONFIG_C1, sizeof(CONFIG_C1));
  return config_buffer + 2 + sizeof(CONFIG_C1);
}


void cfg_write_c1(const char *config_data) {
  memcpy(&CONFIG_C1, config_data, sizeof(CONFIG_C1));
  CONFIG_C1.rx_freq = swap32(trx_setfreq(swap32(CONFIG_C1.rx_freq), swap32(CONFIG_C1.tx_freq)));
  // ...
}

