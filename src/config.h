/*
 * config.h
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
 */

#ifndef CONFIG_H_
#define CONFIG_H_


// configuration routines - physical config (MAIN config C0)
typedef struct PACKED_DATA {
  unsigned char flags;
  unsigned char mod_level;
  unsigned short txdelay;	// attention: little-endian here!
} t_config_0;


#define MAX_ALLOWED_TXDELAY	6000	// 6s

// Bitset-Definition of "flags":
#define C0FLAG_RXINVERS		0x01
#define C0FLAG_TXINVERS		0x02
#define C0FLAG_CHAN_B		0x04



// physical config (MAIN config C0) read/write & apply functions:
char		*cfg_read_c0(char *config_buffer);
void		cfg_write_c0(const char *config_data);
void		cfg_apply_c0(void);

// load all configs from EEProm CFG0 block:
void		load_configs_from_eeprom(void);

// test, if a config (0xCx) weas loaded from EEProm:
unsigned char	is_config_loaded(char cfg_id);

unsigned char	config_setup(const char *config_data, int len);
unsigned char	save_configs(void);


#endif // CONFIG_H_
