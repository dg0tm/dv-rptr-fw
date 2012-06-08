/*
 * eeblockrec.h
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


#ifndef EEBLOCKREC_H_
#define EEBLOCKREC_H_

#include "defines.h"

#include "compiler.h"

typedef enum {
  EMBRok, EMBRerror, EMBRnoeeprom, EMBRidenterr, EMBRcrcerr, EMBRtimeout,
  EMBRnofreeblock, EMBRnospaceleft
} tEMBRresult;

typedef struct PACKED_DATA {
  char		ident[4];	// a valid ident includes ASCII 32-128 chars only
  U16		offset; 	// empty blocks are filled with 0xFF
  U16		length;
} tEMBRitem;


// EEProm Master Block Record
typedef struct PACKED_DATA {
  char		embr_name[4];	// Name of the EMBR itself - to identify
  tEMBRitem	block[7];	// 56 byte
  U16		memsize;	// no of bytes behind EMBR (i.e. EEProm-Size-sizeof(tEMBR))
  U16		checksum;	// CRC CCITT
} tEMBR;			// sum size = 64 byte


#define EMBRID	"MBR0"		// 'Zero' stands for the first Version

void		create_empty_embr(tEMBR *dest, unsigned short storage_size);
tEMBRresult	add_embr_block(tEMBR *dest, char blockname[4], unsigned short len);

tEMBRitem	*get_block(tEMBR *embr, char blockname[4]);

// Functions to read and store EMBR from/on eeprom memory:
tEMBRresult	read_embr(tEMBR *dest, unsigned int eeprom_addr);
tEMBRresult	write_embr(tEMBR *src, unsigned int eeprom_addr);
// both function needed an initialized TWI hardware. They are blocking and
// returns EMBRok if success.


#endif // EEBLOCKREC_H_
