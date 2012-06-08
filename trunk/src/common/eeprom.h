/*
 * eeprom.h
 *
 *  Created on: 22.04.2011
 *      Author: Jan Alte, DO1FJN
 *
 * This module coordinates read/write jobs on standard TWI(I²C) EEProm's.
 * It's simplified to "one access at time" only (possible is 2 parallel on
 * both devices).
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

#ifndef EEPROM_H_
#define EEPROM_H_

#define EEjobSize	8	// maximum number of unfinished read/write ops.
// (keep an eye of calling a lot ee-read/write functions at the "same" time)

// addresses, sizes (ic model) and page-size (ic model) of TWI-EEProms
// addr-format:	highest nibble = address-pin-configuration (part of TWI device addr)
//		lower 2 bytes  = internal addresses ("smart" addressing only)
#define EE_DV_BASE	0x10000
#define EE_DV_SIZE	8192
#define EE_DV_PAGESIZE	32

typedef enum {
  eeIDLE,  eeBUSYREAD, eeBUSYWRITE, eeDONE,
  eeERROR, eeTIMEOUT,  eeIVALIDJOB, eeINVALIDID
} tEEstatus;


#define EEINVALID		(255)	// a guaranteed invalid queue ID
#define EEPROM_QUEUE_BUSY	(254)	// can't create an new job - no space in array


// eeprom functions:
// returns a ID number of the actual created job or an error-code
// function returns immediately so keep an eye of the operation with "eeprom_status()"
char		eeprom_read(char *dest, unsigned short len, unsigned int addr);
char		eeprom_write(char *src, unsigned short len, unsigned int addr);

// eeprom_status()
// returns status (see enum type) of the operation assigned with ID.
// Invalied ID values: function returns eeINVALIDID.
tEEstatus	eeprom_status(char ID);


#endif // EEPROM_H_
