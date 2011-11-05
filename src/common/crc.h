/*
 * crc.h
 *
 * checksum algorithms used by D-Star an serial communication
 *
 *  Created on: 11.03.2009
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
 * 2011-11-05	Add "count_no_of_1()" as a AVR32 optimized function
 */

#ifndef CRC_H_
#define CRC_H_

#define USECCITTTABLE

void append_crc_ccitt(char *buffer, unsigned int len);

unsigned short crc_ccitt(const char *buffer, unsigned int len);


void append_crc_ccitt_revers(char *buffer, unsigned int len);

unsigned short crc_ccitt_revers(const char *buffer, unsigned int len);

// count_no_of_1() is not a polynomal crc check. This function counts the number of ones found
// in the pattern (U32) and returns it (0 to 32).
int count_no_of_1(unsigned int pattern);


#endif // CRC_H_/
