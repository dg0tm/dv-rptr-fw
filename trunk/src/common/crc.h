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
 */

#ifndef CRC_H_
#define CRC_H_

#define USECCITTTABLE

void append_crc_ccitt(char *buffer, unsigned int len);

unsigned short crc_ccitt(const char *buffer, unsigned int len);


void append_crc_ccitt_revers(char *buffer, unsigned int len);

unsigned short crc_ccitt_revers(const char *buffer, unsigned int len);


#endif // CRC_H_/
