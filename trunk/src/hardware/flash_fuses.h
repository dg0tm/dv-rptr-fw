/*
 * flash_fuses.h
 *
 *  Created on: 24.09.2011
 *      Author: DO1FJN, Jan Alte
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

#ifndef FLASH_FUSES_H_
#define FLASH_FUSES_H_

#define ISP_FORCE_FUSE	31

#define SetForceISP()	(erase_fusebit(ISP_FORCE_FUSE))


void erase_fusebit(int gpfusebit);
void write_fusebit(int gpfusebit);
void write_fusebyte(int gpfusebyteno, unsigned char value);
char flash_check4err(void);


#endif // FLASH_FUSES_H_
