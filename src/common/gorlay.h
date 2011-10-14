/*
 * gorlay.h
 *
 * Gorlay (23,12,8) and (24,12,8) error-correcting-codes
 * (see http://en.wikipedia.org/wiki/Binary_Golay_code)
 * implementation using ~ 8KiB lookup tables.
 *
 *  Created on: 14.10.2011
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
 */

#ifndef GORLAY_H_
#define GORLAY_H_

// Warning: Use Encode-Function only with valid data (12bits 0..4095)!

unsigned int	gorlay_encode23127(unsigned int data);
unsigned int	gorlay_decode23127(unsigned int code);

unsigned int	gorlay_encode24128(unsigned int data);
unsigned int	gorlay_decode24128(unsigned int code);


#endif // GORLAY_H_

