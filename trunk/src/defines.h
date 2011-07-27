/*
 * defines.h
 *
 * Defines / typedefs for main-source-file and asm (trampoline.x)
 *
 *  Created on: 19.06.2010
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

#ifndef DEFINES_H_
#define DEFINES_H_

#define PROGRAM_START_OFFSET	0x00004000

#define FIRMWAREVERSION		0x0010

// Version Vxx.yy (last digit is char ' ', a, b...)

/* What can be read from this version - String?
 * V0.82c
 * Main Version  0 - This Project is in a early developing state
 * Subversion    8 - Suberversion, increased, if new features implemented
 * Subsubversion 2 - Increased if parts of the source are rewritten or optimized
 * Bugfixlevel   c - Increased with one or more bugfixes (at the same time)
 */

#define RELEASE_STRING		"2011-07-26"


typedef void (*tfunction)(void);	// void-void Functions generell


typedef enum {
  OpUnknown, RequestReceive, Receiving, RequestTransmit, Transmitting
} tOPmode;


#endif // DEFINES_H_
