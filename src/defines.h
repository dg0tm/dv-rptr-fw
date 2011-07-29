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

#define FIRMWAREVERSION		0x0030

// Version Vxx.yy (last digit is char ' ', a, b...)

/* What can be read from this version - String?
 * V0.82c
 * Main Version  0 - This Project is in a early developing state
 * Subversion    8 - Suberversion, increased, if new features implemented
 * Subsubversion 2 - Increased if parts of the source are rewritten or optimized
 * Bugfixlevel   c - Increased with one or more bugfixes (at the same time)
 */

#define RELEASE_STRING		"2011-07-29"	// Release Date


#ifndef ASM_DEFINE


#if __GNUC__

#define PACKED_DATA     __attribute__((__packed__))
#define ALIGNED_DATA	__attribute__ ((aligned(4)))

#elif __ICCAVR32__

#define PACKED_DATA
#define ALIGNED_DATA	_Pragma("data_alignment=4")

#endif



typedef void (*tfunction)(void);	// void-void Functions generell

// Serial (RS232) and USB-CDC (serial link) uses frame-based I/O beginning
// with a start-identification
#define FRAMESTARTID	0xD0		// RS232-Paket-Startzeichen
#define PAKETBUFFERSIZE	(512+8)		// Maximum Size per Frame we can receive

// Returncodes (first data in answer)
#define NAK			15
#define ACK			6

#define PKT_PARAM_IDX		4

// below: constuction of a frame:
// 0xD0 <len> <cmd> <data> <crc>
// <len> is a 16bit little endian
typedef struct __attribute__((__packed__)) {
  unsigned char		id;
  unsigned short	len;
  unsigned char		cmd;
} tRS232pktHeader;

typedef struct __attribute__((__packed__)) {
  union {
    tRS232pktHeader	head;
    char		data[PAKETBUFFERSIZE];
  };
} tRS232paket;


#endif


#endif // DEFINES_H_
