/*
 * dv_dstar.h
 *
 * Header file for D-STAR
 * Defines header and packet structures of d-star communication.
 * function-prototypes to build/restore header.
 *
 *  Created on: 11.03.2009
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
 *
 */


#ifndef DV_DSTAR_H_
#define DV_DSTAR_H_

#define DSTAR_SYNCINTERVAL	21

#define DSTAR_HEADERBITSIZE	330
#define DSTAR_HEADEROUTSIZE	83

#define DSTAR_VOICEFRAMESIZE	9
#define DSTAR_DATAFRAMESIZE	3

#define DSTAR_HEADERBSBUFSIZE	(DSTAR_HEADEROUTSIZE/4+1)

#define DSTAR_HDR_ITAB_X	28
#define DSTAR_HDR_ITAB_Y	24
#define DSTAR_HDR_IMTX		(DSTAR_HDR_ITAB_X*DSTAR_HDR_ITAB_Y)

// DStar Data Bitsizes
#define DSTAR_HEADEROUTBITSIZE	660
#define DSTAR_VOICEFRAMEBITSIZE	72
#define DSTAR_DATAFRAMEBITSIZE	24
#define DSTAR_LASTFRAMEBITSIZE	(32+15+1)	//48bits in sum
#define DSTAR_FRAMEBITSIZE	96


// BITpattern - this defines are openDV/D-Star related!
#define DSTAR_PATTERN_MASK	0xFFFFFF00	// 24 bits to match patterns in rx-bitstream
#define DSTAR_SYNCPREAMBLE	0xAAAAAA00
#define DSTAR_SYNCSTART		0x0A6EAA00
#define DSTAR_SYNCSTART_LONG	0x0A6EAAAA
#define DSTAR_SYNCSTOP		0xF590AA00	// EOT, is inverted START Pattern
#define DSTAR_FRAMESYNC		0x162D5500	// all >> direction

typedef enum {
  RPTnoinfo=0, RPTunavailable, RPTnoreply, RPTack,
  RPTresend, RPTunused, RPTautoreply, RPTcontrolled
} tRptInfo;	// Lower 3bits of flags[0] is Repeater-Info

#define FLAG0_DATA_MASK		0x80
#define FLAG0_RPT_MASK		0x40
#define FLAG0_BREAK_MASK	0x20
#define FLAG0_CTRL_MASK		0x10	// represents control signal (0 = regular data signal)
#define FLAG0_EMERG_MASK	0x08	// represents an urgent priority signal,


typedef struct __attribute__((__packed__)) {
  unsigned char flags[3];
  char	RPT2Call[8];
  char	RPT1Call[8];
  char	YourCall[8];
  char	MyCall[8];
  char	MyCall2[4];
  unsigned short CRC;
} tds_header;


typedef struct __attribute__((__packed__)) {
  union {
    unsigned int packet[3];
    struct {
      unsigned char voice[9];
      unsigned char data[3];
    };
  };
} tds_voicedata;


typedef char tSlowDataFrame[60];


void	dstar_buildheader(unsigned int *wrbuffer, const tds_header *header);

// Creates a 60byte slow-data stream filled with the radio header (used 4 idle data):
// function is obsolete
//void	dstar_buildheader_sd(char *sddatabuf, const tds_header *header);

// Decoding Header-Stream into header, returns NoOf corr. BitErrors in header
unsigned char dstar_decodeheader(tds_header *header, const char *rdbuffer);

unsigned char dstar_decodeheader_sd(tds_header *header, const char *slowdata);

// Checks the CRC, returns true, if CRC is correct
char	dstar_checkheader(const tds_header *header);


#endif // DV_DSTAR_H_

