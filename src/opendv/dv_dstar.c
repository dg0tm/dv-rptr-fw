/*
 * dv_dstar.c
 *
 * implements function to build/restore header.
 *
 *  Created on: 11.03.2009
 *      Author: DO1FJN, Jan Alte
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
 *  Report:
 *  2010-01-16	Header-Funktionen fertig und ausgemistet (Debug-Kram entfernt)
 *
 */

#include "dv_dstar.h"

#include "crc.h"
#include "compiler.h"
#include <string.h>		// using memset()/memcpy()...


#define interleavebitpos24(bp)	{ bp+=24; if (bp>671) bp -= 671; else if (bp>659) bp -= 647; }
#define interleavebitpos28(bp)	{ if (bp>335) { bp+= 27; if (bp>659) bp-=659; } else bp+=28; }



void dstar_buildheader(unsigned int *wrbuffer, const tds_header *header) {
  int hdr_bitcnt, hdr_bitpos, hdr_bytcnt; //, hdr_interbit;
  char *hdr_rptr = (char *)header;	// Pointer to Original Header
  char *hdr_wbuf = (char *)wrbuffer;	// Byte-Pointer to storage buffer
  Union16 hdr_dsr;			// data-shift "register" & crc buffer
  // 1. Clear Bitstream-Buffer
  memset(wrbuffer, 0, DSTAR_HEADERBSBUFSIZE*sizeof(unsigned long));
  // 2. Faltung und Interleave am Stück
  hdr_dsr.u16 = 0;				// Init Faltungsregister (convolution)
  hdr_bitpos  = 0;
  for (hdr_bitcnt=0; hdr_bitcnt < DSTAR_HEADERBITSIZE; hdr_bitcnt++) {
    char hdr_cvsr;
    if ((hdr_bitcnt&0x07)==0) hdr_dsr.u8[0] = *hdr_rptr++;	// Reload Hdr-Byte
    hdr_dsr.u16 >>= 1;			// Shift Right (MSB of u8[1] is "databit in"
    hdr_cvsr = hdr_dsr.u8[1] ^ (hdr_dsr.u8[1]<<2);	// Even Output Bit 7
    // Convolution / Faltung:
    // Setbit Odd:
    hdr_wbuf[hdr_bitpos>>3] |= ((hdr_cvsr ^ (hdr_dsr.u8[1]<<1))&0x80) >> (hdr_bitpos&0x07);
    interleavebitpos28(hdr_bitpos);
    // Setbit Even: "7- mean change Bit oder MSB first"
    hdr_wbuf[hdr_bitpos>>3] |= (hdr_cvsr&0x80) >> (hdr_bitpos&0x07);
    interleavebitpos28(hdr_bitpos);
  }  // rof (330bits processed)
  // 3. scramble data
  hdr_dsr.u16 = 0;				// Init Scramble-Register
  for (hdr_bytcnt=0; hdr_bytcnt < DSTAR_HEADEROUTSIZE; hdr_bytcnt++) {
    // 1. Calc Scramble Byte
    for (hdr_bitcnt=8; hdr_bitcnt>0; hdr_bitcnt--) {
      hdr_dsr.u16 <<= 1;	// Build new Bit:  ScramBit0 ^ Scrambit4 ^ Scrambit7 ^ 1
      hdr_dsr.u8[1] |= ( (hdr_dsr.u8[1]>>4) ^ (hdr_dsr.u8[1]>>7) ^ 1) & 1;
    } // rof build ScramByte
    *hdr_wbuf = bit_reverse8(*hdr_wbuf ^ ~hdr_dsr.u8[1]);	// XOR NOT ScramByte
    hdr_wbuf++;				// next Byte
  } // rof
}


/* obsolet
void dstar_buildheader_sd(char *sddatabuf, const tds_header *header) {
  int cnt, lblockpos;
  char *headerptr = (char *)header;
  for (cnt=0; cnt<(sizeof(tds_header)/5); cnt++) {
    sddatabuf[cnt*6] = 0x55;	// Type 5, Len = 5
    memcpy(sddatabuf+(cnt*6)+1, headerptr, 5);
    headerptr += 5;
  } // rot 8 full 5Byte-Blocks
  lblockpos = (sizeof(tds_header)/5)*6;
  sddatabuf[lblockpos] = 0x50|(sizeof(tds_header)%5);
  memcpy(sddatabuf+lblockpos+1, headerptr, sizeof(tds_header)%5);	// Rest Header
  lblockpos += (sizeof(tds_header)%5) + 1;
  memset(sddatabuf+lblockpos, 0x66, 60-lblockpos);
}
*/

/*
 * Restore a header using traceback method.
 * an good example (c-source) was published by Satoshi Yasuda (7M3TJZ/AD6GZ)
 *
 * This source is a semi-optimized version with minimal memory usage
 *
 */

void restore_header_viterbi(char *hdr_buf, const char *hdr_bitstream) {
  char path_buffer[4][(DSTAR_HEADERBITSIZE+7)>>3];
  char path_metric[4];
  char akt_metric[4];
  int v_bitno;
  int state;
  // 1. init path_buffers (clear) + reset metric-buffer
  memset(path_buffer, 0, sizeof(path_buffer));
  memset(path_metric, 0, sizeof(path_metric));
  // 2. build 4 paths with header-bitstream
  for (v_bitno = 0; v_bitno < DSTAR_HEADERBITSIZE; v_bitno++) { // += 2) {
    char metric0, metric1, metric2, metric3;
    char m1, m2;
    // 2.1. Get 2Bit-couple from bitstream
    char decode_data = (hdr_bitstream[v_bitno>>2] >> (6-((v_bitno&0x03)<<1))) & 0x03;
    // 2.2. Prepare metric-results
    metric0 =  (decode_data>>1)    +  (decode_data&1);		// NORM+NORM
    metric1 = ((decode_data^2)>>1) + ((decode_data^1)&1);	// NEG +NEG
    metric2 = ((decode_data^2)>>1) +  (decode_data&1);		// NEG +NORM
    metric3 =  (decode_data>>1)    + ((decode_data^1)&1);	// NORM+NEG
    // 2.3. walk forward in path-buffers
    m1 = metric0+path_metric[0];	// present state S0 (prev. S0/S2)
    m2 = metric1+path_metric[2];
    akt_metric[0] = m1;
    if (m1 >= m2) {
      akt_metric[0] = m2;
      path_buffer[0][v_bitno>>3] |= 1 << (v_bitno&7);
    } // fi
    m1 = metric1+path_metric[0];	// present state S1 (prev. S0/S2)
    m2 = metric0+path_metric[2];
    akt_metric[1] = m1;
    if (m1 >= m2) {
      akt_metric[1] = m2;
      path_buffer[1][v_bitno>>3] |= 1 << (v_bitno&7);
    }
    m1 = metric2+path_metric[1];	// present state S2 (prev. S1/S3)
    m2 = metric3+path_metric[3];
    akt_metric[2] = m1;
    if (m1 >= m2) {
      akt_metric[2] = m2;
      path_buffer[2][v_bitno>>3] |= 1 << (v_bitno&7);
    }
    m1 = metric3+path_metric[1];	// present state S3 (prev. S1/S3)
    m2 = metric2+path_metric[3];
    akt_metric[3] = m1;
    if (m1 >= m2) {
      akt_metric[3] = m2;
      path_buffer[3][v_bitno>>3] |= 1 << (v_bitno&7);
    }
    memcpy(path_metric, akt_metric, sizeof(path_metric));
  } // rof 330 couples from bitsream
  // 3. clear output-buffer
  memset(hdr_buf, 0, sizeof(tds_header));
  // 4. Trace back path-buffer
  // 4.1. init States (latest 2 Bits)
  state = (path_buffer[0][329>>3] & (1<<(329&0x07)))?2:0;	// Bit1 im Pfad0 gesetzt?
  if (path_buffer[state][328>>3] & (1<<(328&0x07)))
    state = (state>1)?3:2;
  else
    state = (state>1)?1:0;
  // 4.2: 328 Databits left -> put in output-buffer
  for(v_bitno=327; v_bitno>=0; v_bitno--) {	// last 2bit irrel
    hdr_buf[v_bitno>>3] |= (state&0x01)<<(v_bitno&0x07);
    if (path_buffer[state][v_bitno>>3] & (1<<(v_bitno&0x07)) ) // Bit im Pfad gesetzt?
      state = (state>1)?3:2;
    else
      state = (state>1)?1:0;
  } // rof
}



unsigned char dstar_decodeheader(tds_header *header, const char *rdbuffer) {
  char hdr_tmp[DSTAR_HEADEROUTSIZE];
  char hdr_byte, hdr_bitcnt, pos;
  int  bitpos;
  Union16 hdr_dsr;		// data-shift "register" & crc buffer

  memset(hdr_tmp, 0, sizeof(hdr_tmp));		// init temporary header
  hdr_dsr.u16 = 0;				// Init DeScramble-Register
  bitpos = 0;					// Init Interleave Bitpos Cnt
  for (pos=0; pos < DSTAR_HEADEROUTSIZE; pos++) {
    // 1. Calc Scramble Byte
    for (hdr_bitcnt=8; hdr_bitcnt>0; hdr_bitcnt--) {
      hdr_dsr.u16 <<= 1;			// Shift Left (LSB of u8[0] is "databit in"
      // Build new Bit:  ScramBit0 ^ Scrambit4 ^ Scrambit7 ^ 1
      hdr_dsr.u8[1] |= ( (hdr_dsr.u8[1]>>4) ^ (hdr_dsr.u8[1]>>7) ^ 1) & 1;
    } // rof build ScramByte
    // 2. Descamble 8bit of raw header
    hdr_byte =  bit_reverse8(*rdbuffer) ^ ~hdr_dsr.u8[1]; // XOR NOT ScramByte
    rdbuffer++;
    // 3. Spread byte over temp header
    for (hdr_bitcnt=0; hdr_bitcnt<8; hdr_bitcnt++) {
      if (hdr_byte & (0x80 >> hdr_bitcnt))	// Set '1' in hdr_tmp
	hdr_tmp[bitpos>>3] |= 0x80 >> (bitpos&0x07);
      interleavebitpos24(bitpos);
    } // rof every bit
  } // rof
  // Decode Step 4. DeConvolustion (Viterbi method)
  restore_header_viterbi((char *)header, hdr_tmp);
  // Decode Step 5. Rebuild header-stream and check/count hdr_temp bit-errors
  // ToDo:
  // fast verify header, count bit-errors (no Scrample / interleave needed)
  return 0;	// return biterrors
}


unsigned char dstar_decodeheader_sd(tds_header *header, const char *slowdata) {
  const char *blockdata = slowdata;
  const char *dataendpos = slowdata + 60;
  char *hdrdat = (char *)header;
  char id, len, hdrlen=0;
  while ((blockdata<dataendpos) && (hdrlen<41)) {
    id = blockdata[0]>>4;
    if (id==4) len=5; else {
      len = blockdata[0]&0x07;
      if (len==7) {
	if (blockdata[5]=='F') len=3; else len=5; //Correct Biterr.
      } else if (len==6) {
	if (blockdata[3]=='F') len=2; else len=4; //Correct Biterr.
      }
    } // fi not id==4
    if (id==5) {	// ID Header
      if ((hdrlen+len) > sizeof(tds_header)) len = sizeof(tds_header)-hdrlen;
      memcpy(hdrdat, blockdata+1, len);
      hdrdat += len;
      hdrlen += len;
    } // fi id header
    blockdata += 6;	// Es ist immer feste 6-Byte Blöcke
  } // elihw
  return (hdrlen==sizeof(tds_header))?1:0;
}


char dstar_checkheader(const tds_header *header) {
  unsigned short test_crc = swap16(crc_ccitt_revers((char*)header, sizeof(tds_header)-2));
  return (header->CRC==test_crc)?1:0;
}


