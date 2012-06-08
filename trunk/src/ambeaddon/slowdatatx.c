/*
 * slowdata.c
 *
 * Interface-Function to handle various slow-data formats
 *
 *  Created on: 29.05.2010
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
 * Description:
 * Handle / Interprets embedded slow-data 20 x 3 bytes between 2 FRAME-SYNC in 3 different
 * modes:
 * Mode 1: Text / GPS Icom-like:
 * All incoming 6-byte blocks (two packets odd/even is a logical block) are interpreted.
 * If the first byte upper nibble is a "3" then the rest (lower nibble = len) is "Text" and
 * transfered to the interface (PC).
 * Transmitting like above. Data will be "6-framed" with a "0x3x" header byte.
 *
 * Mode 2: Raw Data Stream:
 * All incoming 3-byte blocks transmitted to the interface. Data fom interface MUST be fit in
 * 6-byte blocks (a rest of 1 to 5 byte will not send out).
 *
 * Mode 3: PCP2 Blocktransfer:
 * A PCP2-frame encapsulate a 1 to 60 byte data-block (+ Message-Type-Byte). Such a Block will
 * be send out after a FRAME-SYNC only. You can transmits up to 4 blocks / PCP2 messages in a burst.
 *
 *
 * Report:
 * 2011-11-27	Add slowdata receive logic
 * 2011-12-03	New slowdata module finished
 *
 */


#include "slowdatatx.h"

#include "defines.h"
#include "dv_dstar.h"

#include "compiler.h"

#include <string.h>


#define ICOMHEADER_MASK		0xF0
#define ICOMHEADER_TEXTID	0x30
#define ICOMHEADER_MSGID	0x40
#define ICOMHEADER_HEADID	0x50
#define ICOMHEADER_DSQID	0xC0
#define ICOM_FILLBYTE		'f'

#define SD_FRAMESTARTID		0xD0
#define SD_DATABLOCKMSG		0xDD


extern tds_header dongle_header;		// Zugriff auf aktuellen HEADER

unsigned char DigitalCode_Value = 0;	// DQS - digital squelch, hold value in BCD form

char	SD_TX_Message[ICOM_CTEXT_SIZE] = 	// current Message for Transmit
    "using DV-RPTR + AMBE";




static char UserSlowData[6];		// holds actual 6 byte of user-data

static const char noUserSlowData[6] = {	// holds actual 6 byte of user-data
  ICOM_FILLBYTE, ICOM_FILLBYTE, ICOM_FILLBYTE,
  ICOM_FILLBYTE, ICOM_FILLBYTE, ICOM_FILLBYTE
};

static char *slowdata_nextblock;	// SlowData Sende-Bytes (auf 6er Block)



// Slowdata transmit function, must be optimized / fast (called in int)

static char *SD_header_ptr;
static char SD_ctext_cnt;

bool UserSD_Set_First(int framecnt) {
  // ToDo Adjustable Re-send-counter for CText
  SD_ctext_cnt  = ((framecnt % (MESSAGE_RESEND_INTERVAL*DSTAR_SYNCINTERVAL)) == 1)?0:127;
  SD_header_ptr = (char *)&dongle_header;	// restart idle transmitting header
  if (DigitalCode_Value > 0) {			// sending DSQ
    UserSlowData[0] = ICOMHEADER_DSQID|2;
    UserSlowData[1] = DigitalCode_Value;
    UserSlowData[2] = DigitalCode_Value;
    memset(UserSlowData+3, ICOM_FILLBYTE, 3);	// sum 6 byte
    slowdata_nextblock = UserSlowData;		// default UD written in this buffer
    return true;
  }
  return false;
}


bool UserSD_Set_CMsg(void) {
  if (SD_ctext_cnt < (ICOM_CTEXT_SIZE/5)) {	// have message-text to send?
    UserSlowData[0] = ICOMHEADER_MSGID|SD_ctext_cnt;
    memcpy(UserSlowData+1, SD_TX_Message+(SD_ctext_cnt*5), 5);
    SD_ctext_cnt++;
    slowdata_nextblock = UserSlowData;		// default UD written in this buffer
    return true;
  } else return false;
}


#define UserSD_Stop_Hdr()		(SD_header_ptr = NULL)

void UserSD_Set_Hdr(void) {
  if (SD_header_ptr != NULL) {
    U32 header_left = (U32)&dongle_header + sizeof(dongle_header) - (U32)SD_header_ptr;
    if (header_left > 0) {
      U32 header_blk  = (header_left > 5)?5:header_left;
      UserSlowData[0] = ICOMHEADER_HEADID|header_blk;
      memcpy(UserSlowData+1, SD_header_ptr, header_blk);
      SD_header_ptr += header_blk;
      if (header_blk < 5) {
	memset(UserSlowData+header_blk+1, ICOM_FILLBYTE, 5-header_blk);
      } // fi REST.
      slowdata_nextblock = UserSlowData;	// default UD written in this buffer
    } else {
      UserSD_Stop_Hdr();
    }
  }
}


// die Default-Sendefunktion: Erzeugt DSQ, CText oder Header...

// ToDo Interleave Text with DStar_SD_IcomXmit() calls...

void DStar_SDTX_Default(unsigned int framecnt) {
  U32 cycle = framecnt % DSTAR_SYNCINTERVAL;
  if (cycle == 1) {	// special handling at the beginning
    if (UserSD_Set_First(framecnt)) return;
  } // fi first cycle
  if (UserSD_Set_CMsg()) {		// Message-Block eingefügt?
    if (cycle > 2) UserSD_Stop_Hdr(); 	// kein Header passt da hinter (20/50 Bytes weg)!
  } else UserSD_Set_Hdr();		// SD-Header-Daten eingefügt?
}



//Sync and Slow-Data functions:

__inline void slowdata_get_sync(unsigned char *databuffer) {
  memset(UserSlowData, ICOM_FILLBYTE, sizeof(UserSlowData));	// Init SD block
  databuffer[0] = 0x55;
  databuffer[1] = 0x2D;
  databuffer[2] = 0x16;
  // reset sd pointer
}


__inline void slowdata_get_txdata(unsigned int framecnt, unsigned char *databuffer) {
  char *slowdataptr;
  U32 cycle = framecnt%DSTAR_SYNCINTERVAL;
  if (cycle & 0x01) {				// Odd-Frames? reload 6 databytes
    slowdata_nextblock = (char *)noUserSlowData; // Default: no Data available
    DStar_SDTX_Default(framecnt);		// Fill UserSlowData or set slowdata_nextblock
    slowdataptr = slowdata_nextblock;
  } else { // fi next Stream-Block
    slowdataptr = slowdata_nextblock+3;
  }
  databuffer[0] = slowdataptr[0] ^ 0x70;
  databuffer[1] = slowdataptr[1] ^ 0x4F;
  databuffer[2] = slowdataptr[2] ^ 0X93;
}


// DigitalCode_Value is in BCD-Coded.
// returns only valid numbers up to value "99"
__inline static char bin2bcd(unsigned char num) {
  return ((num / 10)<<4) | (num % 10);
}

__inline void Set_SlowData_TXMsg(const char *msg) {
  memcpy(SD_TX_Message, msg, sizeof(SD_TX_Message));
}

__inline char *Get_SlowData_TXMsg(void) {
  return SD_TX_Message;
}


void Set_DigitalCodeSquelch(unsigned char number) {
  if (number > 99) number = 0;
  DigitalCode_Value = bin2bcd(number);
}

