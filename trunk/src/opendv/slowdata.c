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
 *
 */

#include "defines.h"
#include "slowdata.h"
#include "dv_dstar.h"
#include "compiler.h"

#include <string.h>


#define ICOMHEADER_MASK		0xF0
#define ICOMHEADER_TEXTID	0x30
#define ICOMHEADER_MSGID	0x40
#define ICOMHEADER_HEADID	0x50
#define ICOMHEADER_DSQID	0xC0
#define ICOM_FILLBYTE		'f'

extern tds_header DSTAR_HEADER;		// Zugriff auf aktuellen HEADER

tdatatxfunc	DVDataTransmit = NULL;	// -> to Serial/Usb Port (get from rx)
tdatarxfunc	DVDataReceived = NULL;	// <- get no of bytes of pc-data
tdatacopyfunc	DVDataCopyRcvd = NULL;	// <- from Port (to be tx)
tdataflushfunc	DVOldDataFlush = NULL;	//    store unsend data safty in trash
tdatarxfunc	DVNewDataAvailable = NULL; // have new data waiting 4 send out

char	SD_RX_Message[ICOM_CTEXT_SIZE];		// current Message for Receive
U8	SD_RX_Msg_Mask;

char	SD_TX_Message[ICOM_CTEXT_SIZE] = 	// current Message for Transmit
    "DV-RPTR R." RELEASE_STRING
  ;

char	DigitalCode_Value = 0;		// DQS - digital squelch, hold value in BCD form

char	UserSlowData[6];

tSlowDataFrame	UserSlowBlock;

int	PendingBytes;

// DigitalCode_Value is in BCD-Coded.
// returns only valid numbers up to value "99"
__inline char bin2bcd(unsigned char num) {
  return ((num / 10)<<4) | (num % 10);
}


tRXTOmode DStar_RS232_SlowData_Timeout(int len) {
  PendingBytes = len;
  return (rxto_waitend);
}

void DStar_CDC_SlowData_Timeout(int len) {
  PendingBytes = len;
}


int DStar_DataAvailable(void) {
  int rxed_bytes;
  int pending_bytes = PendingBytes;
  PendingBytes = 0;
  if (DVDataReceived != NULL)
    rxed_bytes = DVDataReceived();
  else
    rxed_bytes = 0;
  return ((pending_bytes > 0)||(rxed_bytes > 30))?rxed_bytes:0;
}



void DStar_SD_Default(const char *datatriple, unsigned int framecnt) {
  static char frame_id;
  if (framecnt != 21) {
    if (framecnt & 1) {
      frame_id = datatriple[0];
      switch (frame_id&ICOMHEADER_MASK) {
      case ICOMHEADER_DSQID:
	// Todo: Digital-Squelch Nummer 00..99 auswerten können
	break;
      case ICOMHEADER_MSGID:
	if ((frame_id&0x0F) < (ICOM_CTEXT_SIZE/5)) {
          memcpy(SD_RX_Message+(frame_id&0x0F)*5, datatriple+1, 2);
	}
	break;
      // ICOMHEADER_TEXTID -> Texte, GPS, APRS etc. werden Blockweise aus den SD gefischt
      }
    } else {	// 2nd part of a icom data-block
      if (((frame_id&ICOMHEADER_MASK) == ICOMHEADER_MSGID) && ((frame_id&0x0F) < (ICOM_CTEXT_SIZE/5)) ) {
	memcpy(SD_RX_Message+(frame_id&0x0F)*5+2, datatriple, 3);
	SD_RX_Msg_Mask |= 1 << (frame_id&0x0F);
      }
    }
  }
}


void DStar_SD_RawStream(const char *datatriple, unsigned int framecnt) {
  if ((DVDataTransmit != NULL) && (framecnt != 21))
    DVDataTransmit(datatriple, 3);
  DStar_SD_Default(datatriple, framecnt);
}


void DStar_SD_IcomMsgs(const char *datatriple, unsigned int framecnt) {
  static U8 _2ndlen;
  if ((DVDataTransmit != NULL) && (framecnt != 21)) {
    if (framecnt & 1) {		// odd-numbers 1-3: Begin of a 6-byte-block
      char frame_id = datatriple[0];
      if ( ((frame_id&ICOMHEADER_MASK) == ICOMHEADER_TEXTID) && ((frame_id&0x0F)<6) ) {
	_2ndlen = frame_id&0x0F;
	if (_2ndlen < 3) {
          DVDataTransmit(datatriple+1, _2ndlen);
	  _2ndlen = 0;
	} else {
          DVDataTransmit(datatriple+1, 2);
	  _2ndlen -= 2;
	}
      } else _2ndlen = 0;	// fi Valid block type "30"
    } else {
      if ((_2ndlen > 0) && (_2ndlen < 4)) DVDataTransmit(datatriple, _2ndlen);
    }
  } // fi TR-fct def and not a sync
  DStar_SD_Default(datatriple, framecnt);
}


// Sende-Funktionen - sollten schnell abgearbeitet werden (called in int)

static char *SD_header_ptr;
static char SD_ctext_cnt;


bool UserSD_Set_First(int framecnt) {
    // ToDo Adjustable Re-send-counter for CText
  SD_ctext_cnt  = ((framecnt % (MESSAGE_RESEND_INTERVAL * 21)) == 1)?0:127;
  SD_header_ptr = (char *)&DSTAR_HEADER;	// restart idle transmitting header
  if ((DigitalCode_Value > 0) && (DigitalCode_Value < 0x99)) {	// sending DSQ
    UserSlowData[0] = ICOMHEADER_DSQID|2;
    UserSlowData[1] = DigitalCode_Value;
    UserSlowData[2] = DigitalCode_Value;
    memset(UserSlowData+3, ICOM_FILLBYTE, 3);	// sum 6 byte
    return true;
  }
  return false;
}


bool UserSD_Set_CMsg(void) {
  if (SD_ctext_cnt < (ICOM_CTEXT_SIZE/5)) {	// have message-text to send?
    UserSlowData[0] = ICOMHEADER_MSGID|SD_ctext_cnt;
    memcpy(UserSlowData+1, SD_TX_Message+(SD_ctext_cnt*5), 5);
    SD_ctext_cnt++;
    if (SD_ctext_cnt >= (ICOM_CTEXT_SIZE/5))
      SD_header_ptr = NULL;		// kein Header passt da hinter (20/50 Bytes weg)!
    return true;
  } else return false;
}


bool UserSD_Set_Hdr(void) {
  if (SD_header_ptr != NULL) {
    U32 header_left = (U32)&DSTAR_HEADER + sizeof(DSTAR_HEADER) - (U32)SD_header_ptr;
    if (header_left > 5) header_left = 5;
    // Falls der DSTAR_HEADER mal auf eine durch 5teilbare Größe geändert werden würde,
    // wäre eine leichte Überarbeitung vorteilhaft.
    UserSlowData[0] = ICOMHEADER_HEADID|header_left;
    memcpy(UserSlowData+1, SD_header_ptr, header_left);
    SD_header_ptr += header_left;
    if (header_left < 5) {
      memset(UserSlowData+header_left+1, ICOM_FILLBYTE, 5-header_left);
      SD_header_ptr = NULL;
    } // fi REST.
    return (header_left != 0);
  } else return false;
}


void UserSD_Set_Data(int data_len) {
  if (data_len > 5) data_len = 5;
  UserSlowData[0] = ICOMHEADER_TEXTID|data_len;
  memset(UserSlowData+2, ICOM_FILLBYTE, 4);	// 6 Blocks
  DVDataCopyRcvd(UserSlowData+1, data_len);
}


// die Default-Sendefunktion: Erzeugt DSQ, CText oder Header...

// ToDo Interleave Text with DStar_SD_IcomXmit() calls...

char *DStar_SD_XmitDefault(unsigned int framecnt) {
  U32 cycle = framecnt%21;
  if (cycle == 1) {			// special handling at the beginning
    if (UserSD_Set_First(framecnt)) return UserSlowData;
  } // fi first cycle
  if (UserSD_Set_CMsg()) {		// Message-Block eingefügt?
    return UserSlowData;
  } else if (UserSD_Set_Hdr()) {	// SD-Header-Daten eingefügt?
    return UserSlowData;
  } else return NULL;
}


/* DStar_SD_IcomXmit()
 * überträgt Icom-kompatibel = 0x5x Blöcke mit max 5 Bytes. Gibt es keine USER-Daten,
 * so liefert die Funktion einen NULL pointer zurück. "dstar_transmit_voicedata()" greift
 * dann auf die Daten des TxIdleFrame[] Buffers zurück.
 *
 * Alle Stream-Funktionen werden immer nur zu "ungeraden" FrameCnt aufgerufen und sollten
 * dann einen Zeiger auf die Daten (6 Byte) liefern (oder NULL wenn nix da).
 */
char *DStar_SD_IcomXmit(unsigned int framecnt) {
  int data_len;
  U32 cycle = framecnt%21;
  if (cycle == 1) {			// special handling at the beginning
    if (UserSD_Set_First(framecnt)) return UserSlowData;
  } // fi first cycle
  if (DVDataReceived != NULL) {		// Sind USER-Daten da?
    data_len = DVDataReceived();
  } else {
    data_len = 0;
  }
  if (data_len > 0) {			// es gibt USER Daten
    if ((SD_ctext_cnt < (ICOM_CTEXT_SIZE/5)) && (cycle & 1)) {
      // Daten da, aber auch die Message! Msg bei ungeraden Cycles senden
      UserSD_Set_CMsg();
    } else {				// jetzt mal Daten senden
      UserSD_Set_Data(data_len);
    }
    if (cycle > 1) SD_header_ptr = NULL;	// Header abbrechen.
  } else {				// keine Daten, einfach Idle-Kram
    if (!UserSD_Set_CMsg()) 		// Message-Block eingefügt?
      if (!UserSD_Set_Hdr())		// SD-Header-Daten eingefügt?
        return NULL;			// oh je: Nix eingefügt. NULL -> Fillblock senden
  } // esle
  return UserSlowData;
}


/* DStar_SD_RawXmit()
 * funtioniert wie obige Funktion, setzt jedoch beliebige Daten ein.
 * Achtung: Sind nicht mehr genug Daten da, wird mit dem Füll-Byte 'f' der 6er
 * Block aufgefüllt.
 */
char *DStar_SD_RawXmit(unsigned int framecnt) {
  if (DVDataReceived!=NULL) {
    int len = DVDataReceived();
    if (len > 0) {
      if (len > 6) len = 6;
      memset(UserSlowData+1, ICOM_FILLBYTE, 5);
      DVDataCopyRcvd(UserSlowData, len);
      return UserSlowData;
    }
  }
  return DStar_SD_XmitDefault(framecnt);
}


/* DStar_SD_BlockXmit()
 * funtioniert wie obige Funktion, wartet mit dem "Einsetzen" der Daten jedoch
 * bis zu einem FrameSync. Sind dann min. 60 Byte vorhanden, werden diese in dem
 * Cycle (20 Frames á 20ms) ausgesendet. Mit 'f' aufgefüllte Lücken gibt es so nicht.
 */
char *DStar_SD_BlockXmit(unsigned int framecnt) {
  static char *BlockBufPtr;
  if (DVDataReceived != NULL) {
    if ((framecnt%21) == 1) {
      int len = DVDataReceived();
      if (len >= sizeof(UserSlowBlock)) {
        DVDataCopyRcvd(UserSlowBlock, sizeof(UserSlowBlock));
	BlockBufPtr = UserSlowBlock;
	return BlockBufPtr;
      } else BlockBufPtr = NULL;			// reset block-buffer-pointer
      return DStar_SD_XmitDefault(framecnt);
    } else {
      char *retptr;
      if (BlockBufPtr != NULL) {
	retptr = BlockBufPtr;
	BlockBufPtr += 6;
      } else
	retptr = DStar_SD_XmitDefault(framecnt);
      return retptr;
    } // esle not start
  }
  return DStar_SD_XmitDefault(framecnt);
}



void SlowData_Reset(void) {
  DVDataReceived = NULL;
  DVDataTransmit = NULL;
  DVDataCopyRcvd = NULL;
  DVOldDataFlush = NULL;
  DVNewDataAvailable = NULL;
  PendingBytes = 0;
}


__inline void SlowData_Flush(void) {
  if (DVOldDataFlush != NULL) DVOldDataFlush();
}


void SlowData_Reset_Reception(void) {
  SD_RX_Msg_Mask = 0;
}

__inline char SlowData_CMsg_Complete(void) {
  return (SD_RX_Msg_Mask == 0x0F);
}


__inline char *Get_SlowData_RXMsg(void) {
  SD_RX_Msg_Mask = 0;
  return SD_RX_Message;
}


__inline void Set_SlowData_TXMsg(char *msg) {
  memcpy(SD_TX_Message, msg, sizeof(SD_TX_Message));
}

__inline char *Get_SlowData_TXMsg(void) {
  return SD_TX_Message;
}


void Set_DigitalCodeSquelch(unsigned char number) {
  if (number > 99) number = 0;
  DigitalCode_Value = bin2bcd(number);
}

