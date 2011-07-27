/*
 * rptr_func.c
 *
 * realtime controlling openDV (D-Star) operation of DV-RPTR
 *
 *  Created on: 26.07.2011 fomr opendv_func
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

 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 * Report:
 * 2011-01-27  JA  long Zero-Chain at end of transmission (until tx down, 64bits)
 * 2011-07-07  JA  re-struct slow-data-sending system. Only 6-byte block FILL data left here
 */


#include "rptr_func.h"
#include "defs.h"
#include "dv_dstar.h"
#include "gmsk_func.h"

#include "defines.h"

#include "crc.h"
#include "hw_defs.h"
#include "gpio_func.h"

#include "compiler.h"
#include <string.h>


#define DSTAR_SYNCINTERVAL	21

#define DSTAR_SYNC		0x552D1600
#define DSTAR_SYNCMSK		0xFFFFFF00
#define DSTAR_SCRAMDATA		0x0EF2C902



static const unsigned long preamble_dstar[5] = {
  0x55555555, 0x55555555,
  0x55555555, 0x55555555,
  0x37050000
};

static const unsigned long lastframe_dstar[2] = {
  0x55555555,
  0xC87A0000
};
#define DSTAR_LASTFRAMEBITSIZE_TX	64	// keep modulated at END


// AMBE-no Voice Data (just silence)
static const unsigned long SilenceFrame[3] = {
  0x8e4fb8b0,	// ToDo Check!
  0xd55f2ba0,
  0xe8000000
};



// Globale Variablen für D-Star-Transmittions:

unsigned int	RPTR_Flags;
unsigned char	RPTR_RxFrameCount;

tds_header DSTAR_HEADER = {		// Der zu sendende Header
  flags:{0x00, 0x00, 0x00},
  RPT2Call:"        ",
  RPT1Call:"        ",
  YourCall:"CQCQCQ  ",
  MyCall  :"no Call ",
  MyCall2 :"RPTR",
};

static unsigned long	DStar_HeaderBS[DSTAR_HEADERBSBUFSIZE];	// D* Header Bitstream
static unsigned long	DStar_RxHeader[DSTAR_HEADERBSBUFSIZE];	// Received Header Bitstream

#define	TxVoiceBufSize	25
static tds_voicedata	DStar_TxVoice[TxVoiceBufSize];	// 0.5s Voicedata

unsigned int	TxVoice_RdPos, TxVoice_WrPos;


tDV_RXstate	rxstate, last_rxstate;
tDV_TXstate	txstate;

unsigned int	DStar_PacketCounter;
unsigned int	DStar_LostSyncCounter;		// Zähler, wie oft pro Durchgang Sync weg


tds_voicedata	*RxVoiceBuffer;			// Pointer to VoiceBuffer


//Sync and Slow-Data functions:

void dstar_insert_sync(tds_voicedata *dest) {
  dest->data[0] = 0x55;
  dest->data[1] = 0x2D;
  dest->data[2] = 0x16;
}

void dstar_scramble_data(tds_voicedata *dest) {
  dest->data[0] ^=  0x70;
  dest->data[1] ^=  0x4F;
  dest->data[2] ^=  0X93;
}

char *dstar_get_data(char *DataBuffer, const tds_voicedata *srcpkt) {
  *DataBuffer++ = srcpkt->data[0] ^ 0x70;
  *DataBuffer++ = srcpkt->data[1] ^ 0x4F;
  *DataBuffer++ = srcpkt->data[2] ^ 0X93;
  return DataBuffer;
}


// Abfrage, ob Daten-Bytes == FrameSyncDaten
__inline int dstar_syncpaket(const tds_voicedata *rxdata) {
  return ((rxdata->packet[2]&DSTAR_SYNCMSK) == DSTAR_SYNC);
}


__inline void dstar_tx_preamble(void) {
  gmsk_transmit((U32 *)preamble_dstar, DSTAR_PREAMPLELEN, 1);
}


__inline void dstar_tx_stop(void) {
  gmsk_transmit((U32 *)lastframe_dstar, DSTAR_LASTFRAMEBITSIZE_TX, DSTAR_LASTFRAMEBITSIZE);
}




/*! \name DSTAR Handler Functions
 */
//! @{

// Handler-Funktionen
void dstar_stopped(void) {
  gmsk_set_reloadfunc(NULL);
  disable_ptt();
}


void dstar_transmit_stopframe(void) {
  gmsk_set_reloadfunc(&dstar_stopped);
  dstar_tx_stop();
}


void dstar_transmit_voicedata(void) {
  int cycle = DStar_PacketCounter%DSTAR_SYNCINTERVAL;
  if (TxVoice_RdPos == TxVoice_WrPos) {	// no data left, flushed buffer!
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&dstar_transmit_stopframe);
  } else {
    unsigned long *voicedat = DStar_TxVoice[TxVoice_RdPos].packet;
    TxVoice_RdPos = (TxVoice_RdPos+1) % TxVoiceBufSize;
    if (cycle==0) {		// Sync-Daten in SendeBuffer
      dstar_insert_sync(&DStar_TxVoice[TxVoice_RdPos]);
    } else {
      dstar_scramble_data(&DStar_TxVoice[TxVoice_RdPos]);
    } // esle
    if (TxVoice_RdPos == TxVoice_WrPos) {	// last voice frame?
      gmsk_transmit(voicedat, DSTAR_VOICEFRAMEBITSIZE, 1);
      gmsk_set_reloadfunc(&dstar_transmit_stopframe);
    } else {
      gmsk_transmit(voicedat, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    }
    DStar_PacketCounter++;
  }
}


void dstar_transmit_header(void) {
  gmsk_transmit((U32 *)&DStar_HeaderBS, DSTAR_HEADEROUTBITSIZE, DSTAR_HEADEROUTBITSIZE-DSTAR_BEFOREFRAMEENDS);
  DStar_PacketCounter = 0;
  gmsk_set_reloadfunc(&dstar_transmit_voicedata);
}



void dstar_restart_header(void) {
  gmsk_set_reloadfunc(&dstar_transmit_header);	// unmittelbar Header hinter
  gmsk_transmit((U32 *)&preamble_dstar[4], 15, 1);
}

void dstar_begin_new_tx(void) {
  gmsk_set_reloadfunc(&dstar_restart_header);
  dstar_tx_stop();
}

void dstar_break_current(void) {
  if (TxVoice_RdPos == TxVoice_WrPos) {	// last voice frame?
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
  } else {
    gmsk_transmit(DStar_TxVoice[TxVoice_RdPos].packet, DSTAR_VOICEFRAMEBITSIZE, 1);
  }
  gmsk_set_reloadfunc(&dstar_begin_new_tx);
}



// *** Demodulator Handler ***


void dstar_receivedframe(void) {
  RPTR_RxFrameCount++;
  if (RPTR_RxFrameCount < 50) {	// stopps rx if no sync
    gmsk_set_receivebuf(RxVoiceBuffer->packet, DSTAR_FRAMEBITSIZE);
    RPTR_Flags |= RPTR_RX_FRAME;
    rxstate = DVrxvoice;
  } else {				// fi Valid Data
    RPTR_Flags |= RPTR_RX_LOST;
    rxstate = DVsilence;
  }
  if ( (RPTR_RxFrameCount%21)==0 ) {	// Every 21 frame
    if (dstar_syncpaket(RxVoiceBuffer)) {
      RPTR_RxFrameCount = 0;
    } else {
      DStar_LostSyncCounter++;
    }
  } // fi
}


void dstar_receivedframesync(void) {
  gmsk_set_receivebuf(RxVoiceBuffer->packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(&dstar_receivedframe);
  RPTR_RxFrameCount = 0;
  RPTR_Flags |= RPTR_RX_SYNC;
}


void dstar_receivedhdr(void) {
  gmsk_set_receivebuf(RxVoiceBuffer->packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(&dstar_receivedframe);
  RPTR_RxFrameCount = 0;
  DStar_LostSyncCounter = 0;
  RPTR_Flags |= RPTR_RX_HEADER;
  rxstate = DVrxvoice;
}


// Handler to Receive Header in Mem-Buffer
void dstar_getrxheader(void) {
  gmsk_set_receivebuf(DStar_RxHeader, DSTAR_HEADEROUTBITSIZE);
  gmsk_set_receivefkt(&dstar_receivedhdr);
  RPTR_Flags |= RPTR_RX_START;
  rxstate = DVrxheader;
}


// Handler to Receive Header in Mem-Buffer
void dstar_gotstopframe(void) {
  gmsk_set_receivebuf(NULL, 0);
  gmsk_set_receivefkt(NULL);
  RPTR_Flags |= RPTR_RX_STOP;
  rxstate = DVsilence;
}

//! @}

/*! \name DSTAR API Functions
 */
//! @{


// *** API Funktionen ***

void dstar_update_header() {
  append_crc_ccitt_revers((char *)&DSTAR_HEADER, sizeof(tds_header));
  dstar_buildheader(DStar_HeaderBS, &DSTAR_HEADER);
}


void dstar_init_data(tds_voicedata *rxvoicedata) {
  dstar_update_header();
  RxVoiceBuffer = rxvoicedata;
  RPTR_Flags = 0;
}


void dstar_init_hardware(void) {
  gmsk_init();			// Init (De)Modulator Timer
  rxstate      = DVnorx;
  last_rxstate = DVnorx;
}


void dstar_exit_hardware(void) {
  gmsk_exit();
}



void dstar_routeflags(void) {
  char *rr1 = DSTAR_HEADER.RPT1Call;
  int cnt, filled = 0;
  for (cnt=0; cnt<8; cnt++) {
    if ((*rr1 > 0x20) && (*rr1 < 0x80)) {
      filled++;
      break;
    }
    rr1++;
  }
  if (filled)
    DSTAR_HEADER.flags[0] |= FLAG0_USERPT_MASK;
  else
    DSTAR_HEADER.flags[0] &= ~FLAG0_USERPT_MASK;
}


void dstar_update_mycall(const char *MyCallSign) {
  memcpy(DSTAR_HEADER.MyCall, MyCallSign, 12);
  dstar_update_header();
}

void dstar_update_route(const char *NewRoute) {
  memcpy(DSTAR_HEADER.RPT2Call, NewRoute, 24);
  dstar_routeflags();
  dstar_update_header();
}

void dstar_update_dest(const char *NewDest) {
  memcpy(DSTAR_HEADER.RPT2Call, NewDest, 8);
  dstar_update_header();
}

void dstar_update_depart(const char *NewDepart) {
  memcpy(DSTAR_HEADER.RPT1Call, NewDepart, 8);
  dstar_routeflags();
  dstar_update_header();
}

void dstar_update_yourcall(const char *NewYourCall) {
  memcpy(DSTAR_HEADER.YourCall, NewYourCall, 8);
  dstar_update_header();
}

void dstar_set_emergency(void) {
  DSTAR_HEADER.flags[0] |= FLAG0_EMERG_MASK;
  dstar_update_header();
}

void dstar_clr_emergency(void) {
  DSTAR_HEADER.flags[0] &= ~FLAG0_EMERG_MASK;
  dstar_update_header();
}


void dstar_init_header(const tds_header *header) {
  DSTAR_HEADER = *header;
  dstar_update_header();
}


void dstar_receive(void) {
  gmsk_set_synchandler(&dstar_getrxheader, &dstar_gotstopframe, &dstar_receivedframesync);
  gmsk_demodulator_start();
  rxstate = DVsilence;
}


char *dstar_getheader(void) {
  return (char *)DStar_RxHeader;
}


int dstar_channel_idle(void) {
  return (rxstate == DVsilence)&&(gmsk_channel_idle());
}



// später mit update_header (Übergabe)
void dstar_transmit(void) {
  TxVoice_RdPos = 0;
  TxVoice_WrPos = 0;
  if (is_pttactive()) {
    gmsk_set_reloadfunc(&dstar_break_current);		// unmittelbar Header hinter EOT
  } else {
    enable_ptt();
    gmsk_set_reloadfunc(&dstar_transmit_header);	// unmittelbar Header hinter
    dstar_tx_preamble();				// die Preamble setzen
  }
}


void dstar_transmit_data(void) {
  //DVnotx, DVtxdelay, DVtxpreamble, DVtxheader, DVtxvoice, DVtxdata
  /*
  if ((dvstate!=DVtxvoice)&&(dvstate!=DVtxvoicesync)) {	// Wechsel von Sprache
    if (dvstate != DVdisabled) {
      gmsk_stoptimer();
    } // fi
    dvstate = DVtxstartdata;			// Nach dem Header, nur Slow-Data ohne Voice
    gmsk_set_reloadfunc(&dstar_transmit_header);	// unmittelbar Header hinter
    dstar_tx_preamble();				// die Preamble setzen
  } else {	// fi not sprache
    dvstate = DVtxdata;
    gmsk_set_reloadfunc(&dstar_transmit_slowdata);	// unmittelbar Header hinter
  } // esle
//  ambe_getsilence(VoiceBuffer.voice);
  rxstate = DVnorx;
  */
}


void dstar_endtransmit(void) {

}


void dstar_addtxvoice(const tds_voicedata *buf) {
  memcpy(&DStar_TxVoice[TxVoice_WrPos], buf, sizeof(tds_voicedata));
  TxVoice_WrPos = (TxVoice_WrPos+1) % TxVoiceBufSize;
  if (TxVoice_RdPos == TxVoice_WrPos) {	// overflow!!!
    TxVoice_RdPos = (TxVoice_RdPos+1) % TxVoiceBufSize;
  }
}



//! @}
