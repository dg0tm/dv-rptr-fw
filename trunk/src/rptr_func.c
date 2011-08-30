/*
 * rptr_func.c
 *
 * realtime controlling openDV (D-Star) operation of DV-RPTR
 *
 *  Created on: 26.07.2011 from opendv_func
 *      Author: Jan Alte, DO1FJN
 *
 *
 * This file is part of the DV-RPTR firmware (DVfirmware).
 * For general information about DVfirmware see "rptr-main.c".
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
#include "dv_dstar.h"
#include "gmsk_func.h"
#include "controls.h"

#include "defines.h"

#include "crc.h"
#include "hw_defs.h"
#include "gpio_func.h"
#include "twi_func.h"

#include "compiler.h"
#include <string.h>


#define DSTAR_SYNCINTERVAL	21

#define DSTAR_SYNC		0x552D1600
#define DSTAR_SYNCMSK		0xFFFFFF00
#define DSTAR_SCRAMDATA		0x0EF2C902



ALIGNED_DATA static const unsigned long preamble_dstar[5] = {
  0x55555555, 0x55555555,
  0x55555555, 0x55555555,
  0x37050000
};

ALIGNED_DATA static const unsigned long lastframe_dstar[2] = {
  0x55555555,
  0xC87A0000
};
#define DSTAR_LASTFRAMEBITSIZE_TX	64	// keep modulated at END


// AMBE-no Voice Data (just silence)
ALIGNED_DATA static const unsigned long SilenceFrame[3] = {
  0x8e4fb8b0,	// ToDo Check!
  0xd55f2ba0,
  0xe8ffffff
};



// Globale Variablen für D-Star-Transmittions:

unsigned int	RPTR_Flags;

tds_header DSTAR_HEADER = {		// Der zu sendende Header
  flags:{0x00, 0x00, 0x00},
  RPT2Call:"       B",
  RPT1Call:"       G",
  YourCall:"CQCQCQ  ",
  MyCall  :"no Call ",
  MyCall2 :"RPTR",
};


static unsigned long	DStar_HeaderBS[DSTAR_HEADERBSBUFSIZE];	// Header Bitstream for TX
static unsigned long	DStar_RxHeader[DSTAR_HEADERBSBUFSIZE];	// Received Header Bitstream


#define	VoiceBufSize	DSTAR_SYNCINTERVAL		// Size of VoiceBuf

ALIGNED_DATA static tds_voicedata	DStar_RxVoice[VoiceBufSize];
ALIGNED_DATA static tds_voicedata	DStar_TxVoice[VoiceBufSize];


unsigned char	RPTR_RxFrameCount;	// Count from 0 to 21 or until RPTR_MAX_PKT_WO_SYNC
unsigned int	RPTR_TxFrameCount;	// Count continously

unsigned int	TxVoice_RdPos, TxVoice_WrPos;
unsigned int	DStar_LostSyncCounter;		// Zähler, wie oft pro Durchgang Sync weg



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


// Abfrage, ob Daten-Bytes == FrameSyncDaten
__inline int rptr_syncpaket(const tds_voicedata *rxdata) {
  return ((rxdata->packet[2]&DSTAR_SYNCMSK) == DSTAR_SYNC);
}


__inline void rptr_tx_preamble(void) {
  gmsk_transmit((U32 *)preamble_dstar, DSTAR_PREAMPLELEN, 1);
  LED_Set(LED_RED);
}


__inline void rptr_tx_stop(void) {
  gmsk_transmit((U32 *)lastframe_dstar, DSTAR_LASTFRAMEBITSIZE_TX, DSTAR_LASTFRAMEBITSIZE);
  LED_Clear(LED_RED);
}




/*! \name DSTAR Handler Functions
 */
//! @{

// Handler-Funktionen
void rptr_stopped(void) {
  gmsk_set_reloadfunc(NULL);
  disable_ptt();
}


void rptr_transmit_stopframe(void) {
  gmsk_set_reloadfunc(&rptr_stopped);
  rptr_tx_stop();
  RPTR_clear(RPTR_TX_EARLYPTT); // If PTT active from EarlySTART
}


void rptr_transmit_voicedata(void) {
#if VoiceBufSize!=DSTAR_SYNCINTERVAL
  int cycle = TxVoice_RdPos%DSTAR_SYNCINTERVAL;
#else
#define cycle	TxVoice_RdPos
#endif
  if (TxVoice_RdPos == TxVoice_WrPos) {	// no data left, flushed buffer!
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&rptr_transmit_stopframe);
  } else {
    tds_voicedata *voicedat = &DStar_TxVoice[TxVoice_RdPos];
    // replace voice data, currently transmitting with Silence
    tds_voicedata *voicejusttxed = &DStar_TxVoice[(TxVoice_RdPos+VoiceBufSize-1) % VoiceBufSize];
    TxVoice_RdPos = (TxVoice_RdPos+1) % VoiceBufSize;
    if (cycle==0) {		// Sync-Daten in SendeBuffer
      dstar_insert_sync(voicedat);
#ifdef PLAIN_SLOWDATA
    } else {
      dstar_scramble_data(voicedat);
#endif
    } // esle
    if (TxVoice_RdPos == TxVoice_WrPos) {	// last voice frame?
      gmsk_transmit(voicedat->packet, DSTAR_VOICEFRAMEBITSIZE, 1);
      gmsk_set_reloadfunc(&rptr_transmit_stopframe);
    } else {
      gmsk_transmit(voicedat->packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    }
    // replace voice data, currently transmitting with Silence
    // DSTAR_BEFOREFRAMEENDS < 32: All bits we need for the current tx are in gmsk-buffer
    memcpy(voicejusttxed, SilenceFrame, sizeof(tds_voicedata));
    RPTR_TxFrameCount++;
  }
}


void rptr_transmit_header(void) {
  gmsk_transmit((U32 *)&DStar_HeaderBS, DSTAR_HEADEROUTBITSIZE, DSTAR_HEADEROUTBITSIZE-DSTAR_BEFOREFRAMEENDS);
  RPTR_TxFrameCount = 0;
  gmsk_set_reloadfunc(&rptr_transmit_voicedata);
}



void rptr_restart_header(void) {
  gmsk_set_reloadfunc(&rptr_transmit_header);	// unmittelbar Header hinter
  gmsk_transmit((U32 *)&preamble_dstar[4], 15, 1);
}

void rptr_begin_new_tx(void) {
  gmsk_set_reloadfunc(&rptr_restart_header);
  rptr_tx_stop();
}

void rptr_break_current(void) {
  if (TxVoice_RdPos == TxVoice_WrPos) {	// last voice frame?
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
  } else {
    gmsk_transmit(DStar_TxVoice[TxVoice_RdPos].packet, DSTAR_VOICEFRAMEBITSIZE, 1);
  }
  gmsk_set_reloadfunc(&rptr_begin_new_tx);
}



// *** Demodulator Handler ***


void rptr_receivedframe(void) {
  U8 cycle = RPTR_RxFrameCount%DSTAR_SYNCINTERVAL;
  RPTR_RxFrameCount++;
  if (RPTR_RxFrameCount < RPTR_MAX_PKT_WO_SYNC) {	// stopps rx if no sync
    gmsk_set_receivebuf(DStar_RxVoice[RPTR_RxFrameCount%DSTAR_SYNCINTERVAL].packet, DSTAR_FRAMEBITSIZE);
    RPTR_Flags |= RPTR_RX_FRAME;
  } else {				// fi Valid Data
    RPTR_Flags |= RPTR_RX_LOST;
    LED_Clear(LED_GREEN);
  }
  if ( cycle==0 ) {	// Every 21 frame
    if (rptr_syncpaket(&DStar_RxVoice[cycle])) {
      RPTR_RxFrameCount = 0;
    } else {
      DStar_LostSyncCounter++;
    }
#ifdef PLAIN_SLOWDATA
  } else { // fi
    dstar_scramble_data(&DStar_RxVoice[cycle]);	// Unscramble 3byte slow data
#endif
  }
}


void rptr_receivedframesync(void) {
  //A Voice-Frame with a Sync-Pattern 0101010111010001101000 stored in position [0]
  gmsk_set_receivebuf(DStar_RxVoice[1].packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(&rptr_receivedframe);
  RPTR_RxFrameCount = 1;
  RPTR_Flags |= RPTR_RX_SYNC;
  LED_Set(LED_GREEN);
}


void rptr_receivedhdr(void) {
  gmsk_set_receivebuf(DStar_RxVoice[0].packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(&rptr_receivedframe);
  RPTR_RxFrameCount = 0;
  DStar_LostSyncCounter = 0;
  RPTR_Flags |= RPTR_RX_HEADER;
}


// Handler to Receive Header in Mem-Buffer
void rptr_getrxheader(void) {
  gmsk_set_receivebuf(DStar_RxHeader, DSTAR_HEADEROUTBITSIZE);
  gmsk_set_receivefkt(&rptr_receivedhdr);
  RPTR_Flags |= RPTR_RX_START;
}


// Handler to Receive Header in Mem-Buffer
void rptr_gotstopframe(void) {
  gmsk_set_receivebuf(NULL, 0);
  gmsk_set_receivefkt(NULL);
  RPTR_Flags |= RPTR_RX_STOP;
  LED_Clear(LED_GREEN);
}

//! @}

/*! \name DSTAR API Functions
 */
//! @{


// *** API Funktionen ***

void rptr_update_header() {
  append_crc_ccitt_revers((char *)&DSTAR_HEADER, sizeof(tds_header));
  dstar_buildheader(DStar_HeaderBS, &DSTAR_HEADER);
}


void rptr_init_data(void) {
  U8 cnt;
  rptr_update_header();
  RPTR_Flags = 0;
  for (cnt=0; cnt<VoiceBufSize; cnt++) {
    memcpy(&DStar_TxVoice[cnt], SilenceFrame, sizeof(tds_voicedata));
  }
}


void rptr_init_hardware(void) {
#ifdef DVATRX
  enable_extdemod();			// Test DV-ATRX Target
#endif
  twi_init();
  gmsk_init();				// Init (De)Modulator Timer
}


void rptr_exit_hardware(void) {
  gmsk_exit();
}



void rptr_routeflags(void) {
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


void rptr_update_mycall(const char *MyCallSign) {
  memcpy(DSTAR_HEADER.MyCall, MyCallSign, 12);
  rptr_update_header();
}

void rptr_update_route(const char *NewRoute) {
  memcpy(DSTAR_HEADER.RPT2Call, NewRoute, 24);
  rptr_routeflags();
  rptr_update_header();
}

void rptr_update_dest(const char *NewDest) {
  memcpy(DSTAR_HEADER.RPT2Call, NewDest, 8);
  rptr_update_header();
}

void rptr_update_depart(const char *NewDepart) {
  memcpy(DSTAR_HEADER.RPT1Call, NewDepart, 8);
  rptr_routeflags();
  rptr_update_header();
}

void rptr_update_yourcall(const char *NewYourCall) {
  memcpy(DSTAR_HEADER.YourCall, NewYourCall, 8);
  rptr_update_header();
}

void rptr_set_emergency(void) {
  DSTAR_HEADER.flags[0] |= FLAG0_EMERG_MASK;
  rptr_update_header();
}

void rptr_clr_emergency(void) {
  DSTAR_HEADER.flags[0] &= ~FLAG0_EMERG_MASK;
  rptr_update_header();
}


void rptr_init_header(const tds_header *header) {
  DSTAR_HEADER = *header;
  rptr_update_header();
}


void rptr_receive(void) {
  gmsk_set_synchandler(&rptr_getrxheader, &rptr_gotstopframe, &rptr_receivedframesync);
  gmsk_demodulator_start();
}


char *rptr_getheader(void) {
  return (char *)DStar_RxHeader;
}


unsigned char rptr_copycurrentrxvoice(tds_voicedata *dest) {
  U8 pkt = (RPTR_RxFrameCount+(VoiceBufSize-1))%VoiceBufSize;
  memcpy(dest, DStar_RxVoice[pkt].packet, sizeof(tds_voicedata));
  return pkt;
}


void rptr_copyrxvoice(tds_voicedata *dest, unsigned char nr) {
  memcpy(dest, DStar_RxVoice[nr%VoiceBufSize].packet, sizeof(tds_voicedata));
}




// später mit update_header (Übergabe)
void rptr_transmit(void) {
  TxVoice_RdPos = 0;
  TxVoice_WrPos = 0;
  if (is_pttactive()) {
    if (RPTR_is_set(RPTR_TX_EARLYPTT)) {
      RPTR_clear(RPTR_TX_EARLYPTT);
      gmsk_set_reloadfunc(&rptr_transmit_header); // Header aussenden
    } else {
      gmsk_set_reloadfunc(&rptr_break_current);	// unmittelbar Header hinter EOT
    }
  } else {
    enable_ptt();
    gmsk_set_reloadfunc(&rptr_transmit_header);	// unmittelbar Header hinter
    rptr_tx_preamble();				// die Preamble setzen
  }
}


void rptr_transmit_early_start(void) {
  if ((gmsk_get_txdelay() > 138)&&(!is_pttactive())) {
    RPTR_Flags |= RPTR_TX_EARLYPTT;
    enable_ptt();
    gmsk_set_reloadfunc(&rptr_transmit_stopframe);	// kill TX, if no header adds
    rptr_tx_preamble();
  }
}


void rptr_transmit_data(void) {
  //DVnotx, DVtxdelay, DVtxpreamble, DVtxheader, DVtxvoice, DVtxdata
  /*
  if ((dvstate!=DVtxvoice)&&(dvstate!=DVtxvoicesync)) {	// Wechsel von Sprache
    if (dvstate != DVdisabled) {
      gmsk_stoptimer();
    } // fi
    dvstate = DVtxstartdata;			// Nach dem Header, nur Slow-Data ohne Voice
    gmsk_set_reloadfunc(&rptr_transmit_header);	// unmittelbar Header hinter
    rptr_tx_preamble();				// die Preamble setzen
  } else {	// fi not sprache
    dvstate = DVtxdata;
    gmsk_set_reloadfunc(&rptr_transmit_slowdata);	// unmittelbar Header hinter
  } // esle
//  ambe_getsilence(VoiceBuffer.voice);
  rxstate = DVnorx;
  */
}


void rptr_endtransmit(void) {

}


void rptr_addtxvoice(const tds_voicedata *buf, unsigned char pkt_nr) {
  if (pkt_nr < VoiceBufSize) {	// a numbered packet (00..20)
    // a numbered packet in the empty buffer area results in an update of WrPos
    // (last packet). Gaps are initialized with Silence.
    if ( ((TxVoice_WrPos > TxVoice_RdPos) &&
	 ((pkt_nr > TxVoice_WrPos)||(pkt_nr < TxVoice_RdPos))) ||
	 ((TxVoice_WrPos < TxVoice_RdPos) &&
 	 ((pkt_nr > TxVoice_WrPos)&&(pkt_nr < TxVoice_RdPos))) ) {
      TxVoice_WrPos = (pkt_nr+1) % VoiceBufSize;
    }
  } else {			// a new packet (w/o number)
    pkt_nr = TxVoice_WrPos;
    TxVoice_WrPos = (TxVoice_WrPos+1) % VoiceBufSize;
  }
  if (TxVoice_RdPos == TxVoice_WrPos) {	// overflow!!!
    TxVoice_RdPos = (TxVoice_RdPos+1) % VoiceBufSize;
  }
  memcpy(&DStar_TxVoice[pkt_nr], buf, sizeof(tds_voicedata));
}



//! @}
