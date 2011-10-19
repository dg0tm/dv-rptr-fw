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
 * 2011-09-05  JA  rptr_addtxvoice() nummeration logic bugfix
 * 2011-09-29  JA  silence-frame with slow-data "fff" (means 'unnused' on Icom devices)
 * 2011-10-08  JA  rptr_rx_state variable to get the TX State in a STATUS message.
 * 		   change unused function rptr_routeflags(), correct behavior, if "DIRECT" found in RPT1
 * 2011-10-19  JA  very long gaps on PC voice now filled with silence, PTT goes off after
 * 		   5.04s no new packed arrives from PC or EOT is received.
 *
 * Attention:
 * Prevent sending 1-voice-frame like HEADER - VOICE - EOT. Minimum 2 frames!
 */


#include "rptr_func.h"
#include "dv_dstar.h"
#include "gmsk_func.h"
#include "controls.h"

#include "defines.h"

#include "crc.h"
#include "hw_defs.h"
#include "gpio_func.h"
#include "int_func.h"		// idle_timer()
#include "twi_func.h"

#include "compiler.h"
#include <string.h>


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


// AMBE-no Voice Data (just silence) added with 'fff' on slow data
ALIGNED_DATA static const unsigned long SilenceFrame[3] = {
  0x8e4fb8b0, 0xd55f2ba0, 0xe81629f5
};



// globale variables for D-Star-Transmittions:

unsigned int	RPTR_Flags;
trptr_tx_state 	rptr_tx_state;


tds_header DSTAR_HEADER = {		// actual header for tx in decoded form
  flags:{0x00, 0x00, 0x00},
  RPT2Call:"       B",
  RPT1Call:"       G",
  YourCall:"CQCQCQ  ",
  MyCall  :"no Call ",
  MyCall2 :"RPTR",
};


static unsigned long	DStar_HeaderBS[DSTAR_HEADERBSBUFSIZE];	// Header Bitstream for TX
static unsigned long	DStar_RxHeader[DSTAR_HEADERBSBUFSIZE];	// Received Header Bitstream


ALIGNED_DATA static tds_voicedata	DStar_RxVoice[VoiceRxBufSize];
ALIGNED_DATA static tds_voicedata	DStar_TxVoice[VoiceTxBufSize];


unsigned int	RPTR_RxFrameCount;	// Count continously upwards
unsigned int	RPTR_RxLastSync;	// Keeps the Number of Last-Sync
unsigned int	RPTR_TxFrameCount;	// Count continously

unsigned int	TxVoice_RdPos, TxVoice_WrPos, TxVoice_StopPos;
unsigned int	DStar_LostSyncCounter;		// Zähler, wie oft pro Durchgang Sync weg



//Sync and Slow-Data functions:

__inline void dstar_insert_sync(tds_voicedata *dest) {
  dest->data[0] = 0x55;
  dest->data[1] = 0x2D;
  dest->data[2] = 0x16;
}

__inline void dstar_scramble_data(tds_voicedata *dest) {
  dest->data[0] ^= 0x70;
  dest->data[1] ^= 0x4F;
  dest->data[2] ^= 0X93;
}



__inline void rptr_tx_preamble(void) {
  gmsk_transmit((U32 *)preamble_dstar, DSTAR_PREAMPLELEN, 1);
  RPTR_Flags |= RPTR_TRANSMITTING;
  LED_Set(LED_RED);
}


__inline void rptr_tx_stop(void) {
  gmsk_transmit((U32 *)lastframe_dstar, DSTAR_LASTFRAMEBITSIZE_TX, DSTAR_LASTFRAMEBITSIZE_TX-2);
  rptr_tx_state = RPTRTX_eot;
}




/*! \name RPTR Handler Functions
 */
//! @{

// rptr_stopped() called at the end of a transmission
void rptr_stopped(void) {
  gmsk_set_reloadfunc(NULL);
  disable_ptt();
  RPTR_Flags &= ~RPTR_TRANSMITTING;
  LED_Clear(LED_RED);
#if (DVTX_TIMER_CH==IDLE_TIMER_CH)
  idle_timer_start();
#endif
  rptr_tx_state = RPTRTX_idle;
}

// rptr_transmit_stopframe() append a END-OF-TRANSMISSION id after voice data (no slowdata)
// after transmission, rptr_stopped() is called back from gmsk module.
void rptr_transmit_stopframe(void) {
  rptr_tx_stop();
  gmsk_set_reloadfunc(&rptr_stopped);
  RPTR_clear(RPTR_TX_EARLYPTT);		// If PTT active from EarlySTART
}


void rptr_transmit_voicedata(void) {
/*
  if (TxVoice_RdPos == TxVoice_WrPos) {	// no data left, flushed buffer!
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&rptr_transmit_stopframe);
    rptr_tx_state = RPTRTX_lastframe;
  } else {
*/
    tds_voicedata *voicedat = &DStar_TxVoice[TxVoice_RdPos];
    // replace voice data, currently transmitting with Silence
    tds_voicedata *voicejusttxed = &DStar_TxVoice[(TxVoice_RdPos+VoiceTxBufSize-1) % VoiceTxBufSize];
    TxVoice_RdPos = (TxVoice_RdPos+1) % VoiceTxBufSize;
    if (TxVoice_RdPos == TxVoice_StopPos) {	// EOT-position reached? -> last voice frame
      gmsk_transmit(voicedat->packet, DSTAR_VOICEFRAMEBITSIZE, 1);
      gmsk_set_reloadfunc(&rptr_transmit_stopframe);
      rptr_tx_state = RPTRTX_lastframe;
    } else { // fi stop with this pkt
      gmsk_transmit(voicedat->packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    } // esle norm
    // replace voice data, currently transmitting with Silence
    // DSTAR_BEFOREFRAMEENDS < 32: All bits we need for the current tx are in gmsk-buffer
    memcpy(voicejusttxed, SilenceFrame, sizeof(tds_voicedata));
    RPTR_TxFrameCount++;
    rptr_tx_state = RPTRTX_voicedata;
//  }
}



void rptr_transmit_testloop(void) {	// Looping Transmit RXVoice Buffer
  if (RPTR_is_set(RPTR_TX_TESTLOOP)) {
    tds_voicedata *voicedat = &DStar_RxVoice[TxVoice_RdPos];
    gmsk_transmit(voicedat->packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    TxVoice_RdPos = (TxVoice_RdPos+1) % VoiceRxBufSize;
  } else {
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(rptr_transmit_stopframe);
  }
}



void rptr_transmit_header(void) {
  gmsk_transmit((U32 *)&DStar_HeaderBS, DSTAR_HEADEROUTBITSIZE, DSTAR_HEADEROUTBITSIZE-DSTAR_BEFOREFRAMEENDS);
  RPTR_TxFrameCount = 0;
  if (RPTR_is_set(RPTR_TX_TESTLOOP))
    gmsk_set_reloadfunc(rptr_transmit_testloop);
  else
    gmsk_set_reloadfunc(rptr_transmit_voicedata);
  TxVoice_StopPos = 0;
  TxVoice_RdPos = 0;
  rptr_tx_state = RPTRTX_header;
}



void rptr_restart_header(void) {
  gmsk_transmit((U32 *)&preamble_dstar[4], 15, 1);
  gmsk_set_reloadfunc(rptr_transmit_header);		// unmittelbar Header hinter
  rptr_tx_state = RPTRTX_preamble;
}


void rptr_begin_new_tx(void) {
  rptr_tx_stop();
  gmsk_set_reloadfunc(rptr_restart_header);
}


void rptr_break_current(void) {
  if (TxVoice_RdPos == TxVoice_WrPos) {	// last voice frame?
    gmsk_transmit((U32 *)&SilenceFrame, DSTAR_VOICEFRAMEBITSIZE, 1);
  } else {
    gmsk_transmit(DStar_TxVoice[TxVoice_RdPos].packet, DSTAR_VOICEFRAMEBITSIZE, 1);
  }
  gmsk_set_reloadfunc(rptr_begin_new_tx);
}


#define DSTAR_SYNC		0x00552D16
#define DSTAR_SYNCMSK		0x00FFFFFF

#define RPTR_SYNC_MAXBITERRS	3	// 3 bits of 24 can be failed


// Abfrage, ob Daten-Bytes == FrameSyncDaten
__inline int rptr_is_syncpaket(U8 index) {
  U32 check_pattern;
  check_pattern = (DStar_RxVoice[index].packet[2] & DSTAR_SYNCMSK) ^ DSTAR_SYNC;
  if (check_pattern != 0) {
    U8 bits_zero, err_cnt = 0;
    do {
       bits_zero = __builtin_clz(check_pattern);
       if (bits_zero < 32) {
         check_pattern <<= (bits_zero+1);
         err_cnt++;
       } // fi count
    } while (bits_zero != 32);
    return (err_cnt <= RPTR_SYNC_MAXBITERRS);
  } else {
    return true;
  }
//  return ((DStar_RxVoice[index].packet[2]&DSTAR_SYNCMSK) == DSTAR_SYNC);
}


// *** Demodulator Handler ***

void rptr_receivedframe(void) {
  U8 cycle;
  U8 index;
  // now setup gmsk receiver to catch next frame:
  if ((RPTR_RxFrameCount-RPTR_RxLastSync) < RPTR_MAX_PKT_WO_SYNC) {
    index = (RPTR_RxFrameCount+1) % VoiceRxBufSize;	// position of next frame
    gmsk_set_receivebuf(DStar_RxVoice[index].packet, DSTAR_FRAMEBITSIZE);
  } else {				// fi valid data
    RPTR_Flags |= RPTR_RX_LOST;
    RPTR_Flags &= ~RPTR_RECEIVING;
    LED_Clear(LED_GREEN);
  } // esle
  RPTR_Flags |= RPTR_RX_FRAME;
  cycle = (RPTR_RxFrameCount-RPTR_RxLastSync) % DSTAR_SYNCINTERVAL;
  if (cycle == 0) {					// is a frame-sync expected?
   index = RPTR_RxFrameCount%VoiceRxBufSize;		// index points now to current
   if (rptr_is_syncpaket(index)) {
      RPTR_RxLastSync = RPTR_RxFrameCount;		// update sync pos cnt
    } else {
      DStar_LostSyncCounter++;
    }
  } // fi must be a frame-sync
  RPTR_RxFrameCount++;					// increase counter
}


void rptr_receivedhdr(void) {
  gmsk_set_receivebuf(DStar_RxVoice[0].packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(rptr_receivedframe);
  RPTR_Flags |= RPTR_RX_HEADER;
}


/* rptr_receivedframesync()
 * called every time, if a FRAMESYNC pattern is found in the rx bitstream
 * (after a call of rptr_receivedframe(), if all ok
 */
void rptr_receivedframesync(void) {
  // A Voice-Frame with a Sync-Pattern 0101010111010001101000 was detected
  RPTR_Flags |= RPTR_RX_SYNC;
  if (RPTR_is_set(RPTR_RECEIVING)) {
    if ((RPTR_RxFrameCount-RPTR_RxLastSync) > DSTAR_SYNCINTERVAL) {
      RPTR_RxLastSync = RPTR_RxFrameCount-1;	// update sync pos cnt
    }
  } else {
    RPTR_Flags |= RPTR_RECEIVING;
    // first update receive-buffer to store the voice data, if no header rxed before:
    // this enables receiving (after DSTAR_FRAMEBITSIZE a rptr_receivedframe() occurs)
    gmsk_set_receivebuf(DStar_RxVoice[1].packet, DSTAR_FRAMEBITSIZE);
    gmsk_set_receivefkt(rptr_receivedframe);
    RPTR_RxFrameCount = 1;	// reset counters
    RPTR_RxLastSync   = 0;
    DStar_LostSyncCounter = 0;
    LED_Set(LED_GREEN);
  }
}



// Handler to Receive Header in Mem-Buffer (DStar_RxHeader)
void rptr_getrxheader(void) {
  gmsk_set_receivebuf(DStar_RxHeader, DSTAR_HEADEROUTBITSIZE);
  gmsk_set_receivefkt(rptr_receivedhdr);
  RPTR_RxFrameCount = 0;	// reset counters
  RPTR_RxLastSync   = 0;
  DStar_LostSyncCounter = 0;
  RPTR_Flags |= RPTR_RECEIVING|RPTR_RX_START;	// a new transmission starts
  LED_Set(LED_GREEN);
}


// Handler to Receive Header in Mem-Buffer
void rptr_gotstopframe(void) {
  gmsk_set_receivebuf(NULL, 0);
  gmsk_set_receivefkt(NULL);
  RPTR_Flags |= RPTR_RX_STOP;
  RPTR_Flags &= ~RPTR_RECEIVING;
  LED_Clear(LED_GREEN);
}

//! @}

/*! \name Repeater API Functions
 */
//! @{


// *** API Funktionen ***

void rptr_update_header() {
  append_crc_ccitt_revers((char *)&DSTAR_HEADER, sizeof(tds_header));
  dstar_buildheader(DStar_HeaderBS, &DSTAR_HEADER);
}


void rptr_init_data(void) {
  U8 cnt;
  rptr_tx_state = RPTRTX_disabled;
  rptr_update_header();
  RPTR_Flags = 0;
  for (cnt=0; cnt<VoiceTxBufSize; cnt++) {
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
  int cnt, filled;
  filled = memcmp(rr1, "DIRECT", 6);		// no "DIRECT" text is in RPT1 -> use PTR
  if (filled!=0) for (cnt=0; cnt<8; cnt++) {	// test for valid ASCII-Chars
    if ((*rr1 > 0x20) && (*rr1 < 0x80)) {	// if some text found, use PTR
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
  U8 pkt = (RPTR_RxFrameCount+(VoiceRxBufSize-1))%VoiceRxBufSize;
  memcpy(dest, DStar_RxVoice[pkt].packet, sizeof(tds_voicedata));
  return pkt;
}


void rptr_copyrxvoice(tds_voicedata *dest, unsigned char nr) {
  memcpy(dest, DStar_RxVoice[nr%VoiceRxBufSize].packet, sizeof(tds_voicedata));
}




void rptr_transmit(void) {
#if (DVTX_TIMER_CH==IDLE_TIMER_CH)
  idle_timer_stop();
#endif
  if (is_pttactive()) {
    if (RPTR_is_set(RPTR_TX_EARLYPTT)) {
      RPTR_clear(RPTR_TX_EARLYPTT);
      TxVoice_WrPos = 0;
      gmsk_set_reloadfunc(&rptr_transmit_header); // Header aussenden
      rptr_tx_state = RPTRTX_preamble;
    } else {
      gmsk_set_reloadfunc(&rptr_break_current);	// unmittelbar Header hinter EOT
    }
  } else {
    TxVoice_WrPos = 0;
    enable_ptt();
    gmsk_set_reloadfunc(&rptr_transmit_header);	// unmittelbar Header hinter
    rptr_tx_preamble();				// die Preamble setzen
    rptr_tx_state = RPTRTX_preamble;
  }
}


void rptr_transmit_early_start(void) {
  if ((gmsk_get_txdelay() > 138)&&(!is_pttactive())) {
    RPTR_Flags |= RPTR_TX_EARLYPTT;
    enable_ptt();
    gmsk_set_reloadfunc(&rptr_transmit_stopframe);	// kill TX, if no header adds
    rptr_tx_preamble();
    rptr_tx_state = RPTRTX_txdelay;
  }
}


void rptr_endtransmit(unsigned char pkt_nr_stop) {
  if (pkt_nr_stop < VoiceTxBufSize)
    TxVoice_StopPos = pkt_nr_stop;
  else
    TxVoice_StopPos = (TxVoice_WrPos+1) % VoiceTxBufSize;
}



/* rptr_addtxvoice() writes a voice-frame into transmit buffer
 * keep an eye of buffer overflow's and write to the buffer, who was in tx
 */
void rptr_addtxvoice(const tds_voicedata *buf, unsigned char pkt_nr) {
  tds_voicedata *new_data;

  // Test: un-commend next line to disable buffer-sorting (ignore pkt_nr)
  // pkt_nr = TxVoice_WrPos;	// *** TEST

#if VoiceTxBufSize != DSTAR_SYNCINTERVAL
  int cycle = pkt_nr%DSTAR_SYNCINTERVAL;	// set frame #0, #21, #42 ...
#else						// a sync frame!
#define cycle	pkt_nr
#endif
  if (pkt_nr >= VoiceTxBufSize) return;		// prevent buffer overflow
  new_data = &DStar_TxVoice[pkt_nr];
  memcpy(new_data, buf, sizeof(tds_voicedata));	// copy new data
  if (cycle==0) {				// Sync-Frame needed?
    dstar_insert_sync(new_data);
#ifdef PLAIN_SLOWDATA
//  } else {
//    dstar_scramble_data(new_data);		// scramble data (was plain)
#endif
  } // esle
  if (pkt_nr == TxVoice_WrPos) {		// expected paket from PC...
    TxVoice_WrPos = (TxVoice_WrPos+1) % VoiceTxBufSize;
  } else {		// unsorted packet - expand transmit window to pkt_nr
    // a numbered packet in the empty buffer area results in an update of WrPos
    // (last packet). Gaps are initialized with Silence.
    if ( ((TxVoice_WrPos > TxVoice_RdPos) &&
	 ((pkt_nr >= TxVoice_WrPos)||(pkt_nr < TxVoice_RdPos))) ||
	 ((TxVoice_WrPos < TxVoice_RdPos) &&
 	 ((pkt_nr >= TxVoice_WrPos)&&(pkt_nr < TxVoice_RdPos))) ) {
      TxVoice_WrPos = (pkt_nr+1) % VoiceTxBufSize;
    }
  } // esle with gaps
  TxVoice_StopPos = TxVoice_RdPos;	// update stop-position, long-gap silence
}


__inline unsigned char rptr_get_unsend(void) {
  return (TxVoice_WrPos+VoiceTxBufSize-TxVoice_RdPos) % VoiceTxBufSize;
}



//! @}
