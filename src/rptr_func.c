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
 * 2011-10-21  JA  transition from EOT to a new header is now w/o DC - use SYNC pattern instead
 * 2011-10-32  JA  handle all PATTERN checks in this module
 * 2011-11-04  JA  rptr_get_unsend() returns false values in some cases - fixed
 * 2011-11-04  JA  bugfix in  -> function now w/o dead-lock
 * 2011-04-05  JA  add receive-unlock function -> this terminate receiving, if RX-PLL unlock
 *		   moved one-counter part of rptr_is_syncpacket() as a separate function to "crc.c"
 *		   add a "lossy" START-detection: tolerate up to 2 bit-errors, if a 17bit SYNC was before
 * 2011-12-28  JA  long-roger-beep bug fixed
 * 2012-01-06  JA  keep BufferWritePos to actual position (simplified fifo)
 * 2012-01-11  JA  BugFix Pattern-Handling if PLL not correct an a to early match appears.
 * 2012-02-01  JA  Restoring FRAME-SYNC while transmitting silence frames
 *                 Replacement-Header: MyCall/Sign defined in rptr_func.h
 *
 * ToDo:
 * - Bug WrPos
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

#include "transceiver.h"

#include "compiler.h"
#include <string.h>


#define RPTR_PREAMBLE_TO	DSTAR_SYNCINTERVAL	// value in 20ms

#define VOICE_TX_ALLOWED_GAP	(VoiceTxBufSize/3)

#define VOICE_TX_DIFFERENCE(a,b)	((a+VoiceTxBufSize-b) % VoiceTxBufSize)
//#define VOICE_TX_DIFFERENCE(a,b)	((a-b) % VoiceTxBufSize)


ALIGNED_DATA static const unsigned long preamble_dstar[5] = {
  0x55555555, 0x55555555,
  0x55555555, 0x55555555,
  0x37050000
};
#define DSTAR_PREAMPLELEN	(2*64+15)

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
  RPT2Call:"DIRECT B",
  RPT1Call:"DIRECT G",
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


void rptr_clear_tx_buffer(void) {
  int cnt;
  for (cnt=0; cnt<VoiceTxBufSize; cnt++) {
    memcpy(&DStar_TxVoice[cnt], SilenceFrame, sizeof(tds_voicedata));
    if ((cnt%DSTAR_SYNCINTERVAL)==0)
      dstar_insert_sync(&DStar_TxVoice[cnt]);
  } // rof all TX-buffers
}



/*! \name RPTR Handler Functions
 */
//! @{

// rptr_stopped() called at the end of a transmission
void rptr_stopped(void) {
  gmsk_set_reloadfunc(NULL);
  disable_ptt();
  trx_receive();
  RPTR_clear(RPTR_TRANSMITTING);
  LED_Clear(LED_RED);
#if (DVTX_TIMER_CH==IDLE_TIMER_CH)
  idle_timer_start();
#endif
  rptr_tx_state = RPTRTX_idle;
}

// rptr_transmit_stopframe() append a END-OF-TRANSMISSION id after voice data (no slowdata)
// after transmission, rptr_stopped() is called back from gmsk module.
void rptr_transmit_stopframe(void) {
  gmsk_transmit((U32 *)lastframe_dstar, DSTAR_LASTFRAMEBITSIZE_TX, DSTAR_LASTFRAMEBITSIZE_TX-2);
  gmsk_set_reloadfunc(&rptr_stopped);
  rptr_tx_state = RPTRTX_eot;
}


void rptr_transmit_voicedata(void) {
  int last_cycle = (TxVoice_RdPos+DSTAR_SYNCINTERVAL-1) % DSTAR_SYNCINTERVAL;
  tds_voicedata *voicedat = &DStar_TxVoice[TxVoice_RdPos];
  // replace voice data, currently transmitting with Silence
  tds_voicedata *voicejusttxed = &DStar_TxVoice[VOICE_TX_DIFFERENCE(TxVoice_RdPos,1)];
  if (TxVoice_RdPos == TxVoice_WrPos) {		// Buffer is STILL empty
    RPTR_set(RPTR_TX_EMPTY);			// set flag to signalling
    TxVoice_WrPos = (TxVoice_WrPos+1) % VoiceTxBufSize;	// increment WritePos
  } // fi empty
  TxVoice_RdPos = (TxVoice_RdPos+1) % VoiceTxBufSize;
  if (TxVoice_RdPos == TxVoice_StopPos) {	// EOT-position reached? -> last voice frame
    gmsk_transmit(voicedat->packet, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&rptr_transmit_stopframe);
    TxVoice_WrPos = TxVoice_RdPos;		// show zero "unsend" frames
    rptr_tx_state = RPTRTX_lastframe;
  } else { // fi stop with this pkt
    gmsk_transmit(voicedat->packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    rptr_tx_state = RPTRTX_voicedata;
    if (TxVoice_RdPos == TxVoice_WrPos) {	// Buffer get empty NOW!
      RPTR_set(RPTR_TX_EMPTY);			// signalling it
    } // fi empty
  } // esle norm
  // replace voice data, currently transmitting with Silence
  // DSTAR_BEFOREFRAMEENDS < 32: All bits we need for the current tx are in gmsk-buffer
  memcpy(voicejusttxed, SilenceFrame, sizeof(tds_voicedata));
  if (last_cycle==0) {	// insert 55 2D 16 pattern every 21frame (silence tx only)
    dstar_insert_sync(voicejusttxed);
  } // fi restore sync
  RPTR_TxFrameCount++;
}


void rptr_transmit_header(void) {
  gmsk_transmit((U32 *)&DStar_HeaderBS, DSTAR_HEADEROUTBITSIZE, DSTAR_HEADEROUTBITSIZE-DSTAR_BEFOREFRAMEENDS);
  RPTR_TxFrameCount = 0;
  TxVoice_RdPos     = 0;
  gmsk_set_reloadfunc(rptr_transmit_voicedata);
  rptr_tx_state = RPTRTX_header;
}


void rptr_restart_header(void) {
  gmsk_transmit((U32 *)&preamble_dstar[3], 32+15, 1);
  gmsk_set_reloadfunc(rptr_transmit_header);		// unmittelbar Header hinter
  rptr_tx_state = RPTRTX_preamble;
}


void rptr_begin_new_tx(void) {
  gmsk_set_reloadfunc(rptr_restart_header);
  gmsk_transmit((U32 *)lastframe_dstar, DSTAR_LASTFRAMEBITSIZE, 1);
  rptr_clear_tx_buffer();
}


void rptr_break_current(void) {
  gmsk_set_reloadfunc(rptr_begin_new_tx);
  gmsk_transmit(DStar_TxVoice[TxVoice_RdPos].packet, DSTAR_VOICEFRAMEBITSIZE, 1);
  // clear last txed voice, like in rptr_transmit_voicedata()
  memcpy(&DStar_TxVoice[VOICE_TX_DIFFERENCE(TxVoice_RdPos,1)], SilenceFrame, sizeof(tds_voicedata));
  rptr_tx_state = RPTRTX_lastframe;
}



void rptr_preamble(void) {
  if (TxVoice_RdPos < (3*RPTR_PREAMBLE_TO)) {	// max 420ms before shutdown
    gmsk_transmit((U32 *)preamble_dstar, 32, 16);
    TxVoice_RdPos++;
    rptr_tx_state = RPTRTX_preamble;
  } else {
    rptr_stopped();
  }
}


void rptr_transmit_start(void) {
  gmsk_set_reloadfunc(rptr_transmit_header);
  if (TxVoice_RdPos > 4)
    TxVoice_RdPos = 4;
  // transmitting a minimum of 128bits preamble plus START pattern (15bits)
  gmsk_transmit((U32 *)&preamble_dstar[TxVoice_RdPos], DSTAR_PREAMPLELEN-(TxVoice_RdPos*32), 1);
  rptr_tx_state = RPTRTX_preamble;
}




// *** receiving handler functions ***


#define DSTAR_SYNC		0x00552D16
#define DSTAR_SYNCMSK		0x00FFFFFF

#define RPTR_SYNC_MAXBITERRS	3	// 3 bits of 24 can be failed


// Abfrage, ob Daten-Bytes == FrameSyncDaten
__inline int rptr_is_syncpaket(U8 index) {
  return (count_no_of_1((DStar_RxVoice[index].packet[2] & DSTAR_SYNCMSK) ^ DSTAR_SYNC) <= RPTR_SYNC_MAXBITERRS);
}


// *** Demodulator Handler ***

int rptr_waitpattern_START(unsigned int pattern, unsigned int bitcounter);	// Forward
int rptr_whilereceivepattern(unsigned int pattern, unsigned int bitcounter);	// Forward


void rptr_receivedframe(void) {
  U8 cycle;
  U8 index;
  // now setup gmsk receiver to catch next frame:
  if (RPTR_is_set(RPTR_RX_PREAMBLE) ||
    ((RPTR_RxFrameCount-RPTR_RxLastSync) >= RPTR_MAX_PKT_WO_SYNC) ) {
    gmsk_set_patternfunc(rptr_waitpattern_START);
    RPTR_clear(RPTR_RECEIVING|RPTR_RX_PREAMBLE);
    RPTR_set(RPTR_RX_LOST);
    LED_Clear(LED_GREEN);
  } else {				// fi valid data
    index = (RPTR_RxFrameCount+1) % VoiceRxBufSize;	// position of next frame
    gmsk_set_receivebuf(DStar_RxVoice[index].packet, DSTAR_FRAMEBITSIZE);
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


#ifdef DEBUG_PREAMBLE
#define DBG_PREABLE_START()	debugpin_set(DEBUG_PIN2)
#define DBG_PREABLE_STOP()	debugpin_clear(DEBUG_PIN2)
#else
#define DBG_PREABLE_START()
#define DBG_PREABLE_STOP()
#endif


// start-functions, called if a start-receiving condition (pattern) detects.
void rptr_startrx_now(void) {
  gmsk_set_receivebuf(DStar_RxHeader, DSTAR_HEADEROUTBITSIZE);
  gmsk_set_receivefkt(rptr_receivedhdr);
  gmsk_set_patternfunc(rptr_whilereceivepattern);
  RPTR_RxFrameCount = 0;	// reset counters
  RPTR_RxLastSync   = 0;
  DStar_LostSyncCounter = 0;
  RPTR_clear(RPTR_RX_PREAMBLE);
  RPTR_set(RPTR_RECEIVING|RPTR_RX_START);	// a new transmission starts
  LED_Set(LED_GREEN);
  DBG_PREABLE_STOP();
}


void rptr_syncrx_now(void) {
  // first update receive-buffer to store the voice data, if no header rxed before:
  // this enables receiving (after DSTAR_FRAMEBITSIZE a rptr_receivedframe() occurs)
  gmsk_set_receivebuf(DStar_RxVoice[1].packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(rptr_receivedframe);
  gmsk_set_patternfunc(rptr_whilereceivepattern);
  RPTR_RxFrameCount = 1;	// reset counters
  RPTR_RxLastSync   = 0;
  DStar_LostSyncCounter = 0;
  RPTR_clear(RPTR_RX_PREAMBLE);
  RPTR_set(RPTR_RECEIVING|RPTR_RX_FRAMESYNC);
  LED_Set(LED_GREEN);
  DBG_PREABLE_STOP();
}


// Handler to Receive Header in Mem-Buffer (DStar_RxHeader)
int rptr_waitpattern_START(unsigned int pattern, unsigned int bitcounter) {
  switch (pattern&DSTAR_PATTERN_MASK) {
  case DSTAR_SYNCPREAMBLE:
    if (!RPTR_is_set(RPTR_RX_PREAMBLE)) {
      RPTR_set(RPTR_RX_PREAMBLE|RPTR_RX_1STPREAMBLE);
      gmsk_set_receivebuf(NULL, 0);
      DBG_PREABLE_START();
      return 2;			// force sync now
    }
    break;
  case ((~DSTAR_SYNCPREAMBLE)&0xFFFFFF00):	// inverted preamble pattern - do nothing
    break;
  case DSTAR_SYNCSTART:		// A START-PATTERN was found
    rptr_startrx_now();
    return 1;			// request sync, restart rxbitcouter
  case DSTAR_FRAMESYNC:		// A FRAME-SYNC-PATTERN was found (no earlier START detected)
    rptr_syncrx_now();
    return 1;
  default:
    if (RPTR_is_set(RPTR_RX_PREAMBLE) && (bitcounter>RPTR_MAXLEN_RXPREAMBLE)) {
      RPTR_clear(RPTR_RX_PREAMBLE);
      DBG_PREABLE_STOP();
    } // fi
    break;
  } // hctiws
  // now check "lossy" start condition
  if ((pattern & 0x0001FFFF) == 0x0000AAAA) {	// oldest Bits in sync state...
    if (count_no_of_1((pattern&0xFFFE0000)^(DSTAR_SYNCSTART&0xFFFE0000)) < 3) {
      rptr_startrx_now();
      return 1;			// request sync, restart rxbitcouter
    }
  } // fi old16 sync
  return 0;		// do nothing
}



/* rptr_whilereceivepattern()
 * called every received bit. If a FRAMESYNC- or a EOT-pattern is found in the rx bitstream
 * force a re-sync or stop receiving
 */
int rptr_whilereceivepattern(unsigned int pattern, unsigned int bitcounter) {
  switch (pattern&DSTAR_PATTERN_MASK) {
  case DSTAR_SYNCPREAMBLE:	// a 48bit long EOT condition (24 + 24 bit) should follow
    if (bitcounter < 9) {	// syncpreamble pattern on correct position?
      RPTR_set(RPTR_RX_PREAMBLE);
    } // fi wait for EOT
    return 0;
  case DSTAR_FRAMESYNC:	// A Voice-Frame with a Sync-Pattern 0101010111010001101000 was detected
    RPTR_set(RPTR_RX_FRAMESYNC);
    if (bitcounter != 0) {	// on a bitcounter of Zero a correct frame was received
      // otherwise, we must shift it a few bits to re-sync
      if (bitcounter < 5) {	// PLL to slow, FRAME-SYNC to late
        RPTR_RxLastSync = RPTR_RxFrameCount-1;	// update sync pos cnt
        return 2;
      } else if (bitcounter > (DSTAR_FRAMEBITSIZE-5)) {	// PLL to fast, early FRAME-SYNC
        RPTR_RxLastSync = RPTR_RxFrameCount;	// update sync pos cnt
        DStar_RxVoice[RPTR_RxFrameCount%VoiceRxBufSize].packet[2] = swap32(pattern);
        rptr_receivedframe();	// call receive function to handle (incorrect) unfinished data
        return 2;		// force a re-sync
      }
      // ignore FRAME-SYNC pattern far away expected positions
    } // fi found in the near of a regular FRAME-SYNC
    break;
  case DSTAR_SYNCSTOP:
    if (bitcounter < (24+5) ) {
      RPTR_clear(RPTR_RECEIVING|RPTR_RX_PREAMBLE);
      gmsk_set_patternfunc(rptr_waitpattern_START);
      RPTR_set(RPTR_RX_STOP);
      LED_Clear(LED_GREEN);
      return -1;	// a VALID STOP receives on BITCOUNTER Pos 24. Stop Receiving
    }
    break;
  }
  return 0;
}


void rptr_receiveunlock(void) {
  if (RPTR_is_set(RPTR_RECEIVING)) {
    RPTR_clear(RPTR_RECEIVING|RPTR_RX_PREAMBLE);
    gmsk_set_patternfunc(rptr_waitpattern_START);
    RPTR_set(RPTR_RX_LOST);
    LED_Clear(LED_GREEN);
  }
}


//! @}


/*! \name Repeater API Functions
 */
//! @{


// *** API functions ***

void rptr_update_header() {
  append_crc_ccitt_revers((char *)&DSTAR_HEADER, sizeof(tds_header));
  dstar_buildheader(DStar_HeaderBS, &DSTAR_HEADER);
}


void rptr_init_data(void) {
  rptr_tx_state = RPTRTX_disabled;
  rptr_update_header();
  RPTR_Flags = 0;
  rptr_clear_tx_buffer();
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
  if (filled!=0) {
    filled = 0;
    for (cnt=0; cnt<8; cnt++) {			// test for valid ASCII-Chars
      if ((*rr1 > 0x20) && (*rr1 < 0x80)) {	// if some text found, use PTR
	filled++;
	break;
      }
      rr1++;
    }
  }
  if (filled)
    DSTAR_HEADER.flags[0] |= FLAG0_USERPT_MASK;
  else {
    DSTAR_HEADER.flags[0] &= ~FLAG0_USERPT_MASK;
    memcpy(DSTAR_HEADER.RPT2Call, "DIRECT  DIRECT  ", 16);
  }
}


void rptr_replacement_header(void) {
  memcpy(DSTAR_HEADER.YourCall, "CQCQCQ  ", 8);
  memcpy(DSTAR_HEADER.MyCall,  REPLACEMENT_HDR_CALL, 8);
  memcpy(DSTAR_HEADER.MyCall2, REPLACEMENT_HDR_SIGN, 4);
  DSTAR_HEADER.flags[0] = FLAG0_RPT_MASK;
  rptr_update_header();
}


void rptr_init_header(const tds_header *header) {
  DSTAR_HEADER = *header;
  rptr_update_header();
}


void rptr_receive(void) {
  gmsk_demodulator_start();
  gmsk_set_patternfunc(rptr_waitpattern_START);	// call after "START" otherwise func will be resetted
  gmsk_set_unlockedfunc(rptr_receiveunlock);
}


__inline char *rptr_getheader(void) {
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


__inline void rptr_forcefirstsync(void) {
  RPTR_RxLastSync = -RPTR_MAX_PKT_WO_SYNC;
}


void rptr_transmit_preamble(void) {
  if (!is_pttactive()) {
    enable_ptt();
    trx_transmit();
    TxVoice_RdPos = 0;				// counts timeout of preamble loop
    gmsk_set_reloadfunc(rptr_preamble);		// transmit preamble, until rptr_transmit() or timeout
    gmsk_transmit((U32 *)preamble_dstar, 32, 1);
    RPTR_Flags |= RPTR_TRANSMITTING;
    LED_Set(LED_RED);
    rptr_tx_state = RPTRTX_txdelay;
  } // fi transmitter OFF
}


void rptr_transmit(void) {
#if (DVTX_TIMER_CH==IDLE_TIMER_CH)
  idle_timer_stop();
#endif
  if (is_pttactive()) {				// DV-RPTR is transmitting, restart it
    if (rptr_tx_state == RPTRTX_voicedata) {
      gmsk_set_reloadfunc(rptr_break_current);	// unmittelbar Header hinter EOT
    } else if (rptr_tx_state == RPTRTX_lastframe) {
      gmsk_set_reloadfunc(rptr_begin_new_tx);
    } else {
      gmsk_set_reloadfunc(rptr_transmit_start);	// change from preamble to header
    } // esle fi
  } else {
    enable_ptt();
    trx_transmit();
    gmsk_set_reloadfunc(rptr_transmit_header);	// after TXed peamble + START-pattern, load header
    gmsk_transmit((U32 *)preamble_dstar, DSTAR_PREAMPLELEN, 1);
    RPTR_Flags |= RPTR_TRANSMITTING;
    LED_Set(LED_RED);
    rptr_tx_state = RPTRTX_preamble;
    rptr_clear_tx_buffer();
  }
  TxVoice_WrPos = 0;
}


void rptr_endtransmit(unsigned char pkt_nr_stop) {
  if (RPTR_is_set(RPTR_TX_EMPTY)) {
    TxVoice_StopPos = (TxVoice_RdPos+1) % VoiceTxBufSize;
    // if the rptr_transmit_voicedata() occurred (int), set StopPos new:
    if (TxVoice_StopPos == TxVoice_RdPos)
      TxVoice_StopPos = (TxVoice_RdPos+1) % VoiceTxBufSize;
  } else if (pkt_nr_stop < VoiceTxBufSize)
    TxVoice_StopPos = pkt_nr_stop;
  else
    TxVoice_StopPos = TxVoice_WrPos;	//+1) % VoiceTxBufSize;
}



/* rptr_addtxvoice() writes a voice-frame into transmit buffer
 * keep an eye of buffer overflow's and write to the buffer, who was in tx
 */
void rptr_addtxvoice(const tds_voicedata *buf, unsigned char pkt_nr) {
  tds_voicedata *new_data;
  TxVoice_StopPos = TxVoice_RdPos;	// update stop-position, long-gap silence

  if ((pkt_nr >= VoiceTxBufSize) || (pkt_nr == VOICE_TX_DIFFERENCE(TxVoice_RdPos, 1)))
    return;		// prevent buffer overflow OR writing on current TXed buffer.

  new_data = &DStar_TxVoice[pkt_nr];
  memcpy(new_data, buf, sizeof(tds_voicedata));	// copy new data
  if (pkt_nr == TxVoice_WrPos) {		// expected packet from PC...
    TxVoice_WrPos = (TxVoice_WrPos+1) % VoiceTxBufSize;
  } else {		// unsorted packet - expand transmit window to pkt_nr
    // a numbered packet in the empty buffer area results in an update of WrPos
    // (last packet), IF(!) the gap is lower as a threshold
    unsigned int nr_difference = VOICE_TX_DIFFERENCE((U32)pkt_nr, TxVoice_WrPos);
    if (nr_difference < VOICE_TX_ALLOWED_GAP)
      TxVoice_WrPos = (pkt_nr+1) % VoiceTxBufSize;
  } // esle with gaps
  RPTR_clear(RPTR_TX_EMPTY);
}


__inline unsigned char rptr_get_unsend(void) {
  return VOICE_TX_DIFFERENCE(TxVoice_WrPos, TxVoice_RdPos);
}


__inline unsigned char rptr_get_txpos(void) {
  return TxVoice_RdPos;
}


//! @}
