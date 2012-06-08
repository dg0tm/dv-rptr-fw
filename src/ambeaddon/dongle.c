/*
 * dongle.c
 *
 *  Created on: 14.04.2012
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR firmware (RPTRfirmware).
 * For general information about RPTRfirmware see "rptr-main.c".
 *
 * RPTRfirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The RPTRfirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 * Report:
 * 2012-04-25	Add "55 55 55" as Slowdata in the last voice packet (compatibility)
 * 2012-05-22	Autocorrection RPT1/2 works now
 */


#include "dongle.h"
#include "defines.h"
#include "hw_defs.h"

#include "crc.h"
#include "gpio_func.h"
#include "int_func.h"
#include "dv_dstar.h"
#include "gmsk_func.h"
#include "ambe_func.h"
#include "TLV320AIC.h"

#include "dv_dstar.h"
#include "slowdatatx.h"
#include "rptr_func.h"
#include "transceiver.h"

#include "compiler.h"
#include <string.h>


tds_header dongle_header = {		// header used in DONGLE transmissions
    flags:{0x00, 0x00, 0x00},
    RPT2Call: "DIRECT  ",
    RPT1Call: "DIRECT  ",
    YourCall: "CQCQCQ  ",
    MyCall  : "no Call ",
    MyCall2 : "RPTR",
    CRC:0
};

bool		dgl_micptt_enabled;

unsigned int	dgl_micptt_pinstate;
bool		dgl_micptt_triggered;

unsigned int	dgl_HeaderBS[DSTAR_HEADERBSBUFSIZE];	// DStar header bitstream

ALIGNED_DATA static tds_voicedata dgl_voice;

unsigned int	dgl_FrameCount;

tambe_mode	ambe_mode;

tfunction	dgl_function;

bool		own_route_correction;

/*! \name DSTAR Handler Functions
 */
//! @{

void dgl_inactive(void) {}		// dummy function w/o functionality
void dgl_activemicptt(void);		// forward


__inline void dgl_get_lastdata(void) {
  dgl_voice.data[0] = 0x55;
  dgl_voice.data[1] = 0x55;
  dgl_voice.data[2] = 0x55;
}


// GMSK modem handler function:

void dgl_stopped(void) {
  gmsk_set_reloadfunc(NULL);
  disable_ptt();
  trx_receive();
  RPTR_clear(RPTR_TRANSMITTING);
  RPTR_set(DGL_EOT);
  LED_Clear(LED_RED);
#if (DVTX_TIMER_CH==IDLE_TIMER_CH)
  idle_timer_start();
#else
  rptr_receive();		// reenable receiving
#endif
  if (!dgl_micptt_enabled) {	// if disabled, inactivate it
    dgl_function = dgl_inactive;
  }
  rptr_tx_state = RPTRTX_idle;
}


void dgl_transmit_stopframe(void) {
  rptr_transmit_eotpattern();
  gmsk_set_reloadfunc(dgl_stopped);
  ambe_standby();
  rptr_tx_state = RPTRTX_eot;
}


void dgl_transmit_voicedata(void) {
  U8 cycle = dgl_FrameCount%DSTAR_SYNCINTERVAL;
  ambe_getvoice(dgl_voice.voice);
  if (rptr_tx_state != RPTRTX_voicedata) {
    gmsk_transmit(dgl_voice.packet, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&dgl_transmit_stopframe);
    dgl_get_lastdata();
    rptr_tx_state = RPTRTX_lastframe;
  } else {			// kein Stop-Request...
    if (cycle==0) {		// Sync-Daten in SendeBuffer
      slowdata_get_sync(dgl_voice.data);
      if (ambe_getstate()!=AMBEencoding)	// AMBE arbeitet auch?
	rptr_tx_state = RPTRTX_lastframe;	// wenn nicht Aussendung abbrechen!
    } else {
      slowdata_get_txdata(dgl_FrameCount, dgl_voice.data);
    } // esle
    gmsk_transmit(dgl_voice.packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
  } // esle
  dgl_FrameCount++;
  RPTR_set(DGL_FRAME);
}


void dgl_transmit_firstvoice(void) {
  gmsk_transmit(dgl_voice.packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
  if (ambe_packetcount()==0) {		// AMBE funktioniert nicht bzw. steckt im Sleep
    gmsk_set_reloadfunc(dgl_stopped);
    ambe_stop();
  } else if (rptr_tx_state==RPTRTX_idle) {
    gmsk_set_reloadfunc(&dgl_stopped);
    ambe_standby();
    rptr_tx_state = RPTRTX_lastframe;
  } else {
    ambe_getvoice(dgl_voice.voice);
    slowdata_get_sync(dgl_voice.data);
    gmsk_set_reloadfunc(dgl_transmit_voicedata);
    rptr_tx_state = RPTRTX_voicedata;
    dgl_FrameCount = 1;
    RPTR_set(DGL_FRAME);
    if (dgl_micptt_triggered) {
      dgl_function = dgl_activemicptt;	// enable check-mic-ptt for release
    }
  } // fi
}


void dgl_transmit_header(void) {
  gmsk_transmit(dgl_HeaderBS, DSTAR_HEADEROUTBITSIZE, DSTAR_HEADEROUTBITSIZE-DSTAR_BEFOREFRAMEENDS);
  gmsk_set_reloadfunc(&dgl_transmit_firstvoice);
  RPTR_set(DGL_HEADER);
  rptr_tx_state = RPTRTX_header;
  ambe_getsilence(dgl_voice.voice);
  dgl_FrameCount = 0;
}


void dgl_restart_header(void) {
  rptr_transmit_shortpreamble();
  gmsk_set_reloadfunc(dgl_transmit_header);		// unmittelbar Header hinter
  rptr_tx_state = RPTRTX_preamble;
}


void dgl_begin_new_tx(void) {
  rptr_transmit_brkeot();
  gmsk_set_reloadfunc(dgl_restart_header);
}


void dgl_break_current(void) {
  ambe_getsilence(dgl_voice.voice);
  gmsk_transmit(dgl_voice.packet, DSTAR_LASTFRAMEBITSIZE, 1);
  gmsk_set_reloadfunc(dgl_begin_new_tx);
  rptr_tx_state = RPTRTX_lastframe;
}


//! @}




void dgl_waitmicptt(void) {
  unsigned int micptt = get_mic_ptt_pin();
  if (dgl_micptt_pinstate != micptt) {
    if (micptt == 0) {			// PLL pulled to LOW (active)?
      dgl_function = dgl_inactive;	// check nothing, until headertx is finished
      dgl_start_transmit();
      dgl_micptt_triggered = true;
    } else
      dgl_micptt_triggered = false;
    dgl_micptt_pinstate = micptt;
  } // fi toggle PTT
}


void dgl_activemicptt(void) {
  unsigned int micptt = get_mic_ptt_pin();
  if ((micptt != 0) && (micptt == dgl_micptt_pinstate)) {	// simple debounce
    dgl_stop_transmit();
    if (dgl_micptt_enabled)	// if disabled, inactivate it
      dgl_function = dgl_waitmicptt;
    else
      dgl_function = dgl_inactive;
    dgl_micptt_triggered = false;
  }
  dgl_micptt_pinstate = micptt;
}



int dgl_is_inuse(void) {
  return (dgl_function != dgl_inactive);
}

// start transmitting voicedata from AMBE
void dgl_start_transmit(void) {
  if (ambe_mode != AMBE_noboard) {		// ignore, if no board or no config
#if (DVTX_TIMER_CH==IDLE_TIMER_CH)
    idle_timer_stop();
#else
    idle_timer_start();				// disable receiving
#endif
    if (is_pttactive()) {			// DV-RPTR is transmitting, restart it
      if (dgl_micptt_triggered) return;		// already transmitting
      if (rptr_tx_state == RPTRTX_voicedata) {
        gmsk_set_reloadfunc(dgl_break_current);	// unmittelbar Header hinter EOT
      } else if (rptr_tx_state == RPTRTX_lastframe) {
        gmsk_set_reloadfunc(dgl_begin_new_tx);
      } else {
        gmsk_set_reloadfunc(dgl_transmit_header);
        rptr_transmit_fullpreamble();
      } // esle fi
    } else {
      enable_ptt();
      trx_transmit();
      gmsk_set_reloadfunc(dgl_transmit_header);	// after TXed preamble + START-pattern, load header
      rptr_transmit_fullpreamble();
      RPTR_set(RPTR_TRANSMITTING);
      LED_Set(LED_RED);
      ambe_encode();				// Release AMBE2020 reset (enable function)
      rptr_tx_state = RPTRTX_preamble;
    } // fi from idle
  } // fi not tx data
}



void dgl_stop_transmit(void) {
  if (ambe_mode != AMBE_noboard) {		// ignore, if no board
    if (is_pttactive() && (ambe_getstate() >= AMBEbooting)) {
      rptr_tx_state = RPTRTX_lastframe;
    }
  }
}


unsigned int dgl_init(void) {
  dgl_function = dgl_inactive;		// a dummy function, if no AMBE addon board found
  dgl_micptt_enabled = false;
  ambe_mode    = AMBE_noboard;
  ambe_setup();				// setup memory structures for SSC I/O needed by AMBE
  dgl_micptt_pinstate = get_mic_ptt_pin();
  if (tlv_init()) {			// if auto-detect a AMBE addon board...
    ambe_init();			// setup AMBE functions
    ambe_set_timeout(DSTAR_DECODE_TO);
    ambe_mode = AMBE_initialized;
    return DONGLE_AVAIL;
  } // fi found AddOn board
  return 0;
}


int dgl_is_encoding(void) {
  return (is_pttactive() && dgl_micptt_triggered);
}


int dgl_can_encode_by_ptt(void) {
  return dgl_micptt_enabled;
}


void dgl_get_header(tds_header *target) {
  memcpy(target, &dongle_header, sizeof(dongle_header));
}


unsigned char dgl_copyvoice(tds_voicedata *dest) {
  U8 pkt = (dgl_FrameCount+(VoiceRxBufSize-1))%VoiceRxBufSize;
  memcpy(dest, dgl_voice.packet, sizeof(tds_voicedata));
  return pkt;
}



void dgl_routeflags(void) {
  char *rr1 = dongle_header.RPT1Call;
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
    dongle_header.flags[0] |= FLAG0_RPT_MASK;
  else {
    dongle_header.flags[0] &= ~FLAG0_RPT_MASK;
    memcpy(dongle_header.RPT2Call, "DIRECT  DIRECT  ", 16);
  }
}

#define DGL_MYCALL_COMPARELEN	6

void dgl_ProcessHdr(const tds_header *RxHeader) {
  U8 RptInfo = RxHeader->flags[0]&0x07;
  own_route_correction = false;
  if (RptInfo != 0) {					// Repeater Information
    if ( memcmp(RxHeader->YourCall, dongle_header.MyCall, DGL_MYCALL_COMPARELEN)==0 ) {	// I'm recepient?
      if (memcmp(RxHeader->MyCall, RxHeader->RPT2Call, DGL_MYCALL_COMPARELEN)==0) {
        char RptModul;
	// von Gateway oder Repeater gesendet?
	// 1. compare Area (RPT1) with rxed RPT2 (fields are exchanged)
	if ( memcmp(dongle_header.RPT1Call, RxHeader->RPT2Call, 8) != 0 ) {
	  memcpy(dongle_header.RPT1Call, RxHeader->RPT2Call, 8); // replace RPT2
	  own_route_correction = true;		// Set "corrected my own route"
	} // fi different Area (RPT1)
	// 2. compare Dest (RPT2)
	// Test first 7 chars of destination (last one needs special handling)
	if ( memcmp(dongle_header.RPT2Call, RxHeader->RPT1Call, 7) != 0 ) {
	  memcpy(dongle_header.RPT2Call, RxHeader->RPT1Call, 7); // eigene Route anpassen (RPT2)
	  own_route_correction = true;			// Set "have corrected my own route"
	} // fi different Dest (RPT2; first 7 chars)

	// Sonderbehandlung / WorkAround
	// Icom-HW liefert mit alternativer Software immer "lokal" zurück.
	// Ist dies der Fall, ersetze 8tes Zeichen durch "G" für gateway
	RptModul = RxHeader->RPT1Call[7];	// get letter of "gateway/destination"
	if (RxHeader->RPT2Call[7] == RptModul) { // both in RX are the same?
	  // special handling: replace the module-letter by "G".
	  RptModul = 'G';
	} // fi special
	if (dongle_header.RPT2Call[7] != RptModul) {	// is different to my own?
	  dongle_header.RPT2Call[7] = RptModul;		// replace it.
	  own_route_correction = true;
	} // fi Dest[7]

	if (own_route_correction) {
	  dgl_routeflags();
	  append_crc_ccitt_revers((char *)&dongle_header, sizeof(tds_header));
	  dstar_buildheader(dgl_HeaderBS, &dongle_header);
	  ambe_double_tone(AMBE_ERRTONE, 6, 6, AMBE_ERRTONE);
	} else
	  ambe_set_dtmf(AMBE_ACKTONE, 10);
      } // fi von RPT1
    } // fi
  }
}


char *cfg_read_c2(char *config_buffer) {
  config_buffer[0] = 0xC2;			// identifier byte
  config_buffer[1] = CONFIG_C2_SIZE;		// length
  // Bytes 0,1,2,3: Controlflags
  memset(config_buffer + 2, 0, 4);

  if (RPTR_is_set(RPTR_AMBEDECODEHF)) config_buffer[2] |= 0x01;
  if (RPTR_is_set(RPTR_AMBEDECODEINET)) config_buffer[2] |= 0x02;
  if (dgl_micptt_enabled) config_buffer[2] |= 0x10;

  // Bytes 4ff = HeaderData
  memcpy(config_buffer + 6, &dongle_header.RPT2Call, 36);
  return config_buffer + 2 + CONFIG_C2_SIZE;
}


char *cfg_read_c3(char *config_buffer) {
  config_buffer[0] = 0xC3;			// identifier byte
  config_buffer[1] = CONFIG_C3_SIZE;		// length
  memcpy(config_buffer + 2, Get_SlowData_TXMsg(), CONFIG_C3_SIZE);
  return config_buffer + 2 + CONFIG_C3_SIZE;
}


void cfg_write_c2(const char *config_data) {
  // write new DONGLE header data
  memcpy(&dongle_header.RPT2Call, config_data + 4, 36);
  // process new header
  dgl_routeflags();
  append_crc_ccitt_revers((char *)&dongle_header, sizeof(tds_header));
  dstar_buildheader(dgl_HeaderBS, &dongle_header);
  // Control-Flags
  RPTR_clear(RPTR_AMBEDECODEHF|RPTR_AMBEDECODEINET);
  if (config_data[0] & 0x01) RPTR_set(RPTR_AMBEDECODEHF);
  if (config_data[0] & 0x02) RPTR_set(RPTR_AMBEDECODEINET);
  // Control PTT
  dgl_micptt_enabled = (config_data[0] & 0x10) != 0;
  if (ambe_mode != AMBE_noboard){
    if (dgl_micptt_enabled) {	// enable mictrophone PTT
      if (dgl_function == dgl_inactive)
	dgl_function = dgl_waitmicptt;
    } else {
      dgl_stop_transmit();
    }
  }
}


void cfg_write_c3(const char *config_data) {
  Set_SlowData_TXMsg(config_data);
}


