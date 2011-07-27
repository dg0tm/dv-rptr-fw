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

#include "compiler.h"
#include <string.h>


#define DSTAR_SYNCINTERVAL	21

#define DSTAR_SYNC		0x552D1600
#define DSTAR_SYNCMSK		0xFFFFFF00
#define DSTAR_SCRAMDATA		0x0EF2C902



static const unsigned long int preamble_dstar[5] = {
  0x55555555, 0x55555555,
  0x55555555, 0x55555555,
  0x37050000
};

static const unsigned long int lastframe_dstar[2] = {
  0x55555555,
  0xC87A0000
};
#define DSTAR_LASTFRAMEBITSIZE_TX	64	// keep modulated at END


// Globale Variablen für D-Star-Transmittions:

tds_header DSTAR_HEADER = {		// Der zu sendende Header
    flags:{0x00, 0x00, 0x00},
    RPT2Call:"        ",
    RPT1Call:"        ",
    YourCall:"CQCQCQ  ",
    MyCall  :"no Call ",
#ifdef DVMODEM
    MyCall2 :" C5 ",
#elif DVATRX
    MyCall2 :"ATRX",
#else
    MyCall2 :"FJN ",
#endif
};

static unsigned long	DStar_HeaderBS[DSTAR_HEADERBSBUFSIZE];	// D* Header Bitstream
static unsigned long	DStar_RxHeader[DSTAR_HEADERBSBUFSIZE];	// Received Header Bitstream

int		DStar_RxHeaderCnt;				// Counter RX-Hader

tDV_State	dvstate;
tDV_RXstate	rxstate, last_rxstate;

unsigned int	DStar_PacketCounter;
unsigned int	DStar_LostSyncCounter;		// Zähler, wie oft pro Durchgang Sync weg
unsigned int	DStar_DataIdleCounter;
unsigned char	DStar_NFstate;


tds_voicedata	VoiceBuffer;			// D* Voice-Frame (Tx)

static char *slowdataptr;			// SlowData Sende-Bytes (auf 6er Block)
static const char noDataFill[6] = "ffffff";	// keine Daten-Funktion definiert.
// (Notfall-Funktion, normalerweise ist immer eine Fkt. zugeordnet)


tSlowDataFrame 	RxDataFrames[2];		// Double-buffered Slow-Data Frame
char		*DataRxBufPtr;
int		RxD_rpos, RxD_wpos;

tdstar_slowdatrxf	DStar_DataStream;
tdstar_slowdattxf	DStar_DataTransmit;
tdstar_function		DStar_Stophandler;

//Sync and Slow-Data functions:

void dstar_add_sync(tds_voicedata *dest) {
  dest->data[0] = 0x55;
  dest->data[1] = 0x2D;
  dest->data[2] = 0x16;
}

char *dstar_add_data(tds_voicedata *dest, const char *DataBytes) {
  dest->data[0] = (*DataBytes++) ^ 0x70;
  dest->data[1] = (*DataBytes++) ^ 0x4F;
  dest->data[2] = (*DataBytes++) ^ 0X93;
  return (char *)DataBytes;
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
//  dv_endtransmission(dvstate==DVtxemergency);
  dvstate = DVreception;
}


void dstar_transmit_stopframe(void) {
  gmsk_set_reloadfunc(&dstar_stopped);
  dstar_tx_stop();
  if (dvstate == DVtxemergency) {
//    ambe_stop();
  } else {
//    ambe_standby();
    dvstate = DVtxlastframe;
  }
}


void dstar_transmit_voicedata(void) {
  int cycle = DStar_PacketCounter%DSTAR_SYNCINTERVAL;
  //ambe_getvoice(VoiceBuffer.voice);
  if ((dvstate!=DVtxvoice)&&(dvstate!=DVtxvoicesync)) {
    gmsk_transmit(VoiceBuffer.packet, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&dstar_transmit_stopframe);
  } else {			// kein Stop-Request...
    if (cycle==0) {		// Sync-Daten in SendeBuffer
      dstar_add_sync(&VoiceBuffer);
//      if (ambe_getstate()==AMBEencoding)	// AMBE arbeitet auch?
        dvstate = DVtxvoicesync;
//      else				// wenn nicht Aussendung abbrechen!
//	dvstate = DVtxemergency;
    } else {
      if (cycle & 0x01) {		// Odd-Frames? reload 6 databytes
        dvstate = DVtxvoice;
        if (DStar_DataTransmit != NULL)	// Funktion zugeordnet?
	  slowdataptr = DStar_DataTransmit(DStar_PacketCounter);
        else
          slowdataptr = NULL;
        if (slowdataptr == NULL) {	// Funktion hat keine User-Daten (idle)
	  slowdataptr = (char *)noDataFill;
        } // fi get IdleData
      } // fi Nächster Stream-Block
      slowdataptr = dstar_add_data(&VoiceBuffer, slowdataptr);
    } // esle
    gmsk_transmit(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    DStar_PacketCounter++;
  }
}


void dstar_transmit_firstvoice(void) {
/* if (ambe_packetcount()==0) {		// AMBE funktioniert nicht bzw. steckt im Sleep
    gmsk_set_reloadfunc(&dstar_stopped);
    dstar_tx_stop();
    ambe_stop();
    dvstate = DVtxemergency;
  } else*/ if (dvstate==DVtxstop) {
    gmsk_set_reloadfunc(&dstar_stopped);
    dstar_tx_stop();
//    ambe_standby();
    dvstate = DVtxlastframe;
  } else {
//    ambe_getvoice(VoiceBuffer.voice);
    dstar_add_sync(&VoiceBuffer);
    gmsk_transmit(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    DStar_PacketCounter = 1;
    gmsk_set_reloadfunc(&dstar_transmit_voicedata);
    dvstate = DVtxvoicesync;
  } // fi
}


// D-Star: Transmit SlowData only (VoiceBuffer contains Silence)
void dstar_transmit_slowdata(void) {
  int cycle = DStar_PacketCounter%DSTAR_SYNCINTERVAL;
  if ((dvstate!=DVtxdata)&&(dvstate!=DVtxdatasync)) {
    gmsk_transmit(VoiceBuffer.packet, DSTAR_VOICEFRAMEBITSIZE, 1);
    gmsk_set_reloadfunc(&dstar_transmit_stopframe);
  } else {			// kein Stop-Request...
    if (cycle==0) {		// Sync-Daten in SendeBuffer
      dstar_add_sync(&VoiceBuffer);
      dvstate = DVtxdatasync;
    } else {
      if (cycle & 0x01) {		// Odd-Frames? reload 6 databytes
        dvstate = DVtxdata;
        if (DStar_DataTransmit != NULL)	// Funktion zugeordnet?
	  slowdataptr = DStar_DataTransmit(DStar_PacketCounter);
        else
          slowdataptr = NULL;
        if (slowdataptr == NULL) {	// Funktion hat keine User-Daten (idle)
	  slowdataptr = (char *)noDataFill;
        } // fi get IdleData
      } // fi Nächster Stream-Block
      slowdataptr = dstar_add_data(&VoiceBuffer, slowdataptr);
    } // esle
    gmsk_transmit(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
//    if (ambe_getstate()==AMBEencoding) {	// AMBE arbeitet - Wechsel zu Normalbetrieb
      dvstate = DVtxvoice;
      gmsk_set_reloadfunc(&dstar_transmit_voicedata);
//    } else if (DStar_PacketCounter==DStar_DataIdleCounter) {
//      DStar_DataIdleCounter += 2;		// repeat it after 2 frames (40ms)
//      dv_enddatatx();
//    } // fi no new data
    DStar_PacketCounter++;
  }
}


void dstar_transmit_firstsdata(void) {
  if (dvstate==DVtxstop) {
    gmsk_set_reloadfunc(&dstar_stopped);
    dstar_tx_stop();
    dvstate = DVtxlastframe;
  } else {
    dstar_add_sync(&VoiceBuffer);
    gmsk_transmit(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE, DSTAR_FRAMEBITSIZE-DSTAR_BEFOREFRAMEENDS);
    DStar_PacketCounter = 1;
    DStar_DataIdleCounter = DSTAR_DATAIDLETIMEOUT;
    gmsk_set_reloadfunc(&dstar_transmit_slowdata);
    dvstate = DVtxdatasync;
  } // fi
}



void dstar_transmit_header(void) {
  gmsk_transmit((U32 *)&DStar_HeaderBS, DSTAR_HEADEROUTBITSIZE, DSTAR_HEADEROUTBITSIZE-DSTAR_BEFOREFRAMEENDS);
  DStar_PacketCounter = 0;
  if (dvstate==DVtxstartdata)
    gmsk_set_reloadfunc(&dstar_transmit_firstsdata);
  else
    gmsk_set_reloadfunc(&dstar_transmit_firstvoice);
  dvstate = DVtxheader;
}




// *** Demodulator Handler ***


void dstar_receivedframe(void) {
  char *SlowDataPtr;
  DStar_PacketCounter++;
  if (DStar_PacketCounter < 50) {
//    ambe_putvoice(VoiceBuffer.voice);
    gmsk_set_receivebuf(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE);
    if (DStar_PacketCounter < 21) {
      SlowDataPtr  = DataRxBufPtr;
      DataRxBufPtr = dstar_get_data(SlowDataPtr, &VoiceBuffer);
      if (DStar_PacketCounter==20) {
        RxD_wpos++;	// Inc NoOfWritePos
        DataRxBufPtr = RxDataFrames[RxD_wpos&0x01];	// Select next Buffer
      } // fi SlowData complete
      // SlowData-Stream aufbereiten
      if (DStar_DataStream != NULL) DStar_DataStream(SlowDataPtr, DStar_PacketCounter);
    }
    rxstate = DVrxvoice;
  } else {			// fi Valid Data
    rxstate = DVsilence;
//    ambe_standby();		// warscheinich keine gültigen Daten mehr.
    if (DStar_Stophandler != NULL) DStar_Stophandler();
  }
  if ( (DStar_PacketCounter%21)==0 ) {	// Every 21 frame
    if (dstar_syncpaket(&VoiceBuffer)) {
      DStar_PacketCounter = 0;
    } else {
      DStar_LostSyncCounter++;
    }
  } // fi
}


void dstar_receivedframesync(void) {
  gmsk_set_receivebuf(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(&dstar_receivedframe);
  DStar_PacketCounter = 0;
  DataRxBufPtr = RxDataFrames[RxD_wpos&0x01];	// Select next Buffer
//  ambe_decode();
  dvstate = DVreception;
}


void dstar_receivedhdr(void) {
  gmsk_set_receivebuf(VoiceBuffer.packet, DSTAR_FRAMEBITSIZE);
  gmsk_set_receivefkt(&dstar_receivedframe);
  DStar_PacketCounter = 0;
  DStar_LostSyncCounter = 0;
  DataRxBufPtr = RxDataFrames[0];
  RxD_rpos = 0;
  RxD_wpos = 0;
  DStar_RxHeaderCnt++;
  rxstate = DVrxvoice;
}


// Handler to Receive Header in Mem-Buffer
void dstar_getrxheader(void) {
  gmsk_set_receivebuf(DStar_RxHeader, DSTAR_HEADEROUTBITSIZE);
  gmsk_set_receivefkt(&dstar_receivedhdr);
//  ambe_decode();		// Time to Boot AMBE
  dvstate = DVreception;
  rxstate = DVrxheader;
}


// Handler to Receive Header in Mem-Buffer
void dstar_gotstopframe(void) {
  gmsk_set_receivebuf(NULL, 0);
  gmsk_set_receivefkt(NULL);
//  ambe_standby();
  rxstate = DVsilence;
  if (DStar_Stophandler != NULL) DStar_Stophandler();
}

//! @}

/*! \name DSTAR API Functions
 */
//! @{


// *** API Funktionen ***

void dstar_init_data(void) {
  append_crc_ccitt_revers((char *)&DSTAR_HEADER, sizeof(tds_header));
  dstar_buildheader(DStar_HeaderBS, &DSTAR_HEADER);
//  dstar_buildheader_sd(TxIdleFrame, &DSTAR_HEADER);
  DStar_DataStream   = NULL;
  DStar_DataTransmit = NULL;
  DStar_Stophandler  = NULL;
}


void dstar_init_hardware(void) {
//  ambe_init();			// Init SSC for AMBE-data-transfers and AMBE I/O Buffer
//  ambe_set_timeout(DSTAR_DECODE_TO);
  gmsk_init();			// Init (De)Modulator Timer
  rxstate      = DVnorx;
  last_rxstate = DVnorx;
  dvstate      = DVdisabled;
}


void dstar_exit_hardware(void) {
  gmsk_exit();
//  ambe_powerdown();
  dvstate = DVdisabled;
}


tDV_returncode dstar_update_header() {
  append_crc_ccitt_revers((char *)&DSTAR_HEADER, sizeof(tds_header));
//  dstar_buildheader_sd(TxIdleFrame, &DSTAR_HEADER);
  if ((dvstate != DVtxheader) && (dvstate != DVtxpreamble)) {
    dstar_buildheader(DStar_HeaderBS, &DSTAR_HEADER);
    return DVsuccess;
  } else  return DVbusy;
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


void dstar_standby(void) {
  // stopp RX bis wieder Sync/Framesync.
  if (dvstate==DVreception) {
    gmsk_demodulator_start();
  }
/*  if (ambe_getstate() > AMBEpowerdown) {
    ambe_standby();	// Standby
    ambe_getnewstate();	// Sleep ist nicht "neu"
  }*/
}



void dstar_receive(void) {
  gmsk_set_synchandler(&dstar_getrxheader, &dstar_gotstopframe, &dstar_receivedframesync);
  if ((dvstate==DVdisabled)||(dvstate==DVreception)) {
    gmsk_demodulator_start();
    dvstate = DVreception;
  } else {  // fi
    dvstate = DVtxstop;		// Stop Transmission
  } // esle
  rxstate = DVsilence;
}


// später mit update_header (Übergabe)
void dstar_transmit(void) {
  if ((dvstate!=DVtxdata)&&(dvstate!=DVtxdatasync)) {	// Wechsel von Data
    if (dvstate != DVdisabled) {
      gmsk_stoptimer();
    } // fi
    dvstate = DVtxpreamble;			// erstmal, später synchron
    gmsk_set_reloadfunc(&dstar_transmit_header);	// unmittelbar Header hinter
    dstar_tx_preamble();				// die Preamble setzen
  } // fi not tx data
//  ambe_encode();			// Release AMBE2020 reset (enable function)
  rxstate = DVnorx;
}


void dstar_transmit_data(void) {
  if ((dvstate!=DVtxvoice)&&(dvstate!=DVtxvoicesync)) {	// Wechsel von Sprache
    if (dvstate != DVdisabled) {
      gmsk_stoptimer();
    } // fi
    dvstate = DVtxstartdata;			// Nach dem Header, nur Slow-Data ohne Voice
    gmsk_set_reloadfunc(&dstar_transmit_header);	// unmittelbar Header hinter
    dstar_tx_preamble();				// die Preamble setzen
  } else {	// fi not sprache
//    ambe_standby();
    dvstate = DVtxdata;
    gmsk_set_reloadfunc(&dstar_transmit_slowdata);	// unmittelbar Header hinter
  } // esle
//  ambe_getsilence(VoiceBuffer.voice);
  rxstate = DVnorx;
}


int dstar_newheader(void) {
  int rxres = (DStar_RxHeaderCnt);
  DStar_RxHeaderCnt = 0;
  return rxres;
}


unsigned long *dstar_getheader(void) {
  return DStar_RxHeader;
}


int dstar_new_rxstate(void) {
  return (last_rxstate != rxstate);
}


tDV_RXstate dstar_get_rxstate(void) {
  last_rxstate = rxstate;
  return rxstate;
}


int dstar_channel_idle(void) {
  return (rxstate == DVsilence)&&(gmsk_channel_idle());
}


__inline int dstar_newrxSlowData(void) {
  return RxD_wpos-RxD_rpos;	// returns NoOfReceived SlowData
}

char *dstar_getrxSlowData(void) {
  char *dataptr = RxDataFrames[RxD_rpos&0x01];
  if (RxD_rpos!=RxD_wpos) RxD_rpos++;
  return dataptr;
}

__inline void dstar_setslowdatarxfct(tdstar_slowdatrxf HandleFct) {
  DStar_DataStream = HandleFct;
}

__inline void dstar_setslowdatatxfct(tdstar_slowdattxf HandleFct) {
  DStar_DataTransmit = HandleFct;
}

__inline void dstar_setstopfunction(tdstar_function HandleFct) {
  DStar_Stophandler = HandleFct;
}


//! @}
