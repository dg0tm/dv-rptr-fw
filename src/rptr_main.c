/*
 * Standard DV-RPTR Firmware
 * -------------------------
 * This project is firmware of the AVR32 DV-RPTR, developed by J. Alte.
 * It processes various converting modes for digitized speech signals.
 * See www.digisolutions.de (german only) for more information about DV-Modem.
 *
 * Copyright (c) 2011 by Jan Alte, DO1FJN
 *
 *
 * rptr_main.c (main source file of DVfirmware)
 *
 *  Created on: 2011-07-26 (developed from DV-Modem)
 *      Author: Jan Alte, DO1FJN
 *
 *
 * This firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * All source files of DVfirmware are written in UTF-8 charset.
 *
 * Report:
 * 2011-07-26 V0.01  Erstes Firmware-Release
 *
 *
 * ToDo:
 * ALL!!! This Version is only a sum of files, copied from DV-Firmware to
 * build a new (smaller) project for DV-RPTR
 */


#include "defines.h"		// general defines
#include "crc.h"
#include "rptr_func.h"		// realtime-handler part of HF I/O
#include "slowdata.h"		// later used on own (idle) transmittings

#include "gpio_func.h"		// GPIO-functions and macros (PTT, LEDs...)

#include "usb_func.h"

#include "intc.h"
#include "compiler.h"


typedef int (*tdata_rx_fct)(void);	// Data-Received function (return #bytes)
typedef int (*tdata_cpy_rx)(char *, int);	// Copy received data
typedef void (*tdata_tx_fct)(const char *, int); // Data-Transmit function


typedef struct PACKED_DATA {
  tRS232pktHeader	head;
  unsigned char		rxid;
  unsigned char		rsvd;
  unsigned short	crc;
} tctrlpcdata;

typedef struct PACKED_DATA {
  tRS232pktHeader	head;
  unsigned char		rxid;
  unsigned char		biterrs;
  tds_header		header;
  unsigned short	crc;
} theadrpcdata;

typedef struct PACKED_DATA {
  tRS232pktHeader	head;
  unsigned char		rxid;
  unsigned char		pktcount;
  tds_voicedata		DVdata;
  unsigned short	crc;
} tvoicepcdata;




typedef enum {
  RPTR_GET_STATUS=0x10, RPTR_GET_VERSION, RPTR_GET_CONFIG, RPTR_SET_CONFIG,
  RPTR_RXPREAMBLE, RPTR_START, RPTR_HEADER, RPTR_RXSYNC, RPTR_DATA, RPTR_EOT,
  RPTR_RXLOST
} tRPTRcmds;


// *** Globale Variablen ***

// functions used for pc/gateway communication:
// pre-initialisation to USB-CDC, but easy reconfigurable to RS232
tdata_rx_fct	data_received	= cdc_received;
tdata_cpy_rx	data_copyrx	= cdc_copyblock;
tdata_tx_fct	data_transmit	= cdc_transmit;
tfunction	data_flushrx	= cdc_flushrx;



tRS232paket	rxdatapacket;	// data from pc/gateway received
tRS232paket	answer;		// data to reply (get cmds)

tctrlpcdata	ctrldata;
theadrpcdata	headerdata;
tvoicepcdata	voicedata;	// Voice+Slowdata from HF



void init_pcdata(void) {
  ctrldata.head.id  = FRAMESTARTID;
  ctrldata.head.len = swap16(sizeof(headerdata)-5);
  ctrldata.rxid = 0;
  headerdata.head.id  = FRAMESTARTID;
  headerdata.head.len = swap16(sizeof(headerdata)-5);
  headerdata.head.cmd = RPTR_HEADER;
  voicedata.head.id   = FRAMESTARTID;
  voicedata.head.len  = swap16(sizeof(voicedata)-5);
  voicedata.head.cmd  = RPTR_RXSYNC;
}


__inline void pc_send_byte(U8 data) {
  answer.head.len = 2;
  answer.data[PKT_PARAM_IDX] = data;
}


void pc_fill_answer(void) {
  answer.head.id  = FRAMESTARTID;
  answer.head.len = 0;
  answer.head.cmd = 0x80|rxdatapacket.head.cmd;
}


// handle_serial_paket()
// verarbeitet "paket" mit len optionalen Daten (Header NICHT mitgerechnet).
__inline void handle_pc_paket(int len) {
  switch (rxdatapacket.head.cmd) {	// Kommando-Byte
  case RPTR_GET_STATUS:
    break;
  case RPTR_GET_VERSION:
  case RPTR_GET_CONFIG:
  case RPTR_SET_CONFIG:
  case RPTR_HEADER:		// start transmitting TXDelay-Preamble-Start-Header
    dstar_transmit();		// Turn on Xmitter
    // keep 2 bytes for future use, keep layout identical to RX
    dstar_init_header((tds_header *)&rxdatapacket.data[PKT_PARAM_IDX+2]);
    break;
  case RPTR_DATA:		// transmit data (voice and slowdata or sync)
    // keep 2 bytes for future use, keep layout identical to RX
    dstar_addtxvoice((tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+2]);
    break;
  case RPTR_EOT:		// end transmission with EOT tail
    dstar_endtransmit();
    break;
  default:
    pc_fill_answer();
    pc_send_byte(NAK);
    break;
  }
  if ((answer.head.len > 0)&&(answer.head.len<(PAKETBUFFERSIZE-4))) {
    U32 anslen = answer.head.len+5;
    answer.head.len = swap16(answer.head.len);
    append_crc_ccitt(answer.data, anslen);
    data_transmit(answer.data, anslen);
  } // fi send
}



// handle_pcdata() testet, ob im Receive-Buffer Paket-Daten (auch unvollst.) liegen
// und kopiert diese nach 'rxdatapaket' um.
// Bei gültigen Paketen, wird 'handle_pc_paket()'
// aufgerufen.
void handle_pcdata(void) {
  int rxbytes;
  rxbytes = data_received();
  if (rxbytes) { // Irgendetwas empfangen
    if (rxbytes > sizeof(rxdatapacket)) rxbytes = sizeof(rxdatapacket);
    data_copyrx(rxdatapacket.data, rxbytes);
    if (rxdatapacket.head.id == FRAMESTARTID) {
      U16 framelen = swap16(rxdatapacket.head.len);
      if ((framelen <= rxbytes) && (crc_ccitt(rxdatapacket.data, framelen+5)==0)) {
	handle_pc_paket(framelen);
      } // fi correct / enough bytes received / crc ok
    } // fi correct ID
  } // fi was da
}


void handle_hfdata(void) {
  static bool sync_received = false;
  if (RPTR_Flags!=0) {
    if (RPTR_is_set(RPTR_RX_SYNC)) {
      RPTR_clear(RPTR_RX_SYNC);
      sync_received = true;
    }
    if (RPTR_is_set(RPTR_RX_FRAME)) {
      RPTR_clear(RPTR_RX_FRAME);
      voicedata.head.cmd = (sync_received)?RPTR_RXSYNC:RPTR_DATA;
      voicedata.pktcount = RPTR_RxFrameCount;
      if (!sync_received) {	// descramble slowdata
	voicedata.DVdata.data[0] ^= 0x70;
	voicedata.DVdata.data[1] ^= 0x4F;
	voicedata.DVdata.data[2] ^= 0X93;
      }
      append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
      data_transmit((char *)&voicedata, sizeof(voicedata));
    } // fi voice data
    if (RPTR_is_set(RPTR_RX_START)) {
      RPTR_clear(RPTR_RX_START);
      ctrldata.head.cmd = RPTR_START;
      ctrldata.rxid++;
      headerdata.rxid = ctrldata.rxid;
      voicedata.rxid =  ctrldata.rxid;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_STOP)) {
      RPTR_clear(RPTR_RX_STOP);
      ctrldata.head.cmd = RPTR_EOT;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_LOST)) {
      RPTR_clear(RPTR_RX_LOST);
      ctrldata.head.cmd = RPTR_RXLOST;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_HEADER)) {
      RPTR_clear(RPTR_RX_HEADER);
      headerdata.biterrs = dstar_decodeheader(&headerdata.header, dstar_getheader());
      append_crc_ccitt((char *)&headerdata, sizeof(headerdata));
      data_transmit((char *)&headerdata, sizeof(headerdata));
    } // fi start detected

  } // fi do something
  // Testet, ob was emfangen wurde.
  // sendet sofort an pc
}


int main(void) {
  Disable_global_interrupt();		// Disable all interrupts.

  // *** Initialization-Section ***
  init_hardware();
  INTC_init_interrupts();		// Initialize interrupt vectors.

  usb_init();				// Enable VBUS-Check

  // *** Initialierung der verschiedenen Parameter ***

  dstar_init_data(&voicedata.DVdata);	// Default Header "noCall"

  Enable_global_interrupt();		// Enable all interrupts.

  dstar_receive();

  while (TRUE) {			// *** Main-Loop ***
    SLEEP();				// erst einmal dösen bis zu einem IRQ
    watchdog();
    // USB-Funktion
    usb_handler();

    handle_pcdata();

    handle_hfdata();

  } // ehliw MAINloop
}

// MAIN Ende

