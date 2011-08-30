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
 * 2011-07-28 V0.02  Empfang -> D0-Pakete funktioniert nun. TX nicht getestet.
 * 2011-07-29 V0.03  Reception & Transmit uses buffers with 1-SYNC-Size (21 frames).
 * 		     rename all repeater-function calls from "dstar_xxx()" to "rptr_xxx()"
 * 2011-08-19 V0.04  RXSYNC received from PC results in TX (last header used)
 * 		     No CRC needed on USB (control with bool var 'rx_crc_disabled'
 *
 * 2011-08-30 V0.50  first version for DV-RPTR Target, w/o setup. cmd VERSION works
 *
 * ToDo:
 * - first SYNC detect -> Message (early switch on Transceiver)
 * - testing transmit-logic
 * - configuration: Hotspot/Repeater, TXDelay, Modulation-Vss ...
 */


#include "defines.h"		// general defines
#include "hw_defs.h"		// PTT-Macro
#include "crc.h"
#include "rptr_func.h"		// realtime-handler part of HF I/O
#include "slowdata.h"		// later used on own (idle) transmittings
#include "controls.h"

#include "gpio_func.h"		// GPIO-functions and macros (PTT, LEDs...)

#include "usb_func.h"

#include "intc.h"
#include "compiler.h"

#include <string.h>


typedef int (*tdata_rx_fct)(void);		// Data-Received function (return #bytes)
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
  tRS232pktHeader	head;		// 4byte
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
bool		rx_crc_disabled = true;



tRS232paket	rxdatapacket;	// data from pc/gateway received
tRS232paket	answer;		// data to reply (get cmds)

tctrlpcdata	ctrldata;
theadrpcdata	headerdata;
tvoicepcdata	voicedata;	// Voice+Slowdata from HF



void init_pcdata(void) {
  ctrldata.head.id  = FRAMESTARTID;
  ctrldata.head.len = swap16(sizeof(ctrldata)-5);
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
  answer.head.len = 0;			// no answer.
  switch (rxdatapacket.head.cmd) {	// Kommando-Byte
  case RPTR_GET_STATUS:
    break;
  case RPTR_GET_VERSION:
    pc_fill_answer();
    answer.data[PKT_PARAM_IDX+0] = FIRMWAREVERSION & 0xFF;
    answer.data[PKT_PARAM_IDX+1] = FIRMWAREVERSION >> 8;
    memcpy(answer.data+PKT_PARAM_IDX+2, VERSION_IDENT, sizeof(VERSION_IDENT));
    answer.head.len = sizeof(VERSION_IDENT)+3-1;	// cut String-Terminator /0
    break;
  case RPTR_GET_CONFIG:
  case RPTR_SET_CONFIG:
    break;
  case RPTR_START:		// early Turn-On xmitter, if configured a long TXD
    rptr_transmit_early_start(); // PTTon, only if TXD > Header-TX-Lengh 137.5ms
    break;
  case RPTR_HEADER:		// start transmitting TXDelay-Preamble-Start-Header
    rptr_transmit();		// Turn on Xmitter
    // keep 2 bytes for future use, keep layout identical to RX
    rptr_init_header((tds_header *)&rxdatapacket.data[PKT_PARAM_IDX+2]);
    break;
  case RPTR_RXSYNC:		// start transmitting TXDelay-Preamble-Start-Header
    if (!is_pttactive()) rptr_transmit();	// Turn on Xmitter only if PTT off
    // (transmission use last header)
    break;
  case RPTR_DATA:		// transmit data (voice and slowdata or sync)
    // keep 2 bytes for future use, keep layout identical to RX
    rptr_addtxvoice((tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+2],
      rxdatapacket.data[PKT_PARAM_IDX+1]);
    break;
  case RPTR_EOT:		// end transmission with EOT tail
    rptr_endtransmit();
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
      U16 pkt_crc = 0;
      U16 framelen = swap16(rxdatapacket.head.len);
      if (!rx_crc_disabled) {	// check CRC only, if needed.
	pkt_crc = crc_ccitt(rxdatapacket.data, framelen+5);
      } // fi check CRC
      if ((framelen <= rxbytes) && (pkt_crc==0)) {
	handle_pc_paket(framelen);
      } // fi correct / enough bytes received / crc ok
    } // fi correct ID
  } // fi was da
}


/* handle_hfdata() processes bit-flags from rptr_func
 * A typical reception look like this:
 *
 * D0 | 03 00 | 14 | 00 00 crc crc	>Preamble detected (not implemented jet)
 * D0 | 03 00 | 15 | 01 00 crc crc	>Start-Frame-Pattern detected
 * D0 | 2C 00 | 16 | 01 00 {header data} crc crc
 *                      ^ Biterrors in header (not implemented jet)
 * D0 | 0F 00 | 18 | 01 00 {VoiceData} 55 2D 16 crc crc > Voicedata with FrameSync
 *                      ^ pktcount 0 to 20
 * D0 | 0F 00 | 18 | 01 01 {VoiceData} {SlowData} crc crc > Slowdata not descrambled
 * ...
 * D0 | 03 00 | 19 | 01 00 crc crc	> End of Transmission
 *                   ^ transmission counter 1..255,0
 *              ^ cmd / type of paket ( = part of pkt-data )
 *      ^ Length of Data (packetlength-5)
 * ^ FramestartID
 */

void handle_hfdata(void) {
  static bool transmission = false;
  if (RPTR_Flags!=0) {
    if (RPTR_is_set(RPTR_RX_SYNC)) {
      RPTR_clear(RPTR_RX_SYNC);
      if (!transmission) {
        ctrldata.head.cmd = RPTR_RXSYNC;
        ctrldata.rxid++;
        headerdata.rxid = ctrldata.rxid;
        voicedata.rxid  = ctrldata.rxid;
        append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
        data_transmit((char *)&ctrldata, sizeof(ctrldata));
        transmission = true;
      }
    }
    if (RPTR_is_set(RPTR_RX_FRAME)) {
      RPTR_clear(RPTR_RX_FRAME);
      voicedata.head.cmd = RPTR_DATA;
      voicedata.pktcount = rptr_copycurrentrxvoice(&voicedata.DVdata);
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
      transmission = true;
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_STOP)) {
      RPTR_clear(RPTR_RX_STOP);
      ctrldata.head.cmd = RPTR_EOT;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
      transmission = false;
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_LOST)) {
      RPTR_clear(RPTR_RX_LOST);
      ctrldata.head.cmd = RPTR_RXLOST;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
      transmission = false;
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_HEADER)) {
      RPTR_clear(RPTR_RX_HEADER);
      if (!transmission) {
        ctrldata.rxid++;
        headerdata.rxid = ctrldata.rxid;
        voicedata.rxid  =  ctrldata.rxid;
        transmission    = true;
      }
      headerdata.biterrs = dstar_decodeheader(&headerdata.header, rptr_getheader());
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
  rptr_init_hardware();
  usb_init();				// Enable VBUS-Check

  // *** Initialierung der verschiedenen Parameter ***

  rptr_init_data();			// Default Header "noCall"
  init_pcdata();			// Initialisation of persitent Pkts (Header, Voice)

  Enable_global_interrupt();		// Enable all interrupts.
  LEDs_Off();

  // Replace with SETUP later...
  set_dac_power_mode(TWI_DAC_POWERUP);	// Enable Reference-DAC MAX5820
  set_chA_level(100 * 256 / V_Ref_5);	// Set Vss to 1.00V
  set_chB_level(100 * 256 / V_Ref_5);

  rptr_receive();			// enable receiving.

  while (TRUE) {			// *** Main-Loop ***
    SLEEP();				// do nothing, until some IRQ wake the CPU
    watchdog();				// do external watchdog for ST706T

    handle_hfdata();			// look, if something to do with the HF interface

    usb_handler();			// handle USB-based reception and enumeration

    handle_pcdata();			// got a message / pkt from pc/gateway? handle it.

  } // ehliw MAINloop
}

// MAIN Ende

