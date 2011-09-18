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
 * 2011-08-30 V0.05  first version for DV-RPTR Target, w/o setup. cmd VERSION works
 * 2011-09-01 V0.06  large transmit buffer, new - I hope final - CMD set
 * 2011-09-02 V0.07  Checksum enable-function
 * 2011-09-03 V0.08  MODFDIS bit @ dac_init(), Test-Loop (0x1F cmd)
 *                   bugfix handle_pcdata() length check
 * 2011-09-05 V0.09  rptr_addtxvoice() nummeration logic bugfix
 * 2011-09-06 V0.10  gmsk: Additional Interrupt-Handler keeps critical Timer-based ADC-start, DAC-out
 * 		     with a minimum of jitter in the case of duplex operation
 * 2011-09-18 V0.12  various tests with demodulator.
 *
 * ToDo:
 * - enable / disable receiver (if disabled keep firmware alive by a idle-counter)
 * - enable / disable transmitter (logic only, return NAK on START / SYNC cmd if off)
 * - Transmitter-State enumeration for GET_STATUS
 * - PC watchdog
 * - first SYNC detect -> Message (early switch on Transceiver)
 * + bugfix transmit-logic
 * + configuration: TXDelay, Modulation-Vss ...
 */


#include "defines.h"		// general defines
#include "hw_defs.h"		// PTT-Macro
#include "dac_func.h"
#include "crc.h"
#include "rptr_func.h"		// realtime-handler part of HF I/O
#include "slowdata.h"		// later used on own (idle) transmittings
#include "gmsk_func.h"		// configuration
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
  RPTR_GET_STATUS=0x10, RPTR_GET_VERSION, RPTR_GET_SERIAL, RPTR_GET_CONFIG,
  RPTR_SET_CONFIG, RPTR_RXPREAMBLE, RPTR_START, RPTR_HEADER,
  RPTR_RXSYNC, RPTR_DATA, RPTR_EOT, RPTR_RXLOST,
  RPTR_MSG_RSVD1, RPTR_MSG_RSVD2, RPTR_MSG_RVSD3, RPTR_SET_TESTMDE
} tRPTRcmds;


#define STA_RXENABLE_MASK	0x01
#define STA_TXENABLE_MASK	0x02
#define STA_WDENABLE_MASK	0x04
#define STA_CRCENABLE_MASK	0x08
#define STA_IO21_STATE		0x10
#define STA_IO23_STATE		0x20
#define STA_NOCONFIG_MASK	0x80

#define STA_RECEIVING		0x01
#define STA_TRANSMITTING	0x02
#define STA_PCWATCHDOG		0x04

// *** Globale Variablen ***

U8		status_control =  STA_NOCONFIG_MASK;	// Holds persitent control-flags
U8		status_state;	// Holds state of RX/TX

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


// configuration routines - physical config
typedef struct PACKED_DATA {
  unsigned char flags;
  unsigned char mod_level;
  unsigned short txdelay;	// attention: little-endian here!
} t_config_0;

#define MAX_ALLOWED_TXDELAY	6000	// 6s
#define C0FLAG_RXINVERS		0x01
#define C0FLAG_TXINVERS		0x02
#define C0FLAG_CHAN_B		0x04

t_config_0 CONFIG_C0 = {
    0x00,			// flags
    1000 * 256 / V_Ref,		// modulation voltage peak-peak ~ 1.00V
    GMSK_STDTXDELAY<<8		// little endian!
};



void update_status(void) {
#ifdef DVRPTR
  if (gpio0_readpin(EXP_IO21_PIN))
    status_control &= ~STA_IO21_STATE;
  else
    status_control |= STA_IO21_STATE;
  if (gpio0_readpin(EXP_IO23_PIN))
    status_control &= ~STA_IO23_STATE;
  else
    status_control |= STA_IO23_STATE;
#endif
  status_state = 0;
  if (RPTR_is_set(RPTR_RECEIVING))    status_state |= STA_RECEIVING;
  if (RPTR_is_set(RPTR_TRANSMITTING)) status_state |= STA_TRANSMITTING;
}


void cfg_apply(void) {
  U16 txd;
  // Receive Inversion
  gmsk_demodulator_invert(CONFIG_C0.flags & C0FLAG_RXINVERS);
  // Transmitter Inversion
  gmsk_set_mod_hub(((CONFIG_C0.flags & C0FLAG_TXINVERS)?-1:1) * GMSK_DEFAULT_BW);
  // Select Channel A or B from Dual DAC (FSK/AFSK on connector)
  dac_set_active_ch((CONFIG_C0.flags & C0FLAG_CHAN_B)?1:0);
  // adjust modulation level
  set_chA_level(CONFIG_C0.mod_level);
  set_chB_level(CONFIG_C0.mod_level);
  // setup delay before modulation
  txd = swap16(CONFIG_C0.txdelay);
  if (txd > MAX_ALLOWED_TXDELAY) txd = MAX_ALLOWED_TXDELAY;
  gmsk_set_txdelay(txd);
}


char *cfg_read_c0(char *config_buffer) {
  config_buffer[0] = 0xC0;			// identifier byte
  config_buffer[1] = sizeof(CONFIG_C0);		// length
  memcpy(config_buffer + 2, &CONFIG_C0, sizeof(CONFIG_C0));
  return config_buffer + 2 + sizeof(CONFIG_C0);
}


void cfg_write_c0(const char *config_data) {
  memcpy(&CONFIG_C0, config_data, sizeof(CONFIG_C0));
  cfg_apply();
  status_control &= ~STA_NOCONFIG_MASK;		// clear no-config bit
}


bool config_setup(const char *config_data, int len) {
  if (len == 0) return false;
  while (len > 0) {
    U8 block_len = config_data[1];
    switch (config_data[0]) {
    case 0xC0:
      if (block_len==sizeof(CONFIG_C0)) cfg_write_c0(config_data+2); else return false;
      break;
    default:
      // ignore other config blocks
      break;
    } // hctiews
    if ((block_len-2) > len) {	// get next block
      len -= block_len + 2;
      config_data += block_len + 2;
    } else len = 0;
  } // ehliw all blocks
  return true;
}



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



// handle_serial_paket()
// verarbeitet "paket" mit len optionalen Daten (Header NICHT mitgerechnet).
__inline void handle_pc_paket(int len) {
  answer.head.len = 0;			// no answer.
  switch (rxdatapacket.head.cmd) {	// Kommando-Byte
  case RPTR_GET_STATUS:
    if (len==1) {			// request
      update_status();
      answer.data[PKT_PARAM_IDX+0] = status_control;
      answer.data[PKT_PARAM_IDX+1] = status_state;
      answer.data[PKT_PARAM_IDX+2] = 0;	// ToDo TX-State
      answer.data[PKT_PARAM_IDX+3] = VoiceRxBufSize;
      answer.data[PKT_PARAM_IDX+4] = VoiceTxBufSize;
      answer.data[PKT_PARAM_IDX+5] = rptr_get_unsend();	// unsend frames left (incl. gaps)
      answer.head.len = 7;
    } else if (len==2) {		// enable/disable
      status_control = (status_control & 0xF0) | (rxdatapacket.data[PKT_PARAM_IDX] & 0x0F);
      pc_send_byte(ACK);
    } else {				// invalid - more than one parameter byte
      pc_send_byte(NAK);
    }
    break;
  case RPTR_GET_VERSION:
    answer.data[PKT_PARAM_IDX+0] = FIRMWAREVERSION & 0xFF;
    answer.data[PKT_PARAM_IDX+1] = FIRMWAREVERSION >> 8;
    memcpy(answer.data+PKT_PARAM_IDX+2, VERSION_IDENT, sizeof(VERSION_IDENT));
    answer.head.len = sizeof(VERSION_IDENT)+3-1;	// cut String-Terminator /0
    break;
  case RPTR_GET_SERIAL:
    memcpy(answer.data+PKT_PARAM_IDX, (void *)SERIALNUMBER_ADDRESS, 4);
    answer.head.len = 5;
    break;
  case RPTR_GET_CONFIG:
    if (len==1) {			// request all config
      char *nextblock = cfg_read_c0(answer.data+PKT_PARAM_IDX);
      //...
      answer.head.len = nextblock - answer.data - 3;
    } else if (len==2) {		// request a single block
      switch(rxdatapacket.data[PKT_PARAM_IDX]) {
      case 0xC0:
	cfg_read_c0(answer.data+PKT_PARAM_IDX);
	answer.head.len = sizeof(CONFIG_C0)+3;
	break;
      default:
	pc_send_byte(NAK);
	break;
      }
    } else {
      pc_send_byte(NAK);
    }
    break;
  case RPTR_SET_CONFIG:
    if (config_setup(rxdatapacket.data+PKT_PARAM_IDX, len-1)) {
      pc_send_byte(ACK);
    } else {
      pc_send_byte(NAK);
    }
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
#ifdef PLAIN_SLOWDATA		// scramble data (if was plain)
    dstar_scramble_data((tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+2]);
#endif
    // keep 2 bytes for future use, keep layout identical to RX
    rptr_addtxvoice((tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+2],
      rxdatapacket.data[PKT_PARAM_IDX+1]);
    break;
  case RPTR_EOT:		// end transmission with EOT tail
    rptr_endtransmit();
    break;
  case RPTR_SET_TESTMDE:
    if (len==2) {
      if (rxdatapacket.data[PKT_PARAM_IDX]&0x01) {
	if (RPTR_is_set(RPTR_RECEIVING)) {
	  pc_send_byte(NAK);
	} else {
	  RPTR_set(RPTR_TX_TESTLOOP);
          rptr_transmit();		// Turn on Xmitter
	  pc_send_byte(ACK);
	}
      } else {	// disable Test Mode
	RPTR_clear(RPTR_TX_TESTLOOP);
        pc_send_byte(ACK);
      }
    } else pc_send_byte(NAK);
    break;
  default:
    pc_send_byte(NAK);
    break;
  } // hctiws
  if ((answer.head.len > 0)&&(answer.head.len<(PAKETBUFFERSIZE-4))) {
    U32 anslen = answer.head.len+5;
    answer.head.id  = FRAMESTARTID;
    answer.head.cmd = 0x80|rxdatapacket.head.cmd;
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
      if (status_control&STA_CRCENABLE_MASK) {	// check CRC only, if needed.
	pkt_crc = crc_ccitt(rxdatapacket.data, framelen+5);
      } // fi check CRC
      if ((framelen <= (rxbytes-5)) && (pkt_crc==0)) {
	handle_pc_paket(framelen);
      } // fi correct / enough bytes received / crc ok
    } // fi correct ID
  } // fi was da
}


/* handle_hfdata() processes bit-flags from rptr_func
 * A typical reception look like this:
 *
 * D0 | 03 00 | 15 | 00 00 crc crc	>Preamble detected (not implemented jet)
 * D0 | 03 00 | 16 | 01 00 crc crc	>Start-Frame-Pattern detected
 * D0 | 2C 00 | 17 | 01 00 {header data} crc crc
 *                      ^ Biterrors in header (not implemented jet)
 * D0 | 0F 00 | 19 | 01 00 {VoiceData} 55 2D 16 crc crc > Voicedata with FrameSync
 *                      ^ pktcount 0 to 20 (defined by VoiceRxBufSize)
 * D0 | 0F 00 | 19 | 01 01 {VoiceData} {SlowData} crc crc > Slowdata not descrambled
 * ...
 * D0 | 03 00 | 1A | 01 00 crc crc	> End of Transmission
 *                   ^ transmission counter 1..255,0
 *              ^ cmd / type of paket ( = part of pkt-data )
 *      ^ Length of Data (packetlength-5)
 * ^ FramestartID
 */

void handle_hfdata(void) {
  static bool transmission = false;
  if (RPTR_Flags != 0) {
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
#ifdef PLAIN_SLOWDATA
      dstar_scramble_data(&voicedata);		// Unscramble 3byte slow data
#endif
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

  // *** initializing constant parts of I/O structures ***

  rptr_init_data();			// Default Header "noCall"
  init_pcdata();			// Initialisation of persitent Pkts (Header, Voice)

  Enable_global_interrupt();		// Enable all interrupts.
  LEDs_Off();

  set_dac_power_mode(TWI_DAC_POWERUP);	// Enable Reference-DAC MAX5820
  cfg_apply();				// setup default config

  rptr_receive();			// enable receiving.

  while (TRUE) {			// *** Main-Loop ***
    SLEEP();				// do nothing, until some IRQ wake the CPU
    watchdog();				// do external watchdog for STM706T

    handle_hfdata();			// look, if something to do with the HF interface

    usb_handler();			// handle USB-based reception and enumeration

    handle_pcdata();			// got a message / pkt from pc/gateway? handle it.

  } // ehliw MAINloop
}

// MAIN Ende

