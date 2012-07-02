/*
 * stdmodem.c
 *
 *  Created on: 22.05.2012
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
 * 2012-05-22	created (copied from rptr_main.c)
 * 2012-06-18	change sequence of FLAG handling in handle_hfdata()
 * 		(DGL_HEADER and DGL_FRAME appears simoutanously)
 *
 */


#include "stdmodem.h"
#include "defines.h"		// general defines
#include "hw_defs.h"		// pins

#include "compiler.h"
#include "crc.h"

#include "gpio_func.h"		// GPIO-functions and macros (PTT, LEDs...)
#include "int_func.h"		// idle_timer()
#include "dac_func.h"
#include "twi_func.h"
#include "gmsk_func.h"
#include "usb_func.h"
#include "rptr_func.h"		// realtime-handler part of HF I/O

#include "controls.h"
#include "transceiver.h"
#include "dongle.h"
#include "TLV320AIC.h"		// config function analog frontend
#include "flash_fuses.h"	// to force BL
#include "storage.h"		// load/save eeprom data high level functions

#include "config.h"

#include <string.h>


typedef enum {
  RPTR_GET_STATUS=0x10, RPTR_GET_VERSION, RPTR_GET_SERIAL, RPTR_GET_CONFIG,
  RPTR_SET_CONFIG, RPTR_RXPREAMBLE, RPTR_START, RPTR_HEADER,
  RPTR_RXSYNC, RPTR_DATA, RPTR_EOT, RPTR_RXLOST,
  RPTR_MSG_RSVD1, RPTR_MSG_RSVD2, RPTR_MSG_RVSD3, // this message-types we can use for APCO P25
  RPTR_SET_SPECIALFUNCT		// 0x1F is used for special commands (bootloader, testmode...)
} tRPTRcmds;




typedef int (*tdata_rx_fct)(void);		// Data-Received function (return #bytes)
typedef int (*tdata_cpy_rx)(char *, int);	// Copy received data
typedef void (*tdata_tx_fct)(const char *, int); // Data-Transmit function
typedef unsigned char (*tdata_look_b)(int);	// Data-Look-Byte function
typedef unsigned short (*tdata_look_lew)(int);	// Data-Look-Word function (Little Endian)


typedef struct PACKED_DATA {
  tRS232pktHeader	head;
  unsigned char		rxid;
  unsigned char		rsvd;
  unsigned short	crc;
} tctrlpcdata;

typedef struct PACKED_DATA {
  tRS232pktHeader	head;		// 4 byte (ID, length and first payload = message-type)
  unsigned char		rxid;		// ID of current transmission (alter it, when start a new one)
  unsigned char		flags;		// control-flags
  unsigned char		biterrs;	// no of bit-errors in a received header (not jet)
  unsigned char		srcflags;	// source flags, bit1 = internal by AMBEaddon
  tds_header		header;		// header starts at offset +8! 41 bytes long
  unsigned char		rsvd2;		// for future use
  unsigned short	crc;
} theadrpcdata;

typedef struct PACKED_DATA {
  tRS232pktHeader	head;		// 4 byte (ID, length and first payload = message-type)
  unsigned char		rxid;		// ID of current transmission
  unsigned char		pktcount;
  unsigned short	rssi;		// SQL-line value (averaged for every voice frame)
  tds_voicedata		DVdata;		// DVdata starts at offset +8!
  unsigned char		srcflags;	// source flags, bit1 = internal by AMBEaddon
  unsigned char		rsvd;		// for future use
  unsigned short	crc;
} tvoicepcdata;


// *** global variables ***

U8		status_control  = STA_NOCONFIG_MASK|STA_CANDUPLEX_MASK;	// Holds persitent control-flags
U8		status_state;	// Holds state of RX/TX
U8		current_txid;
U32		last_pc_activity;	// timestamp, updated every valid PC-packet handled

extern U32	trx_capabilities;
extern U32	dgl_capabilities;



// functions used for pc/gateway communication:
// pre-initialization to USB-CDC, but easy reconfigurable to RS232
tdata_rx_fct	data_received	= cdc_received;
tdata_cpy_rx	data_copyrx	= cdc_copyblock;
tdata_tx_fct	data_transmit	= cdc_transmit;
tfunction	data_flushrx	= cdc_flushrx;
tdata_look_b	data_look_byte  = cdc_look_byte;
tdata_look_lew	data_look_word  = cdc_look_leword;


tRS232packet	rxdatapacket;	// data from pc/gateway received
tRS232packet	answer;		// data to reply (get cmds)
tTINYpacket	async_reply;	// various replies created by a handler function

tctrlpcdata	ctrldata;
theadrpcdata	headerdata;
tvoicepcdata	voicedata;	// Voice+Slowdata from HF



void init_modemdata(void) {
  ctrldata.head.id  = FRAMESTARTID;
  ctrldata.head.len = swap16(sizeof(ctrldata)-5);
  ctrldata.rxid = 0;
  headerdata.head.id  = FRAMESTARTID;
  headerdata.head.len = swap16(sizeof(headerdata)-5);
  headerdata.head.cmd = RPTR_HEADER;
  headerdata.srcflags = 0x00;
  headerdata.rsvd2    = 0x00;
  voicedata.head.id   = FRAMESTARTID;
  voicedata.head.len  = swap16(sizeof(voicedata)-5);
  voicedata.head.cmd  = RPTR_DATA;
  voicedata.srcflags  = 0x00;
  voicedata.rsvd      = 0x00;
  async_reply.head.id  = FRAMESTARTID;
}



void data_timeout(int rx_length) {
  data_flushrx();		// simple flush buffer!
}

/* rptr_reset_inferface() is called from USB-hander, if someone unplug the USB
 * in this case disable DV-RPTR (exeption: AMBE-addon used for HF)
 */
void rptr_reset_inferface(void) {	// usb disconneced
  if (data_received == cdc_received) {	// we use USB CDC interface
    if (!RPTR_is_set(RPTR_AMBEDECODEHF)) // not HF decode to hear it?
      rptr_standby();
    else
      status_control &= ~STA_WDENABLE_MASK;
  } // fi CDC used
}



void rptr_standby(void) {
  status_control &= 0xF0;		// Watchdog/TX and RX disabled
  trx_standby();
  idle_timer_start();			// disable receiving
  // Disable Reference-DAC MAX5820
  set_dac_power_mode(TWI_DAC_POWERDOWN);
  // Disable selected output of modulation DAC
  dac_power_ctrl(0);
}



__inline void pc_send_byte(U8 data) {
  answer.head.len = 2;			// simple function to generate a ACK/NAK reply
  answer.data[PKT_PARAM_IDX] = data;	// in buffer 'answer'
}


__inline void sfc_create_reply(char sfc, char code) {
  async_reply.head.id = FRAMESTARTID;
  async_reply.head.len = swap16(3);
  async_reply.head.cmd = 0x80|RPTR_SET_SPECIALFUNCT;
  async_reply.param[0] = sfc;
  async_reply.param[1] = code;
}


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




// *** special function command set ***
/*! \name MAIN special functions PC handle
 */
//! @{

#define	SFC_TRX_CAPABILITIES	0xC1	// Transceiver Capabilities Readout
#define	SFC_DGL_CAPABILITIES	0xC2	// AMBE-addon presence / capabilities

#define SFC_GET_CURR_RSSI	0x08

#define SFC_CORRECT_TX_CLOCK	0x10

// 0x20 TWI direct access
#define SFC_TWI_ACCESS		0x20
#define SFC_TWI_WRITE		0x21
#define SFC_TWI_READ		0x22	// no subadr
#define SFC_TWI_READREG		0x23	// 1 Byte "register" address
#define SFC_TWI_READMEM		0x24	// 2 Byte address

// 0x40: Dongle Module
#define SFC_DGL_MANUALPTT	0x40	// set manual PTT on or off
#define SFC_DGL_VOLUMECTRL	0x41	// controls volume (selector + value 8bit)

#define SFC_DGL_ADCFILTER	0x44	// Load a FilterBlock


#define SFC_DEFAULT_CONFIGS	0x80	// load all stored configs from EEProm
#define SFC_STORE_CONFIG	0x81	// write a configuration from ram into EEProm

#define SFC_RESET		0xF1
#define SFC_FORCE_BOOTLOADER	0xF2




void sfc_twi_wrt_return(tTWIresult res, unsigned int len) {
  U32 anslen = 8;
  sfc_create_reply(SFC_TWI_WRITE, ACK);
  if (res != TWIok) {
    async_reply.head.len = swap16(6);
    async_reply.param[1] = NAK;
    async_reply.param[2] = (char)res;
    async_reply.param[3] = len & 0xFF;
    async_reply.param[4] = (len >> 8) & 0xFF;
    anslen = 11;
  }
  append_crc_ccitt(async_reply.data, anslen);
  data_transmit(async_reply.data, anslen);
}


// handle_special_func_cmd()
// a subset of commands / functions (not often used) are accessed by main-command 0x1F
// this function handles such calls from the PC.
void handle_special_func_cmd(int len) {
  S32 parameter;
  if (len < 2) {
    pc_send_byte(NAK);
  } else switch (rxdatapacket.data[PKT_PARAM_IDX]) {
  case SFC_TRX_CAPABILITIES:
    answer.head.len = 6;
    answer.data[PKT_PARAM_IDX]   = SFC_TRX_CAPABILITIES;
    answer.data[PKT_PARAM_IDX+1] = LSB0W(trx_capabilities);
    answer.data[PKT_PARAM_IDX+2] = LSB1W(trx_capabilities);
    answer.data[PKT_PARAM_IDX+3] = LSB2W(trx_capabilities);
    answer.data[PKT_PARAM_IDX+4] = LSB3W(trx_capabilities);
    break;
  case SFC_DGL_CAPABILITIES:
    answer.head.len = 6;
    answer.data[PKT_PARAM_IDX]   = SFC_DGL_CAPABILITIES;
    answer.data[PKT_PARAM_IDX+1] = LSB0W(dgl_capabilities);
    answer.data[PKT_PARAM_IDX+2] = LSB1W(dgl_capabilities);
    answer.data[PKT_PARAM_IDX+3] = LSB2W(dgl_capabilities);
    answer.data[PKT_PARAM_IDX+4] = LSB3W(dgl_capabilities);
    break;
  case SFC_CORRECT_TX_CLOCK:
    parameter = (rxdatapacket.data[PKT_PARAM_IDX+1]) | (rxdatapacket.data[PKT_PARAM_IDX+2]<<8) |
      (rxdatapacket.data[PKT_PARAM_IDX+3]<<16) | (rxdatapacket.data[PKT_PARAM_IDX+4]<<24);
    if ((len == 6) && (abs(parameter) < 0x1FFFF)) {
      gmsk_adjusttxperiod(parameter);
      pc_send_byte(ACK);
    } else pc_send_byte(NAK);
    break;
  case SFC_TWI_WRITE:
    if (len > 3)
      twi_write(rxdatapacket.data[PKT_PARAM_IDX+1], &rxdatapacket.data[PKT_PARAM_IDX+2], len-3, sfc_twi_wrt_return);
    break;
  case SFC_GET_CURR_RSSI:
    parameter = gmsk_get_rssi_current();
    answer.head.len = 4;
    answer.data[PKT_PARAM_IDX] = SFC_GET_CURR_RSSI;
    answer.data[PKT_PARAM_IDX+1] = parameter & 0xFF;
    answer.data[PKT_PARAM_IDX+2] = (parameter>>8) & 0xFF;
    break;
  case SFC_DGL_MANUALPTT:
    if (len == 3) {
      if (rxdatapacket.data[PKT_PARAM_IDX+1] & 0x01)
	dgl_start_transmit();
      else
	dgl_stop_transmit();
      pc_send_byte(ACK);
    } else pc_send_byte(NAK);
    break;
  case SFC_DGL_VOLUMECTRL:
    if ((dgl_capabilities) && (len==4)) {
      switch (rxdatapacket.data[PKT_PARAM_IDX+1]) {
      case 0:		// "MASTER" DAC-Volume
	tlv_set_DACvolume(rxdatapacket.data[PKT_PARAM_IDX+2]);
	break;
      case 1:		// headphone
	tlv_set_HSvolume(rxdatapacket.data[PKT_PARAM_IDX+2]);
	break;
      case 2:		// speaker
	tlv_set_SPKRvolume(rxdatapacket.data[PKT_PARAM_IDX+2]);
	break;
      case 3:		// microphone digital ADC gain (-12dB..20dB ^= -24..40 allowed)
	tlv_set_adcgain(rxdatapacket.data[PKT_PARAM_IDX+2]);
	break;
      default:
	pc_send_byte(NAK);
      } // hctiws selection
      if (answer.head.len == 0) pc_send_byte(ACK);
    } else pc_send_byte(NAK);
    break;
  case SFC_DGL_ADCFILTER:	// loads a FilterBlock into TLV320AIC
    if ((dgl_capabilities) && (len>=9)) {
      switch (rxdatapacket.data[PKT_PARAM_IDX+1]) {
      case 0:		// FirstOrder IIR
	if (len != 9)
	  pc_send_byte(NAK);
	else
	  tlvfilter_load_iir((const S16 *)&rxdatapacket.data[PKT_PARAM_IDX+2]);
	break;
      case 1:	// BiQuad A or FIR coeffs Fir0 to Fir4
      case 2:	// BiQuad B or FIR coeffs Fir5 to Fir9
      case 3:	// BiQuad C or FIR coeffs Fir10 to Fir14
      case 4:	// BiQuad D or FIR coeffs Fir15 to Fir19
      case 5:	// BiQuad E or FIR coeffs Fir20 to Fir24
	if (len != 13)
	  pc_send_byte(NAK);
	else
	  tlvfilter_load_bqfir(rxdatapacket.data[PKT_PARAM_IDX+1]-1,
	    (const S16 *)&rxdatapacket.data[PKT_PARAM_IDX+2]);
	break;
      default:
	pc_send_byte(NAK);
	break;
      } // hctiws kind/no of filter
    }
    break;
  case SFC_DEFAULT_CONFIGS:
    if (have_configs()) {
      load_configs_from_eeprom();
      pc_send_byte(ACK);
    } else pc_send_byte(NAK);
    break;
  case SFC_STORE_CONFIG:	// storing configurations
    if (len==2) {
      pc_send_byte((save_configs())?ACK:NAK);
    } // fi save all configs
    break;
  case SFC_FORCE_BOOTLOADER:
  case SFC_RESET:
    // to prevent unwanted calling - this commands need a parameter as a key:
    // send the SERIAL-NUMBER (32bit) as unique identifier.
    if ((len == 6) && (memcmp(&rxdatapacket.data[PKT_PARAM_IDX+1], (char *)SERIALNUMBER_ADDRESS, 4)==0))  {
      if (rxdatapacket.data[PKT_PARAM_IDX]==SFC_FORCE_BOOTLOADER) {
	SetForceISP();
      } // fi bootloader
      usb_exit();
      // ToDo after Reset the USB-CDC is not working - but USB-DFU is ok.
      rptr_exit_hardware();
      exit_usb_hardware();
      Disable_global_interrupt();
      __asm__ __volatile__ ("rjmp _trampoline");	// jump direct to reset-vector (reset-section)
    } else {
      pc_send_byte(NAK);
    }
    break;
  default:
    pc_send_byte(NAK);
  } // esle hctiws
}

//! @}



bool config_readout(unsigned char cfg_id) {
  char *nextblock = answer.data+PKT_PARAM_IDX;
  switch(cfg_id) {
  case 0x00:	// all available configs
    nextblock = cfg_read_c0(nextblock);
    if (trx_capabilities&TRXCAP_AVAIL)
      nextblock = cfg_read_c1(nextblock);
    if (dgl_capabilities&DONGLE_AVAIL) {
      nextblock = cfg_read_c2(nextblock);
      nextblock = cfg_read_c3(nextblock);
      nextblock = cfg_read_c4(nextblock);	// analog frontend
      nextblock = cfg_read_c5(nextblock);	// AGC+DRC
    }
    answer.head.len = nextblock - answer.data - 3;
    break;
  case 0xC0:
    cfg_read_c0(nextblock);
    answer.head.len = sizeof(t_config_0)+3;
    break;
  case 0xC1:
    cfg_read_c1(nextblock);
    answer.head.len = CONFIG_C1_SIZE+3;
    break;
  case 0xC2:
    cfg_read_c2(nextblock);
    answer.head.len = CONFIG_C2_SIZE+3;
    break;
  case 0xC3:
    cfg_read_c3(nextblock);
    answer.head.len = CONFIG_C3_SIZE+3;
    break;
  case 0xC4:
    cfg_read_c4(nextblock);
    answer.head.len = CONFIG_C4_SIZE+3;
    break;
  case 0xC5:
    cfg_read_c5(nextblock);
    answer.head.len = CONFIG_C5_SIZE+3;
    break;
  case 0xC6:
    cfg_read_c6(nextblock);
    answer.head.len = CONFIG_C6_SIZE+3;
    break;
  default:
    return false;
  } // hctiws id
  return true;
}



// *** handle PC (PCP2) packets ***
/*! \name MAIN PCP2 packet handling
 */
//! @{

__inline void add_multi_voice_2_rptr(int len) {
  unsigned int pkt_cnt = (len-7) / sizeof(tds_voicedata);
  if (pkt_cnt == 1) {		// normal case - handle separated / optimized
#ifdef PLAIN_SLOWDATA		// scramble data (if was plain)
    dstar_scramble_data((tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+4]);
#endif
    // keep 2 bytes for future use, keep layout identical to RX
    rptr_addtxvoice((tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+4],
      rxdatapacket.data[PKT_PARAM_IDX+1]);
  } else if ((pkt_cnt > 1) && (pkt_cnt <= DSTAR_SYNCINTERVAL)) {
    unsigned char pktnr = rxdatapacket.data[PKT_PARAM_IDX+1];
    tds_voicedata *voicedata = (tds_voicedata *)&rxdatapacket.data[PKT_PARAM_IDX+4];
    for ( ; pkt_cnt > 0; pkt_cnt--) {
  #ifdef PLAIN_SLOWDATA		// scramble data (if was plain)
      dstar_scramble_data(voicedata);
  #endif
      rptr_addtxvoice(voicedata, pktnr);
      voicedata++;
      pktnr = (pktnr+1) % VoiceTxBufSize;
    } // rof
  } else pc_send_byte(NAK);
}


// handle_serial_paket()
// verarbeitet "paket" mit len optionalen Daten (Header NICHT mitgerechnet).
__inline void handle_pc_paket(int len) {
  answer.head.len = 0;			// no answer.
  last_pc_activity = Get_system_register(AVR32_COUNT);
  switch (rxdatapacket.head.cmd) {	// Kommando-Byte
  case RPTR_GET_STATUS:
    if (len==1) {			// request
      update_status();
      answer.data[PKT_PARAM_IDX+0] = status_control;
      answer.data[PKT_PARAM_IDX+1] = status_state;
      answer.data[PKT_PARAM_IDX+2] = rptr_tx_state;
      answer.data[PKT_PARAM_IDX+3] = VoiceRxBufSize;
      answer.data[PKT_PARAM_IDX+4] = VoiceTxBufSize;
      answer.data[PKT_PARAM_IDX+5] = rptr_get_unsend();	// unsend frames left (incl. gaps)
      answer.data[PKT_PARAM_IDX+6] = rptr_get_txpos();
      answer.head.len = 8;
    } else if (len==2) {		// enable/disable
      U8 new_control = (status_control & 0xF0) | (rxdatapacket.data[PKT_PARAM_IDX] & 0x0F);
      if ((new_control^status_control) & STA_RXENABLE_MASK) {
	if (new_control & STA_RXENABLE_MASK) {
	  trx_receive();			// set receiving
	  rptr_receive();			// enable receiving
	} else {
	  idle_timer_start();			// disable receiving
	}
      } // fi sw RX
      if ( (new_control^status_control) & STA_TXENABLE_MASK) {
        // Enable / Disable selected output of modulation DAC
        dac_power_ctrl(new_control&STA_TXENABLE_MASK);
	// Enable / Disable Reference-DAC MAX5820
        set_dac_power_mode((new_control&STA_TXENABLE_MASK)?TWI_DAC_POWERUP:TWI_DAC_POWERDOWN);
      } // fi sw TX
      status_control = new_control;
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
      if (!config_readout(0x00))
	pc_send_byte(NAK);
    } else if (len==2) {		// request a single block
      if (!config_readout(rxdatapacket.data[PKT_PARAM_IDX]))
	pc_send_byte(NAK);
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
    if ((status_control & STA_TXENABLE_MASK) && (len==3)) {
      if (rxdatapacket.data[PKT_PARAM_IDX] != current_txid) {
	rptr_transmit_preamble(); // PTTon, wait for a header to start TX
	if (rptr_tx_state <= RPTRTX_preamble)	// starts w/o interrupting a running transmission
	  current_txid = rxdatapacket.data[PKT_PARAM_IDX];
      }
    } else
      pc_send_byte(NAK);
    break;
  case RPTR_HEADER:		// start transmitting TXDelay-Preamble-Start-Header
    if (rptr_tx_state != RPTRTX_header)	// update only, if not transmitting just this moment
      rptr_init_header((tds_header *)&rxdatapacket.data[PKT_PARAM_IDX+4]);
    if ((status_control & STA_TXENABLE_MASK) && (!dgl_is_encoding())) {
      if ((rptr_tx_state <= RPTRTX_preamble) || (rxdatapacket.data[PKT_PARAM_IDX] != current_txid)) {
        rptr_transmit();		// Turn on Xmitter
        current_txid = rxdatapacket.data[PKT_PARAM_IDX];
      } // fi
      // -> ignore a HEADER msg with same TXID while sending
      //add_icom_voice_reset();
    } else
      pc_send_byte(NAK);
    // keep 2 bytes for future use, keep layout identical to RX
    break;
  case RPTR_RXSYNC:		// start transmitting TXDelay-Preamble-Start-Header
    if (rptr_tx_state != RPTRTX_header)	// update only, if not transmitting just this moment
      rptr_replacement_header();
    if ((status_control & STA_TXENABLE_MASK) && (!dgl_is_encoding())) {
      if ((rptr_tx_state <= RPTRTX_preamble) || (rxdatapacket.data[PKT_PARAM_IDX] != current_txid)) {
	rptr_transmit();	// Turn on Xmitter only if PTT off
	current_txid = rxdatapacket.data[PKT_PARAM_IDX];
      } // if idle
      // (transmission use last header)
      //add_icom_voice_reset();
    } else
      pc_send_byte(NAK);
    break;
  case RPTR_DATA:		// transmit data (voice and slowdata or sync)
    if (rxdatapacket.data[PKT_PARAM_IDX] == current_txid)
      add_multi_voice_2_rptr(len);
    break;
  case RPTR_EOT:		// end transmission with EOT tail
    if (len == 3) {
      if (rxdatapacket.data[PKT_PARAM_IDX] == current_txid)
        rptr_endtransmit(rxdatapacket.data[PKT_PARAM_IDX+1]);
    } else
      rptr_endtransmit(0xFF);	// stop after buffer is empty
    break;
  case RPTR_SET_SPECIALFUNCT:
    handle_special_func_cmd(len);
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
  int rxbytes, burst_cnt = 5;
  do {
    rxbytes = data_received();
    if (rxbytes > 4) {					// a frame shout be have D0 length and crc
      U16 length = 0;
      if (data_look_byte(0) != FRAMESTARTID) {
	data_flushrx();		// destroy garbage on COM/USB
      } else {
	length = data_look_word(1);
	if (length <= (sizeof(rxdatapacket)-5)) {
	  if (length <= (rxbytes-5)) {
	    data_copyrx(rxdatapacket.data, length+5);
	    // data_flushrx();	// needed for buggy software, obsolete - using timeout
	  } else					// packet still in receiving (on slow serial)
	    length = 0;
	} else {
	  data_flushrx();				// incorrect packet
	  length = 0;				// remove all data from buffer
	} // fi incomplete
      } // esle
      if (length > 0) {
        // packet begins with a valid frame / FRAMESTARTID checked before
	U16 pkt_crc = 0;
	if (status_control&STA_CRCENABLE_MASK) {	// check CRC only, if needed.
	  pkt_crc = crc_ccitt(rxdatapacket.data, length+5);
	} // fi check CRC
	if (pkt_crc==0) handle_pc_paket(length);	// crc correct
      } // fi payload
    } // fi was da
  } while ( (rxbytes > 4) && ((--burst_cnt)>0) );

  // PC communication watchdog:
  if (status_control&STA_WDENABLE_MASK) {	// active
    if ( (Get_system_register(AVR32_COUNT)-last_pc_activity) > PCWD_TIMEOUT) {
	rptr_standby();
    }
  } // fi do a PC-RX based watchdog
}

//! @}




// *** handle HF requests / RPTR_flag ***
/*! \name MAIN HF RPTR flag handling
 */
//! @{

/* handle_hfdata() processes bit-flags from rptr_func

 */

void handle_hfdata(void) {
  static bool transmission = false;
  if (RPTR_is_set(RPTR_INDICATOR_MASK)) {

    if (RPTR_is_set(RPTR_RX_1STPREAMBLE)) {
      RPTR_clear(RPTR_RX_1STPREAMBLE);
      ctrldata.head.cmd = RPTR_RXPREAMBLE;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
    } // fi preamble 01010... detected

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

    if (RPTR_is_set(RPTR_RX_FRAMESYNC)) {
      RPTR_clear(RPTR_RX_FRAMESYNC);
      if (!transmission) {
        ctrldata.head.cmd = RPTR_RXSYNC;
        ctrldata.rxid++;
        headerdata.rxid = ctrldata.rxid;
        voicedata.rxid  = ctrldata.rxid;
        append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
        data_transmit((char *)&ctrldata, sizeof(ctrldata));
        transmission = true;
      }
    } // fi framesync-start

    // *** HEADER received ***
    if (RPTR_is_set(DGL_HEADER)) {
      RPTR_clear(DGL_HEADER);
      if ((status_control&STA_NOCONFIG_MASK)==0) {
	ctrldata.rxid++;
	headerdata.rxid = ctrldata.rxid;
	voicedata.rxid  = ctrldata.rxid;
	transmission    = true;
	headerdata.biterrs = 0;
	headerdata.srcflags |= 0x01;
	dgl_get_header(&headerdata.header);
	append_crc_ccitt((char *)&headerdata, sizeof(headerdata));
	data_transmit((char *)&headerdata, sizeof(headerdata));
      }
    } else if (RPTR_is_set(RPTR_RX_HEADER)) {	// fi DONGLE starts with header
      RPTR_clear(RPTR_RX_HEADER);
      if (!transmission) {
        ctrldata.rxid++;
        headerdata.rxid = ctrldata.rxid;
        voicedata.rxid  = ctrldata.rxid;
        transmission    = true;
      } // fi no tx
      headerdata.biterrs = dstar_decodeheader(&headerdata.header, rptr_getheader());
      headerdata.srcflags &= ~0x01;
      // Checking header-crc:
      headerdata.flags = 0x00;
      if (dstar_checkheader(&headerdata.header)) {
	// Header seems to be ok:
	if (RPTR_is_set(RPTR_AMBEDECODEHF)) dgl_ProcessHdr(&headerdata.header);
      } else {
	headerdata.flags |= 0x80;	// if corrupt set Bit7 of flags
	rptr_forcefirstsync();
      } // fi crc
      append_crc_ccitt((char *)&headerdata, sizeof(headerdata));
      data_transmit((char *)&headerdata, sizeof(headerdata));
    } // fi header received


    // *** VOICEFRAME received or generated by AMBE-2020 ***
    if (RPTR_is_set(DGL_FRAME)) {
      RPTR_clear(DGL_FRAME);
      if ((status_control&STA_NOCONFIG_MASK)==0) {
	voicedata.pktcount = dgl_copyvoice(&voicedata.DVdata);
	voicedata.rssi     = 0;
	voicedata.srcflags |= 0x01;
	append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
	data_transmit((char *)&voicedata, sizeof(voicedata));
      }
    } else if (RPTR_is_set(RPTR_RX_FRAME)) {	// fi dongle voice data
      RPTR_clear(RPTR_RX_FRAME);
      voicedata.pktcount = rptr_copycurrentrxvoice(&voicedata.DVdata);
      voicedata.rssi     = swap16(gmsk_get_rssi_avrge());
      voicedata.srcflags &= ~0x01;
      append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
      data_transmit((char *)&voicedata, sizeof(voicedata));
    } // fi voice data

    // *** ENDING HF Stream Flags ***
    if (RPTR_is_set(RPTR_RX_STOP|DGL_EOT)) {
      RPTR_clear(RPTR_RX_STOP|DGL_EOT);
      if ((status_control&STA_NOCONFIG_MASK)==0) {
      ctrldata.head.cmd = RPTR_EOT;
      ctrldata.rsvd     = voicedata.pktcount;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
      transmission = false;
      }
    } // fi EOT detected
    if (RPTR_is_set(RPTR_RX_LOST)) {
      RPTR_clear(RPTR_RX_LOST);
      ctrldata.head.cmd = RPTR_RXLOST;
      ctrldata.rsvd     = voicedata.pktcount;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
      transmission = false;
    } // fi start detected

  } // fi do something
  // Testet, ob was emfangen wurde.
  // sendet sofort an pc
}

//! @}

