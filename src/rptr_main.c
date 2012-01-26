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
 * 2011-09-22 V0.20  idle_timer feature, prevent TX, if not enabled. RX is swichable to.
 * 		     disabling RX+TX if USB cable a disconnected
 * 		     if TX switched off, DAC and REF-DAC turned to power-down
 * 2011-09-24 V0.30  add some bytes in header and voice-frame packets for future use.
 * 		     special-function commands RESET + FORCE-BOOTLOADER added
 * 2011-09-25 V0.30a Fixing header / voicedata bug (new offset)
 * 2011-09-29 V0.30b Fix a bug in usb_func() copy function
 * 2011-10-06 V0.31  using timeout of 5ms for incoming data (USB), flush incomplete pkts
 * 2011-10-07 V0.31a STA_CANDUPLEX_MASK > Bit6 in ConfigFlags indicates duplex mode possible
 * 2011-10-08 V0.40  add TX Status (enum see rptr_func.h) in GETSTATUS message
 * 		     new RSSI per frame feature returns a unsigned(16) value (4.85V = max = 1023).
 * 2011-10-11 V0.40a fixing memory overflow on weak signals, if a sync-pattern detected
 * 2011-10-19 V0.41  transmitting logic changed: see rptr_func.c
 * 2011-10-19 V0.42  Multi-Voice-Frame support
 * 2011-10-22 V0.43  EOT(TX) feature stops immediately, if buffer is empty (don't match buffer-pos)
 * 		     feature TESTLOOP removed, start TX with START message, wait until HEADER message
 * 2011-10-30 V0.44  conditional PATTERN checking, gmsk_func module now independent of D-Star related
 * 2011-11-05 V0.50  RC1; watchdog-feature, fixed to 30s, PREAMBLE-Detection-Logic
 * 2011-11-06 V1.00  same as RC1, changed standard config values (0.75Vss TXmod / 120ms TX-Delay
 * 2011-11-13 V1.10  first version with a simple TRX control
 * 2011-11-17 V1.10a a few changes allows Cfg-Applying with transmitting
 * 2011-12-28 V1.10b long-roger-beep bug fixed
 * 2012-01-05 V1.10c alternative #define SIMPLIFIED_FIFO to use a non-sorting Fifo of 420ms
 * 2012-01-11 V1.10d BugFix Pattern-Handling if PLL not correct an a to early match appears
 * 2012-01-22 V1.10e BugFix in controls.c module (dead carrier tx)
 * 2012-01-26 V1.20  THE LAST TRY: pic21mode.h, new reduced incompatible!!! interface
 *
 *
 *
 * ToDo:
 * - USB not working after reset cmd, maybe a USB-unplug msg is needed
 * + no new features for release V1.00
 * + first SYNC detect -> Message (early switch on Transceiver)
 * + PC watchdog
 * + RSSI / SQL sampling while receiving
 * + Transmitter-State enumeration for GET_STATUS
 * + force-bootloader command
 * + enable / disable receiver (if disabled keep firmware alive by a idle-counter)
 * + enable / disable transmitter (logic only, return NAK on START / SYNC cmd if off)
 * + bugfix transmit-logic
 * + configuration: TXDelay, Modulation-Vss ...
 */


#include "defines.h"		// general defines
#include "hw_defs.h"		// PTT-Macro
#include "dac_func.h"
#include "crc.h"
#include "pick21mode.h"		// handles incoming DATA packets with IcomUDP-like numbers
#include "rptr_func.h"		// realtime-handler part of HF I/O
#include "gmsk_func.h"		// configuration
#include "controls.h"
#include "transceiver.h"

#include "gpio_func.h"		// GPIO-functions and macros (PTT, LEDs...)
#include "int_func.h"		// idle_timer()
#include "flash_fuses.h"	// to force BL

#include "usb_func.h"

#include "intc.h"
#include "compiler.h"

#include <string.h>

#define PCWD_TIMEOUT	(30UL*MASTERCLOCK)	// 30 seconds

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
  unsigned char		rsvd1;		// for future use
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
  unsigned char		rsvd[2];	// for future use
  unsigned short	crc;
} tvoicepcdata;




typedef enum {
  RPTR_GET_STATUS=0x10, RPTR_GET_VERSION, RPTR_GET_SERIAL, RPTR_GET_CONFIG,
  RPTR_SET_CONFIG, RPTR_RXPREAMBLE, RPTR_START, RPTR_HEADER,
  RPTR_RXSYNC, RPTR_DATA, RPTR_EOT, RPTR_RXLOST,
  RPTR_MSG_RSVD1, RPTR_MSG_RSVD2, RPTR_MSG_RVSD3, // this message-types we can use for APCO P25
  RPTR_SET_SPECIALFUNCT		// 0x1F is used for special commands (bootloader, testmode...)
} tRPTRcmds;


#define STA_RXENABLE_MASK	0x01
#define STA_TXENABLE_MASK	0x02
#define STA_WDENABLE_MASK	0x04
#define STA_CRCENABLE_MASK	0x08
#define STA_IO21_STATE		0x10
#define STA_IO23_STATE		0x20
#define STA_CANDUPLEX_MASK	0x40
#define STA_NOCONFIG_MASK	0x80

#define STA_RECEIVING		0x01
#define STA_TRANSMITTING	0x02
#define STA_PCWATCHDOG		0x04


// *** global variables ***

U8		status_control  = STA_NOCONFIG_MASK|STA_CANDUPLEX_MASK;	// Holds persitent control-flags
U8		status_state;	// Holds state of RX/TX
//U8		current_txid;

U32		trx_capabilities;

U32		last_pc_activity;	// timestamp, updated every valid PC-packet handled

// functions used for pc/gateway communication:
// pre-initialization to USB-CDC, but easy reconfigurable to RS232
tdata_rx_fct	data_received	= cdc_received;
tdata_cpy_rx	data_copyrx	= cdc_copyblock;
tdata_tx_fct	data_transmit	= cdc_transmit;
tfunction	data_flushrx	= cdc_flushrx;
tdata_look_b	data_look_byte  = cdc_look_byte;
tdata_look_lew	data_look_word  = cdc_look_leword;


#define DATA_TIMEOUT_MS		5	// 5ms time gap allowed for incoming data

void data_timeout(int rx_length) {
  data_flushrx();		// simple flush buffer!
}


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
    750 * 256 / V_Ref,		// modulation voltage peak-peak ~ 0.75V
    GMSK_STDTXDELAY<<8		// little endian! Std-Value 120ms
};



__inline void pc_send_byte(U8 data) {
  answer.head.len = 2;
  answer.data[PKT_PARAM_IDX] = data;
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

// *** configuration ***
/*! \name MAIN configuration routines
 */
//! @{

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
  if ((status_control & STA_NOCONFIG_MASK)||(!RPTR_is_set(RPTR_TRANSMITTING))) {
    //if (current_txid!=0xFE) current_txid = 0xFF;
  }
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
    case 0XC1:
      trx_standby();
      if (block_len==CONFIG_C1_SIZE) cfg_write_c1(config_data+2); else return false;
      if (status_control & STA_RXENABLE_MASK) trx_receive();	// Enable receiving...
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

//! @}


// *** special function command set ***
/*! \name MAIN special functions PC handle
 */
//! @{

#define	SFC_TRX_CAPABILITIES	0xc1

#define SFC_GET_CURR_RSSI	0x08

#define SFC_CORRECT_TX_CLOCK	0x10

#define SFC_TWI_ACCESS		0x20
#define SFC_TWI_WRITE		0x21
#define SFC_TWI_READ		0x22	// no subadr
#define SFC_TWI_READREG		0x23	// 1 Byte "register" address
#define SFC_TWI_READMEM		0x24	// 2 Byte address

#define SFC_RESET		0xF1
#define SFC_FORCE_BOOTLOADER	0xF2


#include "twi_func.h"

void sfc_twi_wrt_return(tTWIresult res, unsigned int len) {
  // nicht sch�n aber selten
  if (res == TWIok) {
    answer.data[PKT_PARAM_IDX] = SFC_TWI_WRITE;
    answer.data[PKT_PARAM_IDX+1] = ACK;
    answer.head.len = 3;
} else {
    answer.data[PKT_PARAM_IDX] = SFC_TWI_WRITE;
    answer.data[PKT_PARAM_IDX+1] = NAK;
    answer.data[PKT_PARAM_IDX+2] = (char)res;
    answer.data[PKT_PARAM_IDX+3] = len & 0xFF;
    answer.data[PKT_PARAM_IDX+4] = (len >> 8) & 0xFF;
    answer.head.len = 6;
  }
  if ((answer.head.len > 0)&&(answer.head.len<(PAKETBUFFERSIZE-4))) {
    U32 anslen = answer.head.len+5;
    answer.head.id  = FRAMESTARTID;
    answer.head.cmd = 0x80|rxdatapacket.head.cmd;
    answer.head.len = swap16(answer.head.len);
    append_crc_ccitt(answer.data, anslen);
    data_transmit(answer.data, anslen);
  } // fi send

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



// *** handle PC (PCP2) packets ***
/*! \name MAIN PCP2 packet handling
 */
//! @{


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
      answer.head.len = 7;
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
      char *nextblock = cfg_read_c0(answer.data+PKT_PARAM_IDX);
      nextblock = cfg_read_c1(nextblock);
      //...
      answer.head.len = nextblock - answer.data - 3;
    } else if (len==2) {		// request a single block
      switch(rxdatapacket.data[PKT_PARAM_IDX]) {
      case 0xC0:
	cfg_read_c0(answer.data+PKT_PARAM_IDX);
	answer.head.len = sizeof(CONFIG_C0)+3;
	break;
      case 0xC1:
	cfg_read_c1(answer.data+PKT_PARAM_IDX);
 	answer.head.len = CONFIG_C1_SIZE+3;
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
    // Ignore
    break;
  case RPTR_HEADER:		// start transmitting TXDelay-Preamble-Start-Header
    if (rptr_tx_state != RPTRTX_header)	// update only, if not transmitting just this moment
      rptr_init_header((tds_header *)&rxdatapacket.data[PKT_PARAM_IDX+4]);
    if (status_control & STA_TXENABLE_MASK) {
      if (rptr_tx_state <= RPTRTX_preamble) {
        rptr_transmit();		// Turn on Xmitter
      } // fi
      // -> ignore a HEADER msg with same TXID while sending
      add_icom_voice_reset();
    } else
      pc_send_byte(NAK);
    // keep 2 bytes for future use, keep layout identical to RX
    break;
  case RPTR_RXSYNC:		// start transmitting TXDelay-Preamble-Start-Header
    // Ignore
    break;
  case RPTR_DATA:		// transmit data (voice and slowdata or sync)
    if (status_control & STA_TXENABLE_MASK) {
      if (rptr_tx_state <= RPTRTX_preamble) {
         rptr_transmit();		// Turn on Xmitter
         add_icom_voice_reset();
      } // fi
    }
    add_icom_voice_2_rptr(rxdatapacket.data[PKT_PARAM_IDX+1], &rxdatapacket.data[PKT_PARAM_IDX+4]);
    break;
  case RPTR_EOT:		// end transmission with EOT tail
    // ignore
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

//! @}


// *** handle HF requests / RPTR_flag ***
/*! \name MAIN HF RPTR flag handling
 */
//! @{

// handle_pcdata() testet, ob im Receive-Buffer Paket-Daten (auch unvollst.) liegen
// und kopiert diese nach 'rxdatapaket' um.
// Bei g�ltigen Paketen, wird 'handle_pc_paket()'
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
  if (RPTR_is_set(RPTR_INDICATOR_MASK)) {
    if (RPTR_is_set(RPTR_RX_1STPREAMBLE)) {
      RPTR_clear(RPTR_RX_1STPREAMBLE);
      // no sending to PC
    }
    if (RPTR_is_set(RPTR_RX_FRAMESYNC)) {
      RPTR_clear(RPTR_RX_FRAMESYNC);
      if (!transmission) {
        ctrldata.head.cmd = RPTR_RXSYNC;
        ctrldata.rxid++;
        headerdata.rxid = ctrldata.rxid;
        voicedata.rxid  = ctrldata.rxid;
        // no sending to PC
        transmission = true;
      }
    }
    if (RPTR_is_set(RPTR_RX_FRAME)) {
      RPTR_clear(RPTR_RX_FRAME);
      voicedata.pktcount = rptr_copycurrentrxvoice(&voicedata.DVdata);
      voicedata.rssi     = swap16(gmsk_get_rssi_avrge());
      append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
      data_transmit((char *)&voicedata, sizeof(voicedata));
    } // fi voice data
    if (RPTR_is_set(RPTR_RX_START)) {
      RPTR_clear(RPTR_RX_START);
      ctrldata.head.cmd = RPTR_START;
      ctrldata.rxid++;
      headerdata.rxid = ctrldata.rxid;
      voicedata.rxid =  ctrldata.rxid;
      // no sending to PC
      transmission = true;
    } // fi start detected
    if ( RPTR_is_set(RPTR_RX_STOP) || RPTR_is_set(RPTR_RX_LOST) ) {
      RPTR_clear(RPTR_RX_STOP|RPTR_RX_LOST);
      voicedata.pktcount = 0x40;
      voicedata.rssi     = swap16(gmsk_get_rssi_avrge());
      append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
      data_transmit((char *)&voicedata, sizeof(voicedata));
      transmission = false;
    } // fi stop detected
    if (RPTR_is_set(RPTR_RX_HEADER)) {
      RPTR_clear(RPTR_RX_HEADER);
      if (!transmission) {
        ctrldata.rxid++;
        headerdata.rxid = ctrldata.rxid;
        voicedata.rxid  = ctrldata.rxid;
        transmission    = true;
      } // fi no tx
      headerdata.biterrs = dstar_decodeheader(&headerdata.header, rptr_getheader());
      // Checking header-crc:
      headerdata.flags = 0x00;
      if (!dstar_checkheader(&headerdata.header)) {
	headerdata.flags |= 0x80;	// if corrupt set Bit7 of flags
	rptr_forcefirstsync();
      } // fi crc
      append_crc_ccitt((char *)&headerdata, sizeof(headerdata));
      data_transmit((char *)&headerdata, sizeof(headerdata));
    } // fi start detected
  } // fi do something
  // Testet, ob was emfangen wurde.
  // sendet sofort an pc
}

//! @}



void init_pcdata(void) {
  ctrldata.head.id  = FRAMESTARTID;
  ctrldata.head.len = swap16(sizeof(ctrldata)-5);
  ctrldata.rxid = 0;
  headerdata.head.id  = FRAMESTARTID;
  headerdata.head.len = swap16(sizeof(headerdata)-5);
  headerdata.head.cmd = RPTR_HEADER;
  headerdata.rsvd1    = 0x00;
  headerdata.rsvd2    = 0x00;
  voicedata.head.id   = FRAMESTARTID;
  voicedata.head.len  = swap16(sizeof(voicedata)-5);
  voicedata.head.cmd  = RPTR_DATA;
  voicedata.rsvd[0]   = 0x00;
  voicedata.rsvd[1]   = 0x00;
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


void rptr_reset_inferface(void) {	// usb disconneced
  if (data_received == cdc_received) {	// we use USB CDC interface
    rptr_standby();
  } // fi CDC used
}


#define TIMER_CMR_TEST_VALUE	( AVR32_TC_NONE << AVR32_TC_BSWTRG_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_BEEVT_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_BCPC_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_BCPB_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_ASWTRG_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_AEEVT_OFFSET | \
    2 << AVR32_TC_ACPC_OFFSET | \
    1 << AVR32_TC_ACPA_OFFSET | \
    1 << AVR32_TC_WAVE_OFFSET | \
    AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET | \
    FALSE << AVR32_TC_ENETRG_OFFSET | \
    AVR32_TC_EEVT_TIOB_INPUT << AVR32_TC_EEVT_OFFSET | \
    AVR32_TC_EEVTEDG_NO_EDGE << AVR32_TC_EEVTEDG_OFFSET | \
    FALSE << AVR32_TC_CPCDIS_OFFSET | \
    FALSE << AVR32_TC_CPCSTOP_OFFSET | \
    AVR32_TC_BURST_NOT_GATED << AVR32_TC_BURST_OFFSET | \
    0 << AVR32_TC_CLKI_OFFSET | \
    AVR32_TC_TCCLKS_TIMER_CLOCK2 << AVR32_TC_TCCLKS_OFFSET )


int main(void) {
  Disable_global_interrupt();		// Disable all interrupts.

  // *** Initialization-Section ***
  init_hardware();

#ifdef TC_A1_OUT
  // Programming a 15MHz Signal on PA21 (Pin45 µC / Pin15 I/O connector)
  AVR32_TC.channel[1].cmr = TIMER_CMR_TEST_VALUE;
  AVR32_TC.channel[1].rc  = 2;
  AVR32_TC.channel[1].ra  = 1;
  AVR32_TC.channel[1].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
  while(1) {
    SLEEP();				// do nothing, until some IRQ wake the CPU
    watchdog();				// do external watchdog for STM706T
  }
#endif

  INTC_init_interrupts();		// Initialize interrupt vectors.
  rptr_init_hardware();
  usb_init();				// Enable VBUS-Check

//  cdc_enabletimeout(data_timeout, DATA_TIMEOUT_MS);

  // *** initializing constant parts of I/O structures ***

  rptr_init_data();			// Default Header "noCall"
  init_pcdata();			// Initialisation of persitent Pkts (Header, Voice)

  Enable_global_interrupt();		// Enable all interrupts.
  LEDs_Off();

  cfg_apply();				// setup default config

  idle_timer_start();			// keep µC alive to handle external WD, if
  // no other activity (USB)

  trx_capabilities = trx_init();	// init optional TRX module

  //set_dac_power_mode(TWI_DAC_POWERUP);
  // Enable Reference-DAC MAX5820 (called, if TX swichted on)
  //rptr_receive();
  // enable receiving (called, if receiver is switched on)

  while (TRUE) {			// *** Main-Loop ***

    SLEEP();				// do nothing, until some IRQ wake the CPU

    watchdog();				// do external watchdog for STM706T
    if (status_control&STA_WDENABLE_MASK) {	// active
      if ( (Get_system_register(AVR32_COUNT)-last_pc_activity) > PCWD_TIMEOUT) {
	rptr_standby();
      }
    } // fi do a PC-RX based watchdog

    handle_hfdata();			// look, if something to do with the HF interface

    usb_handler();			// handle USB-based reception and enumeration

    handle_pcdata();			// got a message / pkt from pc/gateway? handle it.

  } // ehliw MAINloop
}

// MAIN Ende

