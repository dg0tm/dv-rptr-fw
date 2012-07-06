/*
 * rptr_func.h
 *
 * realtime controlling openDV (D-Star) operation of DV-RPTR
 *
 *  Created on: 26.07.2011 from opendv_func
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef DSTAR_FUNC_H_
#define DSTAR_FUNC_H_

#include "dv_dstar.h"

#ifdef SIMPLIFIED_FIFO
  // simplified Fifo buffer behavior, as requested by DG1HT
#define	VoiceTxBufSize	DSTAR_SYNCINTERVAL		// Size of VoiceBuf
#else
#define	VoiceTxBufSize	(12*DSTAR_SYNCINTERVAL)		// Size of VoiceBuf
#endif

#define VoiceRxBufSize	DSTAR_SYNCINTERVAL		// Fixed to one sync


typedef enum {
  RPTRTX_disabled, RPTRTX_idle, RPTRTX_txdelay, RPTRTX_preamble,
  RPTRTX_header, RPTRTX_voicedata, RPTRTX_lastframe, RPTRTX_eot
} trptr_tx_state;


extern unsigned int RPTR_Flags;
// Receive Flags (checked in handle_hfdata()) Bit 0..8
#define RPTR_RX_START		0x0001
#define RPTR_RX_STOP		0x0002
#define RPTR_RX_LOST		0x0004
#define RPTR_RX_FRAMESYNC	0x0008	// first detection of a preamble pattern
#define RPTR_RX_FRAME		0x0010
#define RPTR_RX_HEADER		0x0020
#define RPTR_RX_PREAMBLE	0x0040	// permanent indicator, don't clear flag
#define RPTR_RECEIVING		0x0080	// permanent indicator, don't clear flag

#define RPTR_RX_1STPREAMBLE	0x0100	// first preamble detected

// Transmit Flags Bit 9..15
#define RPTR_TX_EMPTY		0x0200

#define DGL_FRAME		0x0400
#define DGL_HEADER		0x0800
#define DGL_EOT			0x1000

#define RPTR_AMBEDECODEHF	0x2000	// active, if decoding data from receiver (perm)
#define RPTR_AMBEDECODEINET	0x4000	// active, if decoding data from internet (perm)

#define RPTR_TRANSMITTING	0x8000	// permanent indicator, don't clear flag

#define RPTR_RX_AUTOINVERS	0x010000
#define RPTR_HALFDUPLEX		0x020000

#define RPTR_INDICATOR_MASK	0x1F3F	// all w/o permanents


#define RPTR_is_set(f)		(RPTR_Flags&(f))
#define RPTR_set(f)		(RPTR_Flags |= f)
#define RPTR_clear(f)		(RPTR_Flags &= ~(f))

extern trptr_tx_state 		rptr_tx_state;


// maximum receive packets, if no sync-frame received:
#define RPTR_MAX_PKT_WO_SYNC	64	// norm 50! Test

#define RPTR_MAXLEN_RXPREAMBLE	1200	// 250ms - holds PREAMBLE-FLAG until START/FRAME-SYNC or
				        // this value is reached.

#define DSTAR_BEFOREFRAMEENDS	16	// 16 bit-time before gmsk-modulator-data runs out
// attention: this value must be smaller than 32 (last 32bit must be alreay loaded in GMSK fct)

#define DSTAR_DATAIDLETIMEOUT	20	// Frames w/o new data in slow-data-only tx mode


#define REPLACEMENT_HDR_CALL	"        "	// 8 chars of MyCall
#define REPLACEMENT_HDR_SIGN	"miss"		// 4 chars of MyCall2


void	rptr_init_data(void);


void	rptr_init_hardware(void);	// Init des DStar-Modus
void	rptr_exit_hardware(void);	// Verlassen des DSTar-Modus

void	rptr_init_header(const tds_header *header);

void	rptr_replacement_header(void);


void	rptr_receive(void);
void	rptr_disable_receive(void);

// In the case of long TXD, you can turn on PTT before you can send a valid
// header. Until a transmission (rptr_transmit() call) starts, a preamble pattern will
// output on TXout after the configured TX-delay
// a timeout of 420ms disables PTT, if no rptr_transdmit() called
void	rptr_transmit_preamble(void);

// Enable Voice-Tranmitting, starts with tx-delay, preamble and header
// Transmit 60 Bytes between 2 sync cycles (set with "setSlowData()")
void	rptr_transmit(void);


// Enable Silence-Tranmitting, starts with tx-delay, preamble and header
// Transmit 60 Bytes between 2 sync cycles (set with "setSlowData()")
// If no data available, DVcmdRmtTXoff -> DVUP
void	rptr_transmit_data(void);

// use pkt_nr_stop:
// 0xFF = stop if buffer runs out of voice data
// 0 to 251 = stop after transmitting this buffer-index
void	rptr_endtransmit(unsigned char pkt_nr_stop);

// Adds a voice paket into the transmit-voice-buffer on position pkt_nr.
// If pkt_nr is not in 0..VoiceTxBufSize, the pkt is stored as last one (newest).
void	rptr_addtxvoice(const tds_voicedata *buf, unsigned char pkt_nr);

unsigned char rptr_get_unsend(void);

unsigned char rptr_get_txpos(void);


char	*rptr_getheader(void);

unsigned char rptr_copycurrentrxvoice(tds_voicedata *dest);
void	rptr_copyrxvoice(tds_voicedata *dest, unsigned char nr);

// force to check the first SYNC-pattern in the voicedata just after header.
// if no pattern detected, stop RX + set LOST flag
void	rptr_forcefirstsync(void);


void	rptr_transmit_fullpreamble(void);	// used in dongle module
void	rptr_transmit_shortpreamble(void);
void	rptr_transmit_eotpattern(void);		// used in dongle module
void	rptr_transmit_brkeot(void);

#endif // DSTAR_FUNC_H_

