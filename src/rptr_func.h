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
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef DSTAR_FUNC_H_
#define DSTAR_FUNC_H_

#include "dv_dstar.h"

#define	VoiceTxBufSize	(12*DSTAR_SYNCINTERVAL)		// Size of VoiceBuf

#define VoiceRxBufSize	DSTAR_SYNCINTERVAL		// Fixed to one sync


extern unsigned int RPTR_Flags;
// Receive Flags (checked in handle_hfdata()) Bit 0..7
#define RPTR_RX_START		0x01
#define RPTR_RX_STOP		0x02
#define RPTR_RX_LOST		0x04
#define RPTR_RX_SYNC		0x08
#define RPTR_RX_FRAME		0x10
#define RPTR_RX_HEADER		0x20

#define RPTR_RECEIVING		0x80	// permanent indicator, don't clear flag

// Transmit Flags Bit 8..15
#define RPTR_TX_EARLYPTT	0x0100
#define RPTR_TRANSMITTING	0x8000	// permanent indicator, don't clear flag

#define RPTR_is_set(f)		(RPTR_Flags&f)
#define RPTR_clear(f)		(RPTR_Flags &= ~f)


// maximum receive packets, if no sync-frame received:
#define RPTR_MAX_PKT_WO_SYNC	50

#define DSTAR_BEFOREFRAMEENDS	16	// 16 bit-time before gmsk-modulator-data runs out
// attention: this value must be smaller than 32 (last 32bit must be alreay loaded in GMSK fct)

#define DSTAR_DECODE_TO		250	// 5 Sekunden (Minimum-Wert ist 8 = eine Header-Länge)

#define DSTAR_DATAIDLETIMEOUT	20	// Frames w/o new data in slow-data-only tx mode




void	rptr_init_data(void);


void	rptr_init_hardware(void);	// Init des DStar-Modus
void	rptr_exit_hardware(void);	// Verlassen des DSTar-Modus

void	rptr_init_header(const tds_header *header);
void	rptr_update_mycall(const char *MyCallSign);
void	rptr_update_route(const char *NewRoute);

void	rptr_update_dest(const char *NewDest);
void	rptr_update_depart(const char *NewDepart);
void	rptr_update_yourcall(const char *NewYourCall);

void	rptr_set_emergency(void);
void	rptr_clr_emergency(void);


void	rptr_receive(void);

// Enable Voice-Tranmitting, starts with tx-delay, preamble and header
// Transmit 60 Bytes between 2 sync cycles (set with "setSlowData()")
void	rptr_transmit(void);

// In the case of long TXD, you can turn on PTT before you can send a valid
// header. The configured TXD must be >138ms (660bit haeder need 137.5ms on air)
void	rptr_transmit_early_start(void);

// Enable Silence-Tranmitting, starts with tx-delay, preamble and header
// Transmit 60 Bytes between 2 sync cycles (set with "setSlowData()")
// If no data available, DVcmdRmtTXoff -> DVUP
void	rptr_transmit_data(void);

void	rptr_endtransmit(void);

// Adds a voice paket into the transmit-voice-buffer on position pkt_nr.
// If pkt_nr is not in 0..VoiceTxBufSize, the pkt is stored as last one (newest).
void	rptr_addtxvoice(const tds_voicedata *buf, unsigned char pkt_nr);

unsigned char rptr_get_unsend(void);

char	*rptr_getheader(void);

unsigned char rptr_copycurrentrxvoice(tds_voicedata *dest);
void	rptr_copyrxvoice(tds_voicedata *dest, unsigned char nr);


#endif // DSTAR_FUNC_H_

