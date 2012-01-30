/*
 * pick21mode.c
 *
 *  Created on: 26.01.2012
 */

#include "pick21mode.h"
#include "rptr_func.h"

#include "compiler.h"

U8	txbuf_areastart;
U8	last_voicepacket;
U32	packet_marker;


void add_icom_voice_reset(void) {
  txbuf_areastart  = 0;
  packet_marker    = 0;
  last_voicepacket = 0;
}


void add_fifo_voice_2_rptr(unsigned char number, const char *voicepacket) {
  U8 buffer_nr;
  if (number & 0x40) {
    rptr_endtransmit(0xFF);
  } else {
    buffer_nr = (last_voicepacket+1)%VoiceTxBufSize;
    if (RPTR_is_set(RPTR_TX_EMPTY)) {
      buffer_nr = (rptr_get_txpos()+2)%VoiceTxBufSize;
    }
    rptr_addtxvoice((tds_voicedata *)voicepacket, buffer_nr);
    last_voicepacket = buffer_nr;
    if ((voicepacket[9]==0x55)&&(voicepacket[10]==0x55)&&(voicepacket[11]==0x55)) {
      rptr_endtransmit(0xFF);
    }
  }
}


void add_icom_voice_2_rptr(unsigned char number, const char *voicepacket) {
  U8 curr_transmitpos;
  U8 buffer_nr;

  if (number & 0x40) {
    rptr_endtransmit(0xFF);
  } else {
    if ( (packet_marker & (1<<number)) ||
	 ((number==0)&&(last_voicepacket==(DSTAR_SYNCINTERVAL-1))) ) {
      txbuf_areastart += DSTAR_SYNCINTERVAL;
      if (txbuf_areastart > (VoiceTxBufSize-DSTAR_SYNCINTERVAL) )
	txbuf_areastart = 0;
      packet_marker = 0;
    }
    packet_marker |= (1<<number);

    if (RPTR_is_set(RPTR_TX_EMPTY)) {
      curr_transmitpos = rptr_get_txpos();
      txbuf_areastart = curr_transmitpos - (curr_transmitpos % DSTAR_SYNCINTERVAL);
      if (number <= (curr_transmitpos%DSTAR_SYNCINTERVAL)) {
	txbuf_areastart += DSTAR_SYNCINTERVAL;
	if (txbuf_areastart > (VoiceTxBufSize-DSTAR_SYNCINTERVAL) )
	  txbuf_areastart = 0;
      }
      packet_marker = 0;
    }

    buffer_nr = txbuf_areastart + number;
    rptr_addtxvoice((tds_voicedata *)voicepacket, buffer_nr);
    last_voicepacket = number;
    if ((voicepacket[9]==0x55)&&(voicepacket[10]==0x55)&&(voicepacket[11]==0x55)) {
      rptr_endtransmit(0xFF);
    }
  }
}
