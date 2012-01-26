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


void add_icom_voice_2_rptr(unsigned char number, const char *voicepacket) {
  U8 curr_transmitpos = rptr_get_txpos();
  U8 buffer_nr;
  U8 nr_difference;
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
    nr_difference = (txbuf_areastart+number+VoiceTxBufSize-curr_transmitpos) % VoiceTxBufSize;
    if (nr_difference >= (VoiceTxBufSize-DSTAR_SYNCINTERVAL)) {
      txbuf_areastart += DSTAR_SYNCINTERVAL;
      if (txbuf_areastart > (VoiceTxBufSize-DSTAR_SYNCINTERVAL) )
	txbuf_areastart = 0;
    }
    buffer_nr = txbuf_areastart + number;
    rptr_addtxvoice((tds_voicedata *)voicepacket, buffer_nr);
    last_voicepacket = number;
    if ((voicepacket[9]==0x55)&&(voicepacket[10]==0x55)&&(voicepacket[11]==0x55)) {
      rptr_endtransmit(0xFF);
    }
  }
}

