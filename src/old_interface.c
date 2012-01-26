/*
 * old_interface.c
 *
 *  Created on: 26.01.2012
 */

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
    if (status_control & STA_TXENABLE_MASK) {
      if ((rptr_tx_state <= RPTRTX_preamble) || (rxdatapacket.data[PKT_PARAM_IDX] != current_txid)) {
        rptr_transmit();		// Turn on Xmitter
        current_txid = rxdatapacket.data[PKT_PARAM_IDX];
      } // fi
      // -> ignore a HEADER msg with same TXID while sending
      add_icom_voice_reset();
    } else
      pc_send_byte(NAK);
    // keep 2 bytes for future use, keep layout identical to RX
    break;
  case RPTR_RXSYNC:		// start transmitting TXDelay-Preamble-Start-Header
    if (rptr_tx_state != RPTRTX_header)	// update only, if not transmitting just this moment
      rptr_replacement_header();
    if (status_control & STA_TXENABLE_MASK) {
      if ((rptr_tx_state <= RPTRTX_preamble) || (rxdatapacket.data[PKT_PARAM_IDX] != current_txid)) {
	rptr_transmit();	// Turn on Xmitter only if PTT off
	current_txid = rxdatapacket.data[PKT_PARAM_IDX];
      } // if idle
      // (transmission use last header)
      add_icom_voice_reset();
    } else
      pc_send_byte(NAK);
    break;
  case RPTR_DATA:		// transmit data (voice and slowdata or sync)
    if (rxdatapacket.data[PKT_PARAM_IDX] == current_txid)
      add_icom_voice_2_rptr(rxdatapacket.data[PKT_PARAM_IDX+1], &rxdatapacket.data[PKT_PARAM_IDX+4]);
      //add_multi_voice_2_rptr(len);
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
      ctrldata.head.cmd = RPTR_RXPREAMBLE;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
    }
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
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
      transmission = true;
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_STOP)) {
      RPTR_clear(RPTR_RX_STOP);
#ifdef SIMPLIFIED_FIFO
  // simplified Fifo buffer behavior, as requested by DG1HT
      voicedata.pktcount = 0x40;
      voicedata.rssi     = swap16(gmsk_get_rssi_avrge());
      append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
      data_transmit((char *)&voicedata, sizeof(voicedata));
#else
      ctrldata.head.cmd = RPTR_EOT;
      ctrldata.rsvd     = voicedata.pktcount;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
#endif
      transmission = false;
    } // fi start detected
    if (RPTR_is_set(RPTR_RX_LOST)) {
      RPTR_clear(RPTR_RX_LOST);
#ifdef SIMPLIFIED_FIFO
  // simplified Fifo buffer behavior, as requested by DG1HT
      voicedata.pktcount = 0x40;
      voicedata.rssi     = swap16(gmsk_get_rssi_avrge());
      append_crc_ccitt((char *)&voicedata, sizeof(voicedata));
      data_transmit((char *)&voicedata, sizeof(voicedata));
#else
      ctrldata.head.cmd = RPTR_RXLOST;
      ctrldata.rsvd     = voicedata.pktcount;
      append_crc_ccitt((char *)&ctrldata, sizeof(ctrldata));
      data_transmit((char *)&ctrldata, sizeof(ctrldata));
#endif
      transmission = false;
    } // fi start detected
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

