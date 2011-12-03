/*
 * rs232_func.c
 *
 * Simple RS232 communication routines using a fifo buffer.
 *
 *
 *  Created on: 11.08.2009
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
 * 2010-05-01	TimeOut-Mechanism
 * 2011-04-17	BugFix rs232_copyblock (return-value of memcpy is not next ptr)
 * 2011-11-27	add variable rs232_baudrate -> setting timeouts w/o knowing it
 */


#include "rs232_func.h"
#include "hw_defs.h"
#include "gpio_func.h"		// !FORCEOFF Pin
#include "intc.h"
#include "compiler.h"
#include <string.h>

#define CHAR_LF		0x0A

#define RS232_PDAC_RX	AVR32_PDCA.channel[RS232_RXCH]
#define RS232_PDAC_TX	AVR32_PDCA.channel[RS232_TXCH]


U32	rs232_baudrate;

U8	rs232_rxbuffer[RS232_RXBUFFERSIZE];
int	rs232_rdpos, rs232_txtendpos;

trxtimeoutfunc	TimeOutHandler;

#define rs232_wrpos	(RS232_RXBUFFERSIZE-RS232_PDAC_RX.tcr)

#define rs232_rxlen	((2*RS232_RXBUFFERSIZE-RS232_PDAC_RX.tcr-rs232_rdpos)%RS232_RXBUFFERSIZE)



/*! \name RS232 Interrupt Functions
 */
//! @{

__inline void rs232_reset_receive(void) {
  RS232_PDAC_RX.cr   = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  RS232_PDAC_RX.mar  = (U32)rs232_rxbuffer;
  RS232_PDAC_RX.tcr  = sizeof(rs232_rxbuffer);
  RS232_PDAC_RX.marr = (U32)rs232_rxbuffer;
  RS232_PDAC_RX.tcrr = sizeof(rs232_rxbuffer);
  RS232_PDAC_RX.cr   = AVR32_PDCA_TEN_MASK;
}


INTERRUPT_FUNC rs232_pdac_reload_int(void) {
  RS232_PDAC_RX.marr = (U32)rs232_rxbuffer;
  RS232_PDAC_RX.tcrr = sizeof(rs232_rxbuffer);
}


INTERRUPT_FUNC rs232_handler_int(void) {
  tRXTOmode NextTimeoutMode;
  U32 rs232_csr = RS232.csr;
  if  (rs232_csr & AVR32_USART_CSR_TIMEOUT_MASK) {
    if (TimeOutHandler != NULL)
      NextTimeoutMode = TimeOutHandler(rs232_rxlen);
    else
      NextTimeoutMode = rxto_disable;
    switch (NextTimeoutMode) {
    case rxto_disable:
      RS232.idr = AVR32_USART_IDR_TIMEOUT_MASK;	// Int off
      RS232.cr  = AVR32_USART_CR_RSTSTA_MASK;
      break;
    case rxto_waitend:
      RS232.cr = AVR32_USART_CR_RSTSTA_MASK|AVR32_USART_CR_STTTO_MASK;
      break;
    case rxto_rearm:
      RS232.cr = AVR32_USART_CR_RSTSTA_MASK|AVR32_USART_CR_RETTO_MASK;
      break;
    } //hctiws
  } // fi TimeOut
}



//! @}


/*! \name RS232 API Functions
 */
//! @{


void rs232_exit(void) {
  RS232_PDAC_RX.idr = 0xFF;
  RS232_PDAC_RX.cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  RS232_PDAC_TX.cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  RS232.idr = AVR32_USART_IDR_TIMEOUT_MASK;
  RS232.cr  = AVR32_USART_CR_RXDIS_MASK|AVR32_USART_CR_TXDIS_MASK|
    AVR32_USART_CR_STPBRK_MASK|AVR32_USART_CR_RSTRX_MASK|AVR32_USART_CR_RSTTX_MASK;
  rs232_force_off();
  TimeOutHandler = NULL;
}


void rs232_init(unsigned int baudrate, tFlowCtrl flow) {
  rs232_exit();
  rs232_auto_on();
  TimeOutHandler = NULL;
  // Init RX-Structure:
  rs232_rdpos = 0;
  RS232_PDAC_TX.mr  = 0x00 << AVR32_PDCA_SIZE_OFFSET;	// Byte Transfer
  RS232_PDAC_TX.psr = RS232PIDTX;
  RS232_PDAC_TX.tcrr = 0;				// No Next Buffer
  RS232_PDAC_RX.mr  = 0x00 << AVR32_PDCA_SIZE_OFFSET;	// Byte Transfer
  RS232_PDAC_RX.psr = RS232PIDRX;
  rs232_reset_receive();
  INTC_register_interrupt(&rs232_pdac_reload_int, AVR32_PDCA_IRQ_0+RS232_RXCH, AVR32_INTC_INT0);
  INTC_register_interrupt(&rs232_handler_int, RS232IRQ, AVR32_INTC_INT0);
  RS232_PDAC_RX.ier = AVR32_PDCA_RCZ_MASK;

  RS232.mr = (AVR32_USART_MR_NBSTOP_1 << AVR32_USART_MR_NBSTOP_OFFSET) | // 1 StopBits
    (AVR32_USART_MR_PAR_NONE << AVR32_USART_MR_PAR_OFFSET) |	// no Parity
    ((8 - 5) << AVR32_USART_MR_CHRL_OFFSET) |	// char length = 8
    (AVR32_USART_MR_MODE_NORMAL << AVR32_USART_MR_MODE_OFFSET);	// automatic Echo
  RS232.brgr = (MASTERCLOCK+((unsigned long)baudrate<<3))/((unsigned long)baudrate<<4);
  RS232.rtor = RS232_DEFAULTTIMEOUT;		// Receive TimeOut Register
  RS232.cr   = AVR32_USART_CR_RXEN_MASK|AVR32_USART_CR_TXEN_MASK| \
    AVR32_USART_CR_RSTSTA_MASK|AVR32_USART_CR_STTTO_MASK;
  // Reset Status im CSR & Start Waiting TimeOut (löscht TIMEOUT Bit in CSR)
  RS232_PDAC_TX.cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TEN_MASK;
  rs232_baudrate = (MASTERCLOCK/16) / RS232.brgr;
}



void rs232_transmit(const char *data, int len) {
  if (len==0) return;
  if (RS232_PDAC_TX.tcr==0) {	// Nothing to TX
    RS232_PDAC_TX.mar = (U32)data;
    RS232_PDAC_TX.tcr = len;
  } else {
    RS232_PDAC_TX.marr = (U32)data;
    RS232_PDAC_TX.tcrr = len;
  }
}



__inline void rs232_flushrx(void) {
  rs232_rdpos = rs232_wrpos;
}


__inline int rs232_received(void) {
  return rs232_rxlen;
}


int rs232_copyblock(char *destbuffer, int len) {
  int bytes_left;
  if (len <= RS232_RXBUFFERSIZE) {
    bytes_left = RS232_RXBUFFERSIZE-rs232_rdpos;
    if (len <= bytes_left) {	// Normalfall: kein Wrap
      memcpy(destbuffer, rs232_rxbuffer+rs232_rdpos, len);
    } else {
      memcpy(destbuffer, rs232_rxbuffer+rs232_rdpos, bytes_left);
      memcpy(destbuffer+bytes_left, rs232_rxbuffer, len-bytes_left);
    }
    rs232_rdpos = (rs232_rdpos+len)%RS232_RXBUFFERSIZE;
    return len;
  } else return 0;
}


__inline void rs232_settimeout(unsigned int bittimes) {
  RS232.rtor = bittimes;	// Receive TimeOut Register
}


void rs232_settimeoutms(unsigned int ms) {
  RS232.rtor = ((rs232_baudrate*ms)+500)/1000;	// Receive TimeOut Register
}


void rs232_enabletimeout(trxtimeoutfunc RXTOfunc, tRXTOmode startmode) {
  if (RXTOfunc != NULL) {
    RS232.idr = AVR32_USART_IDR_TIMEOUT_MASK;
    TimeOutHandler = RXTOfunc;
    switch (startmode) {
    case rxto_disable:
      break;
    case rxto_waitend:
      RS232.cr  = AVR32_USART_CR_RSTSTA_MASK|AVR32_USART_CR_STTTO_MASK;
      RS232.ier = AVR32_USART_IER_TIMEOUT_MASK;
      break;
    case rxto_rearm:
      RS232.cr  = AVR32_USART_CR_RSTSTA_MASK|AVR32_USART_CR_RETTO_MASK;
      RS232.ier = AVR32_USART_IER_TIMEOUT_MASK;
      break;
    }
  } // fi
}


void rs232_disabletimeout(void) {
  RS232.idr = AVR32_USART_IDR_TIMEOUT_MASK;
  TimeOutHandler = NULL;
}


int rs232_textlinecomplete(void) {
  while ((rs232_rxbuffer[rs232_txtendpos]!=CHAR_LF)&&(rs232_txtendpos!=rs232_wrpos)) {
    rs232_txtendpos = (rs232_txtendpos+1) % RS232_RXBUFFERSIZE;
  } // ehliw
  if ((rs232_rxbuffer[rs232_txtendpos]==CHAR_LF)&&(rs232_txtendpos!=rs232_wrpos))  {
    return ((rs232_txtendpos-rs232_rdpos+1)%RS232_RXBUFFERSIZE);
  } else {
    return 0;
  }
}


__inline void rs232_nexttextline(void) {
  rs232_txtendpos = rs232_rdpos;
}


void rs232_skiptextline(void) {
  if (rs232_txtendpos!=rs232_wrpos)
    rs232_txtendpos++;
  rs232_rdpos = rs232_txtendpos;
}


unsigned char rs232_look_byte(int pos) {
  return rs232_rxbuffer[(rs232_rdpos+pos) % RS232_RXBUFFERSIZE];
}


unsigned short rs232_look_leword(int pos) {
  U16 result;
  result = (rs232_rxbuffer[(rs232_rdpos+pos+1) % RS232_RXBUFFERSIZE] << 8) | \
      rs232_rxbuffer[(rs232_rdpos+pos) % RS232_RXBUFFERSIZE];
  return result;
}


//! @}

