/*
 * twi_func.c
 *
 * I�C functions. This module is not completed jet! A complete version must
 * implement eeprom_read()/eeprom_write() functions (handle 24C512 eeprom).
 *
 * On HW0.9PT TWI-Write function is used to set up the digital potentiometer.
 *
 *  Created on: 08.03.2009
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
 * Report:
 * 2011-07-03  JA  BugFix: ee_write works now with "ack polling", no additional char written
 * 2011-12-15  JA  BugFix: TXCOMP-INT-enabeling must be after transfer-start
 * 2012-01-28  JA  BugFix: EEread-NAK generates no NAK-Int-Request
 * 2012-01-29  JA  reg_read() implemented
 * 2012-06-19  JA  reset STATUS register (clears NAK of previous reads)
 * 2012-08-01  JA  "blocking" write functions added
 *
 */


#include "twi_func.h"
#include "hw_defs.h"
#include "intc.h"
#include "gpio_func.h"	// used fpr SLEEP() macro only

#include "compiler.h"

#define TWI_PDAC	AVR32_PDCA.channel[TWI_CHANNEL]

#define TWIIO		AVR32_TWI
#define TWIIRQ		AVR32_TWI_IRQ

#define TWI_DIV 	(MASTERCLOCK/TWICLOCK)
#define TWICKDIV	0	//7
#define TWICHDIV 	((TWI_DIV+1)/2-4)
#define TWICLDIV 	((TWI_DIV+1)/2-4)

#define TWIQueueMask	0x001F
#define TWIQueueSize	(TWIQueueMask+1)

#define TWI_EE_DEVICE	0x50


typedef enum {
  TWInop, TWIreset, TWIread, TWIwrite, EEread, EEwrite, REGread, REGwrite
} tTWIcmd;

typedef struct {
  tTWIcmd	cmd;
  U8		device;
  U16		addr;
  const char	*buffer;
  U32		len;
  twi_handler	func;
} ttwijob;


ttwijob	TWIjobs[TWIQueueSize];		// Queue für I²C I/O
volatile U32 TWIrjob;
U32	TWIwjob, TWIcjob;
Bool	twi_running;			// used to pause i/o
Bool	twi_idle;			// true, if no job processed
U8	twi_jobrecursion;


__inline void twi_reset(void) {
  TWIIO.cr   = AVR32_TWI_SWRST_MASK;
  TWIIO.cwgr = (TWICKDIV << AVR32_TWI_CWGR_CKDIV_OFFSET)|	\
    (TWICHDIV << AVR32_TWI_CWGR_CHDIV_OFFSET)|
    (TWICLDIV << AVR32_TWI_CWGR_CLDIV_OFFSET);
}


void twi_startpdca(const ttwijob *job) {
  TWI_PDAC.mar = (U32)job->buffer;
  TWI_PDAC.tcr = job->len;
  TWIIO.cr     = AVR32_TWI_MSEN_MASK;
  TWI_PDAC.cr  = AVR32_PDCA_TEN_MASK;
  // TXCOMP set to low with the beginning of the transfer...
  TWIIO.sr;	// reset STATUS register (clears NAK of previous reads)
  TWIIO.ier    = AVR32_TWI_IER_NACK_MASK|AVR32_TWI_IER_TXCOMP_MASK;
}


__inline void twi_startmultipleread(const ttwijob *job) {
  TWIIO.idr    = AVR32_TWI_IDR_TXCOMP_MASK;
  TWI_PDAC.mar = (U32)job->buffer;
  TWI_PDAC.tcr = job->len-1;
  TWIIO.cr     = AVR32_TWI_MSEN_MASK;
  TWI_PDAC.cr  = AVR32_PDCA_TEN_MASK;
  TWI_PDAC.ier = AVR32_PDCA_IER_TRC_MASK;	// multiple read done, one byte left (read+STOP)
  TWIIO.ier    = AVR32_TWI_IER_NACK_MASK;	// when no device -> a NAK receives after address
}


void twi_process_job(void) {
  TWIIO.idr = AVR32_TWI_IDR_TXCOMP_MASK|AVR32_TWI_IDR_NACK_MASK; // Disable INTs
  if (twi_running && (TWIrjob != TWIcjob)) {		// next job?
    ttwijob *actjob;
    twi_idle   = false;
    actjob     = &TWIjobs[TWIrjob&TWIQueueMask];
    TWIIO.mmr  = actjob->device << AVR32_TWI_MMR_DADR_OFFSET;
    TWIIO.iadr = actjob->addr;
    // MREAD=0, IADRZ=0 (no int. adr)
    if (actjob->cmd==REGread)
      TWIIO.mmr |= 1 << AVR32_TWI_MMR_IADRSZ_OFFSET;	// write 1-Byte Address before

    switch (actjob->cmd) {
    case TWInop:
    case TWIreset:
      TWIrjob++;	// do-nothing-job
      break;
    case EEread:
      TWIIO.mmr |=  2 << AVR32_TWI_MMR_IADRSZ_OFFSET;	// read 2-Byte Address before
    case REGread:
    case TWIread:
      TWIIO.rhr;	// read it ?!
      TWI_PDAC.psr = AVR32_PDCA_PID_TWI_RX;
      TWIIO.mmr |= AVR32_TWI_MMR_MREAD_MASK;
      if (actjob->len < 2) {	// Single Byte read.
	TWIIO.cr = AVR32_TWI_CR_START_MASK|AVR32_TWI_CR_STOP_MASK;
        twi_startpdca(actjob);
      } else {
	TWIIO.cr = AVR32_TWI_CR_START_MASK;
        twi_startmultipleread(actjob);
      }
      return;
    case EEwrite:
      TWIIO.mmr |= 2 << AVR32_TWI_MMR_IADRSZ_OFFSET;	// write 2-Byte Address before
    case TWIwrite:
      TWI_PDAC.psr = AVR32_PDCA_PID_TWI_TX;
      twi_startpdca(actjob);
      return;
    case REGwrite:
      TWIIO.mmr |= 1 << AVR32_TWI_MMR_IADRSZ_OFFSET;	// write 1-Byte Address before
      TWI_PDAC.psr = AVR32_PDCA_PID_TWI_TX;
      // like startpdca, but with "embedded" data
      TWI_PDAC.mar = (U32)(&(actjob->buffer));
      TWI_PDAC.tcr = actjob->len;
      TWIIO.cr     = AVR32_TWI_MSEN_MASK;
      TWI_PDAC.cr  = AVR32_PDCA_TEN_MASK;
      TWIIO.ier    = AVR32_TWI_IER_TXCOMP_MASK|AVR32_TWI_IER_NACK_MASK;
      return;
    } // hctiws cmd
  }  // fi next
  twi_idle  = true;
}


INTERRUPT_FUNC twi_stoprd_int(void) {
  TWI_PDAC.idr = AVR32_PDCA_IDR_TRC_MASK;
  TWIIO.cr     = AVR32_TWI_CR_STOP_MASK;
  TWI_PDAC.tcr = 1;				// Last Transfer, then STOP-Condition
  TWIIO.ier    = AVR32_TWI_IER_TXCOMP_MASK;	// twi_interrupt enable
}


INTERRUPT_FUNC twi_interrupt(void) {
  ttwijob *currjob;
  U32 twisr   = TWIIO.sr;
  U32 leftb   = TWI_PDAC.tcr;			// Bytes transfered from PDCA
  TWIIO.cr    = AVR32_TWI_MSDIS_MASK;		// TWI stop
  TWI_PDAC.cr = AVR32_PDCA_TDIS_MASK|AVR32_PDCA_ECLR_MASK; // TWI-PDCA stop
  twi_idle    = false;				// ever false while handling CallBack!
  // handle callback of curr job
  currjob = &TWIjobs[TWIrjob&TWIQueueMask];	// pointer to current job
  TWIrjob++;					// next job / this one is done
  if (currjob->func != NULL) {
    if (twisr&AVR32_TWI_SR_NACK_MASK) {
      TWI_PDAC.idr = AVR32_PDCA_IDR_TRC_MASK;	// stop PDAC (prev. reading mode)
      // Resetting TWI is not the best solution, but I don't find
      // a way to get a clen NAK for the next job.
      // In the case of no RESET a 2nd I²C Adress-Byte-Burst appears
      twi_reset();
      /*U32 txed_data = currjob->len - leftb - 1;
      if ((currjob->cmd == TWIwrite)||(currjob->cmd == EEwrite)) {
	if ((twisr&AVR32_TWI_SR_TXRDY_MASK)==0) txed_data--;
      }
      // ToDo PDCA counter lesen -> NAK bei Adr? -> nodevice
      currjob->func((txed_data==0)?TWInodevice:TWInak, txed_data);*/
      currjob->func(TWInak, 0); //currjob->len - leftb);
    } else {
      currjob->func((leftb==0)?TWIok:TWIerror, currjob->len - leftb);
    }
  } // fi function definded
  twi_process_job();
} // end int



/*! \name TWI / EE-Prom API Functions
 */
//! @{

void twi_exit(void) {
  twi_running= false;
  twi_idle   = false;
  TWIIO.idr  = 0xFFFF;
  TWIIO.cr   = AVR32_TWI_SWRST_MASK|AVR32_TWI_SVDIS_MASK|AVR32_TWI_MSDIS_MASK;
  TWIIO.cwgr = 0;	// To detect a disabled TWI
  // ToDo: pm_disable_TWI() ...
}


void twi_init(void) {
  // ToDo: pm_enable_TWI()...
  TWIIO.idr  = 0xFFFF;
  twi_reset();

  TWI_PDAC.idr = 0x0FFF;
  TWI_PDAC.cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  TWI_PDAC.mr  = 0x00 << AVR32_PDCA_SIZE_OFFSET; // Byte
  TWI_PDAC.tcrr= 0;

  TWIrjob = 0;
  TWIwjob = 0;
  TWIcjob = 0;
  twi_jobrecursion = 0;
  INTC_register_interrupt(twi_interrupt, TWIIRQ, TWI_INTPRIO);
  INTC_register_interrupt(twi_stoprd_int, AVR32_PDCA_IRQ_0+TWI_CHANNEL, TWI_INTPRIO);
  twi_running = true;
  twi_idle    = true;
}


ttwijob *twi_getnewjob(void) {
  ttwijob *newjob = NULL;
  if ( (TWIwjob-TWIrjob) < TWIQueueSize ) {
    Disable_global_interrupt();		// The following code must be atomar
    newjob = &TWIjobs[TWIwjob&TWIQueueMask];
    TWIwjob++;
    twi_jobrecursion++;
    Enable_global_interrupt();
  }
  return newjob;
}


void twi_acknewjob(void) {
  bool is_idle;
  Disable_global_interrupt();		// The following code must be atomar
  if ((--twi_jobrecursion)==0) {	// decrement recursion counter
    TWIcjob = TWIwjob;			// Complete job, if non-recursion call
  }
  is_idle  = twi_idle;
  twi_idle = !twi_running;
  Enable_global_interrupt();
  if (twi_running && is_idle) {
//  if ((TWIIO.imr & AVR32_TWI_IMR_TXCOMP_MASK)==0) {
    twi_process_job();
  }
} // fi idle


ttwijob *twi_blocking_getjob() {
  ttwijob *freejob;
  do {
    freejob = twi_getnewjob();
    if (freejob != NULL) return freejob;
    SLEEP();
  } while (true);
}


tTWIresult twi_write(unsigned char adr, const char *data, unsigned int len, twi_handler RetFunc) {
  ttwijob *newjob = twi_getnewjob();
  if ( newjob == NULL ) return TWIbusy;
  newjob->cmd  = TWIwrite;
  newjob->device = adr;
  newjob->buffer = data;
  newjob->len  = len;
  newjob->func = RetFunc;
  twi_acknewjob();	// Acknowledge job
  return TWIok;
}


void twi_blocking_write(unsigned char adr, const char *data, unsigned int len, twi_handler RetFunc) {
  ttwijob *newjob = twi_blocking_getjob();
  newjob->cmd  = TWIwrite;
  newjob->device = adr;
  newjob->buffer = data;
  newjob->len  = len;
  newjob->func = RetFunc;
  twi_acknewjob();	// Acknowledge job
}


tTWIresult twi_read(unsigned char adr, char *dest, unsigned int len, twi_handler RetFunc) {
  ttwijob *newjob = twi_getnewjob();
  if ( newjob == NULL ) return TWIbusy;
  newjob->cmd  = TWIread;
  newjob->device = adr;
  newjob->buffer = dest;
  newjob->len  = len;
  newjob->func = RetFunc;
  twi_acknewjob();	// Acknowledge job
  return TWIok;
}


tTWIresult ee_write(unsigned int adr, const char *data, unsigned int len, twi_handler RetFunc) {
  ttwijob *newjob = twi_getnewjob();
  if ( newjob == NULL ) return TWIbusy;
  newjob->cmd  = EEwrite;
  newjob->device = TWI_EE_DEVICE | ((adr >> 16)&0x7);
  newjob->addr = adr&0xFFFF;
  newjob->buffer = data;
  newjob->len  = len;
  newjob->func = RetFunc;
  twi_acknewjob();	// Acknowledge job
  return TWIok;
}


tTWIresult ee_read(unsigned int adr, char *dest, unsigned int len, twi_handler RetFunc) {
  ttwijob *newjob = twi_getnewjob();
  if ( newjob == NULL ) return TWIbusy;
  newjob->cmd  = EEread;
  newjob->device = TWI_EE_DEVICE | ((adr >> 16)&0x7);
  newjob->addr = adr&0xFFFF;
  newjob->buffer = dest;
  newjob->len  = len;
  newjob->func = RetFunc;
  twi_acknewjob();	// Acknowledge job
  return TWIok;
}


tTWIresult reg_write(unsigned char adr, unsigned char reg, unsigned int value, char reg_size, twi_handler RetFunc) {
  ttwijob *newjob = twi_getnewjob();
  if ((reg_size==0)||(reg_size>4)) return TWIerror;
  if ( newjob == NULL ) return TWIbusy;
  value <<= (4-reg_size)*8;		// shift bytes to high (big endian)
  newjob->cmd    = REGwrite;
  newjob->device = adr;
  newjob->addr   = reg;
  newjob->buffer = (char *)value;
  newjob->len    = reg_size;
  newjob->func   = RetFunc;
  twi_acknewjob();	// Acknowledge job
  return TWIok;
}


void reg_blocking_write(unsigned char adr, unsigned char reg, unsigned int value, char reg_size, twi_handler RetFunc) {
  ttwijob *newjob;
  if ((reg_size==0)||(reg_size>4)) return;
  newjob = twi_blocking_getjob();
  value <<= (4-reg_size)*8;		// shift bytes to high (big endian)
  newjob->cmd    = REGwrite;
  newjob->device = adr;
  newjob->addr   = reg;
  newjob->buffer = (char *)value;
  newjob->len    = reg_size;
  newjob->func   = RetFunc;
  twi_acknewjob();	// Acknowledge job
}


tTWIresult reg_read(unsigned char adr, unsigned char reg, char *dest, char reg_size, twi_handler RetFunc) {
  ttwijob *newjob = twi_getnewjob();
  if ( newjob == NULL ) return TWIbusy;
  newjob->cmd  = REGread;
  newjob->device = adr;
  newjob->addr   = reg;
  newjob->buffer = dest;
  newjob->len  = reg_size;
  newjob->func = RetFunc;
  twi_acknewjob();
  return TWIok;
}


char twi_busy(void) {
  return (TWIrjob != TWIwjob)||((TWIIO.imr & AVR32_TWI_IMR_TXCOMP_MASK)!= 0);
}


void twi_pause(void) {
  if (twi_running) {
    twi_running = FALSE;
    if (TWIIO.cwgr != 0) {
      while ((TWIIO.imr & AVR32_TWI_IMR_TXCOMP_MASK)!=0); //wait current transfer
    }
  }
  twi_release_pins();
}


void twi_continue(void) {
  twi_connect_pins();
  TWIIO.sr;			// Dummy-read status
  if ((TWIIO.cwgr != 0) && (!twi_running)) {
    twi_running = TRUE;
    if ((TWIIO.imr & AVR32_TWI_IMR_TXCOMP_MASK)==0) {
      twi_process_job();
    } // fi idle
  }
}


//! @}
