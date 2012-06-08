/*
 * eeprom.c
 *
 *  Created on: 22.04.2011
 *      Author: Jan Alte, DO1FJN
 *
 * This module coordinates read/write jobs on standard TWI(I²C) EEProm's.
 * It's simplified to "one access at time" only (possible is 2 parallel on
 * both devices).
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

#include "eeprom.h"
#include "../hardware/twi_func.h"

#include "compiler.h"

//#define ACK_POLLING_RETRIES	48	// ~6ms (optimized code); 5ms minimum needed
#define ACK_POLLING_RETRIES	228	// 114 measured on DebugSession @ 24512


// EEProm JOB structure (entry)
typedef struct {
  tEEstatus	Status;
  U16		Length;
  U32		Addr;
  char 		*buffer;
} tEEjob;


static tEEjob	EEjobs[EEjobSize];

static U8	EEjobPos=0, EEjobNxt=0, ee_retry_count;


void ee_twireaddone(tTWIresult TWIres, unsigned int transfered_bytes);	// Fwd
void ee_pagewrtdone(tTWIresult TWIres, unsigned int transfered_bytes);	// Fwd

#define EE_DONEXTJOB()		(EEjobPos = (EEjobPos+1)%EEjobSize)

#ifdef DVATRX
__inline U8 ee_getpagesize(unsigned int addr) {
  if (addr >= EE_DV_BASE) {
#ifdef USE_DISP_EEPROM
    if (addr >= EE_DISP_BASE) return EE_DISP_PAGESIZE;
#endif
    return EE_DV_PAGESIZE;
  }
  return EE_TRX_PAGESIZE;
}
#else
#define ee_getpagesize(x)	EE_DV_PAGESIZE
#endif


__inline void ee_pagewrite(tEEjob *curr) {
  U8 pagelen, pagesize;
  pagesize = ee_getpagesize(curr->Addr);
  pagelen  = pagesize - (curr->Addr & (pagesize-1));
  if (curr->Length < pagelen) pagelen = curr->Length;
  // call basic TWI rouine to write data to a page:
  ee_write(curr->Addr, curr->buffer, pagelen, ee_pagewrtdone);
}


// process a job waiting in the queue...
void ee_processnextjob(void) {
  tEEjob *curr;
  while (EEjobPos != EEjobNxt) {	// minimum one job unfinished...
    curr = &EEjobs[EEjobPos];		// pointer to current job
    switch(curr->Status) {
    case eeBUSYREAD:			// reading data?
      ee_read(curr->Addr, curr->buffer, curr->Length, ee_twireaddone); //read complete block
      return;
    case eeBUSYWRITE:			// writing data?
      ee_pagewrite(curr);		// write next page.
      return;
    default:
      curr->Status = eeIVALIDJOB;
      EE_DONEXTJOB();			// what kind of job? Only READ/WRITE are allowed
      ee_retry_count = 0;
      break;				// ignore this
    } //hctiws
  } // ehliw
}



// call-back handler, if TWI finish a ee-read job:
void ee_twireaddone(tTWIresult TWIres, unsigned int transfered_bytes) {
  tEEjob *curr = &EEjobs[EEjobPos];
  switch(TWIres) {
  case TWIok:		// all ok
    ee_retry_count = 0;	// success -> reset retry-counter
    if (transfered_bytes < curr->Length) {	// ohh, TWI don't read hole bock:
      curr->buffer += transfered_bytes;
      curr->Addr   += transfered_bytes;
      curr->Length -= transfered_bytes;
    } else {
      curr->buffer += curr->Length;
      curr->Length = 0;
      curr->Status = eeDONE;
      EE_DONEXTJOB();
    }
    break;
  case TWInak:	// EEProm in write-cycle, try again
    if (ee_retry_count < ACK_POLLING_RETRIES) {
      ee_retry_count++;		// count, until EE-Prom is ready or timeout reached
    } else {
      curr->Status = eeTIMEOUT;
      ee_retry_count = 0;
      EE_DONEXTJOB();
    } // esle
    break;
  case TWIerror:
  default:
    curr->Status = eeERROR;
    EE_DONEXTJOB();
    break;
  }
  ee_processnextjob();
}

#ifdef DEBUG
U32 ee_last_pollcount;
#endif

void ee_pagewrtdone(tTWIresult TWIres, unsigned int transfered_bytes) {
  tEEjob *curr = &EEjobs[EEjobPos];
  switch(TWIres) {
  case TWIok:
#ifdef DEBUG
    if (ee_retry_count > 0) ee_last_pollcount = ee_retry_count;
#endif
    ee_retry_count = 0;
    if (transfered_bytes < curr->Length) {
      curr->buffer += transfered_bytes;
      curr->Addr   += transfered_bytes;
      curr->Length -= transfered_bytes;
    } else {
      curr->buffer += curr->Length;
      curr->Length = 0;
      curr->Status = eeDONE;
      EE_DONEXTJOB();
    }
    break;
  case TWInak:	// EEProm in write-cycle, try again (ACKnowledge polling)
    if (ee_retry_count < ACK_POLLING_RETRIES) {
      ee_retry_count++;
    } else {
      curr->Status = eeTIMEOUT;
      ee_retry_count = 0;
      EE_DONEXTJOB();
    } // esle
    break;
  case TWIerror:
  default:
    curr->Status = eeERROR;
    EE_DONEXTJOB();
    break;
  }
  ee_processnextjob();
}



// API functions
// crete a new job in queue and start processing, if idle.

char eeprom_read(char *dest, unsigned short len, unsigned int addr) {
  tEEjob *next;
  char jobnr = EEjobNxt;
  U8   jobnx = (jobnr+1)%EEjobSize;
  if (jobnx == EEjobPos) return EEPROM_QUEUE_BUSY;
  next = &EEjobs[(U8)jobnr];
  next->buffer = dest;
  next->Length = len;
  next->Addr   = addr;
  next->Status = eeBUSYREAD;
  EEjobNxt = jobnx;
  if (jobnr == EEjobPos) ee_processnextjob();
  return jobnr;
}


char eeprom_write(char *src, unsigned short len, unsigned int addr) {
  tEEjob *next;
  char jobnr = EEjobNxt;
  U8   jobnx = (jobnr+1)%EEjobSize;
  if (jobnx == EEjobPos) return EEPROM_QUEUE_BUSY;
  next = &EEjobs[(U8)jobnr];
  next->buffer = src;
  next->Length = len;
  next->Addr   = addr;
  next->Status = eeBUSYWRITE;
  EEjobNxt = jobnx;
  if (jobnr == EEjobPos) ee_processnextjob();
  return jobnr;
}


// returns status of a job:

tEEstatus eeprom_status(char ID) {
  if ( (ID<0) || (ID>EEjobSize) ) return eeINVALIDID;
  return (EEjobs[(U8)ID].Status);
}

