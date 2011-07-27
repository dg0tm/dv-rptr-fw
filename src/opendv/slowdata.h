/*
 * slowdata.h
 *
 * Interface-Function to handle various slow-data formats
 *
 *  Created on: 29.05.2010
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
 *
 */

#ifndef SLOWDATA_H_
#define SLOWDATA_H_

#include "rs232_func.h"


#define SLOWDATA_TYPING_TO	500	// Timeout in ms

#define ICOM_CTEXT_SIZE		20	// Length of the "Free-Form-Text"

#define MESSAGE_RESEND_INTERVAL	12	// mal 420ms (Syncs) ^= ca. 5 Sekunden


typedef void (*tdatatxfunc)(const char *, int);
typedef int  (*tdatarxfunc)(void);
typedef int  (*tdatacopyfunc)(char *, int);
typedef void (*tdataflushfunc)(void);

extern tdatatxfunc	DVDataTransmit;	// -> to Serial/Usb Port (get from rx)
extern tdatarxfunc	DVDataReceived;	// <- get no of bytes of pc-data
extern tdatacopyfunc	DVDataCopyRcvd;	// <- from Port (to be tx)
extern tdataflushfunc	DVOldDataFlush;	//    store unsend data safty in trash
extern tdatarxfunc	DVNewDataAvailable; // have new data waiting 4 send out


tRXTOmode DStar_RS232_SlowData_Timeout(int len);
void	DStar_CDC_SlowData_Timeout(int len);

int	DStar_DataAvailable(void);


// Slow-Data-Receive-Handler:

// DStar_SD_Default()
// receives Data-Triples for fast internal handling
// get Digital-Squelch-Values, CText...
// use only for short blocks, handle longer (GPS, D-PRS) frame-based
void	DStar_SD_Default(const char *datatriple, unsigned int framecnt);

// same as DStar_SD_Default(), but calls DVDataTransmit() on all data-frames
// SYNC-frames skipped
void	DStar_SD_RawStream(const char *datatriple, unsigned int framecnt);

// handles Icom-Text Data and calls DVDataTransmit() with decoded
// 0x3x frames
void	DStar_SD_IcomMsgs(const char *datatriple, unsigned int framecnt);


// DStar_SD_XmitDefault()
// if no Data-Transmit function (below) defined, this one was used to
// set up 6 byte blocks and returns a pointer to it.
char	*DStar_SD_XmitDefault(unsigned int framecnt);


char	*DStar_SD_IcomXmit(unsigned int framecnt);
char	*DStar_SD_RawXmit(unsigned int framecnt);
char	*DStar_SD_BlockXmit(unsigned int framecnt);


void	SlowData_Reset(void);
void	SlowData_Flush(void);

// every time, a new transmission was received, call this function:
void	SlowData_Reset_Reception(void);

// returns true, if all 4x5 chars of a C-Text message received
char	SlowData_CMsg_Complete(void);

char	*Get_SlowData_RXMsg(void);	// returns 20char buffer of C-Text

void	Set_SlowData_TXMsg(char *msg);	// set own C-Text (20chars) for transmit
char	*Get_SlowData_TXMsg(void);	// returns a pointer to the 20char buf 4 edit

// Set a DSQ value (00 disables DSQ, 1..99 enables it)
void	Set_DigitalCodeSquelch(unsigned char number);


#endif // SLOWDATA_H_
