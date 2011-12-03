/*
 * rs232_func.h
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
 */

#ifndef RS232_FUNC_H_
#define RS232_FUNC_H_

#define RS232_DEFAULTTIMEOUT	60		// Bitzeit = 6 Byte Zeit

#define RS232_RXBUFFERSIZE	2048

typedef enum {
  no_flow, xon_xoff, rts_cts, fc_reserved
} tFlowCtrl;

typedef enum {
  rxto_disable, rxto_waitend, rxto_rearm
} tRXTOmode;

typedef tRXTOmode (*trxtimeoutfunc)(int);


void	rs232_init(unsigned int baudrate, tFlowCtrl flow);
void	rs232_exit(void);

void	rs232_transmit(const char *data, int len);

int	rs232_received(void);		// Anzahl empfangener Bytes

void	rs232_flushrx(void);		// löscht den Inhalt des Rx

int	rs232_copyblock(char *destbuffer, int len);

void	rs232_settimeout(unsigned int bittimes);
void	rs232_settimeoutms(unsigned int ms);

void	rs232_enabletimeout(trxtimeoutfunc RXTOfunc, tRXTOmode startmode);
void	rs232_disabletimeout(void);

int	rs232_textlinecomplete(void);	// Länge einer empfangenen Textzeile
void	rs232_nexttextline(void);
void	rs232_skiptextline(void);

unsigned char	rs232_look_byte(int pos);
unsigned short	rs232_look_leword(int pos);


#endif // RS232_FUNC_H_


