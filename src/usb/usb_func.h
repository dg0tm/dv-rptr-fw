/*
 * usb_func.h
 *
 * general managing functions to enable / disable USB (power / clock)
 *
 *  Created on: 23.07.2010
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

 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef USB_FUNC_H_
#define USB_FUNC_H_

#define CDC_RXBUFFERSIZE	2048


typedef void (*tusbfunction)(void);	// void-void Function usb-task.
typedef void (*tusb_to_func)(int);	// void-int Function

extern tusbfunction usb_handler;

extern void rptr_reset_inferface(void);	// called, if disconnected

void	usb_init(void);
void	usb_exit(void);


void	cdc_transmit(const char *data, int len);

int	cdc_received(void);		// returns received bytes (in fifo)

void	cdc_flushrx(void);		// löscht den Inhalt des Rx

int	cdc_copyblock(char *destbuffer, int len);

unsigned char cdc_look_byte(int pos);
unsigned short cdc_look_leword(int pos);

void	cdc_enabletimeout(tusb_to_func Function, int ms);

/* cdc_isactive()
 * returns 'true' if USB port is connected and configured
 */
int	cdc_isactive(void);


#endif // USB_FUNC_H_
