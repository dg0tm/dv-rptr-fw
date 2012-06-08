/*
 * stdmodem.h
 *
 *  Created on: 22.05.2012
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR firmware (RPTRfirmware).
 * For general information about RPTRfirmware see "rptr-main.c".
 *
 * RPTRfirmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The RPTRfirmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef STDMODEM_H_
#define STDMODEM_H_

#define DATA_TIMEOUT_MS		5	// 5ms time gap allowed for incoming data

#define PCWD_TIMEOUT		(30UL*MASTERCLOCK)	// 30 seconds


#define STA_RXENABLE_MASK	0x01
#define STA_TXENABLE_MASK	0x02
#define STA_WDENABLE_MASK	0x04
#define STA_CRCENABLE_MASK	0x08
#define STA_IO21_STATE		0x10
#define STA_IO23_STATE		0x20
#define STA_CANDUPLEX_MASK	0x40
#define STA_NOCONFIG_MASK	0x80

#define STA_RECEIVING		0x01
#define STA_TRANSMITTING	0x02
#define STA_PCWATCHDOG		0x04

extern unsigned char 		status_control;

#define RPTR_CONFIGURED		((status_control&STA_NOCONFIG_MASK) == 0)


void	rptr_standby(void);

void	init_modemdata(void);

void	handle_pcdata(void);

void	handle_hfdata(void);


#endif // STDMODEM_H_
