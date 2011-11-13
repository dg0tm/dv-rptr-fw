/*
 * tranceiver.h
 *
 *  Created on: 13.11.2011
 *      Author: Jan Alte, DO1FJN
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

#ifndef TRANCEIVER_H_
#define TRANCEIVER_H_

typedef void (*ttrx_function)(void);

typedef unsigned int (*ttrx_setfreq)(unsigned int, unsigned int);

// return code (capabilities) of TRX
#define TRXCAP_AVAIL	0x01
#define TRXCAP_DUPLEX	0x02
#define TRXCAP_HF10	0x08		// 10m band
#define TRXCAP_HF06	0x10		// 6m
#define TRXCAP_VHF	0x20		// 2m (140-150MHz)
#define TRXCAP_UHF	0x40		// 70cm
#define TRXCAP_SHF	0x80		// 23cm


extern ttrx_function	trx_standby;
extern ttrx_function	trx_receive;
extern ttrx_function	trx_transmit;
extern ttrx_setfreq	trx_setfreq;


unsigned int trx_init(void);

// configuration interface:

#define	CONFIG_C1_SIZE	(12)		// 12 byte struct

char *	cfg_read_c1(char *config_buffer);
void	cfg_write_c1(const char *config_data);


#endif // TRANCEIVER_H_
