/*
 * dongle.h
 *
 *  Created on: 2012-04-14
 *      Author: Jan Alte, DO1FJN
 *
 * This header file is part of the DV-RPTR firmware (RPTRfirmware).
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
 * Report:
 *
 */


#ifndef DONGLE_H_
#define DONGLE_H_

#include "defines.h"
#include "dv_dstar.h"

#define DONGLE_AVAIL		0x01	// capability flag (init() return value)


typedef enum {
  AMBE_noboard, AMBE_initialized, AMBE_INETonly, AMBE_HFonly, AMBE_INETfirst, AMBE_HFfirst
} tambe_mode;


#define DSTAR_DECODE_TO		250	// 5 seconds (minimum 8 = length of a header)


extern	tfunction dgl_function;		// polled dongle function (for main-loop)

unsigned int dgl_init(void);

int	dgl_is_inuse(void);

void	dgl_start_transmit(void);	// manual start (for testing)
void	dgl_stop_transmit(void);	// manual stop

int	dgl_can_encode_by_ptt(void);
int	dgl_is_encoding(void);		// true, if MicPTT used

void	dgl_get_header(tds_header *target);
void	dgl_ProcessHdr(const tds_header *RxHeader);

unsigned char dgl_copyvoice(tds_voicedata *dest);


#define	CONFIG_C2_SIZE		(40)	// 40 byte ControlFlags + Route + MyCall
#define	CONFIG_C3_SIZE		(20)	// 20 byte ASCII-Text


char *	cfg_read_c2(char *config_buffer);
void	cfg_write_c2(const char *config_data);


char *	cfg_read_c3(char *config_buffer);
void	cfg_write_c3(const char *config_data);


#endif // DONGLE_H_
