/*
 * rptr_func.c
 *
 * realtime controlling openDV (D-Star) operation of DV-RPTR
 *
 *  Created on: 26.07.2011 fomr opendv_func
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

#ifndef DSTAR_FUNC_H_
#define DSTAR_FUNC_H_

#include "dv_dstar.h"
#include "defs.h"


#define DSTAR_BEFOREFRAMEENDS	16	// 16 bit-time before gmsk-modulator-data runs out
#define DSTAR_DECODE_TO		250	// 5 Sekunden (Minimum-Wert ist 8 = eine Header-Länge)

#define DSTAR_DATAIDLETIMEOUT	20	// Frames w/o new data in slow-data-only tx mode


typedef void (*tdstar_slowdatrxf)(const char *, unsigned int);
typedef char *(*tdstar_slowdattxf)(unsigned int);	// Transmit Slow Data Fct.

typedef void (*tdstar_function)(void);


void	dstar_init_data(void);

void	dstar_init_hardware(void);	// Init des DStar-Modus
void	dstar_exit_hardware(void);	// Verlassen des DSTar-Modus

void	dstar_init_header(const tds_header *header);
void	dstar_update_mycall(const char *MyCallSign);
void	dstar_update_route(const char *NewRoute);

void	dstar_update_dest(const char *NewDest);
void	dstar_update_depart(const char *NewDepart);
void	dstar_update_yourcall(const char *NewYourCall);

void	dstar_set_emergency(void);
void	dstar_clr_emergency(void);


// Standby: Enable AMBE in Standby Mode, GMSK-Timer stopped
// Call direct before or after EnableInts
void	dstar_standby(void);

void	dstar_receive(void);

// Enable Voice-Tranmitting, starts with tx-delay, preamble and header
// Transmit 60 Bytes between 2 sync cycles (set with "setSlowData()")
void	dstar_transmit(void);

// Enable Silence-Tranmitting, starts with tx-delay, preamble and header
// Transmit 60 Bytes between 2 sync cycles (set with "setSlowData()")
// If no data available, DVcmdRmtTXoff -> DVUP
void	dstar_transmit_data(void);


int	dstar_newheader(void);
unsigned long *dstar_getheader(void);

int	dstar_new_rxstate(void);
int	dstar_channel_idle(void);	// No dstar-signals heared

tDV_RXstate dstar_get_rxstate(void);

int	dstar_newrxSlowData(void);	// Returns NoOf Received SlowDataFrames
char	*dstar_getrxSlowData(void);	// Read Last SlowDataFrame, Inc Read-Counter

void	dstar_setslowdatarxfct(tdstar_slowdatrxf HandleFct);
void	dstar_setslowdatatxfct(tdstar_slowdattxf HandleFct);

void	dstar_setstopfunction(tdstar_function HandleFct);


#endif // DSTAR_FUNC_H_

