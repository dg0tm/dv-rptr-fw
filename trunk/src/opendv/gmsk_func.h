/*
 * gmsk_func.h
 *
 * function-prototypes for half-duplex GMSK modulation / decoding.
 *
 *  Created on: 30.03.2009
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


#ifndef GMSK_FUNC_H_
#define GMSK_FUNC_H_

// *** Common Definitions ***
#define GMSK_BITRATE		4800

// *** GMSK-Modulator ***
#define GMSK_OVERSAMPLING	4		// >= 4, only even values allowed!

#define GMSK_STDTXDELAY		30		// Time in [ms] to wait after gmsk_transmit()
#define GMSK_TXDRESOLUTION	2		// 2ms Steps

// *** GMSK-Demodulator ***
#define GMSK_BITSAMPLING	4

#define GMSK_DCDSHIFTDEC	6		// 64igstel Mittelwert


#define	GMSK_DEFAULT_BW		0x3F00		// 100% = 0x4000

// *** Debugging ***
#ifdef DEBUG
//#define DEMOD_DBG_BITBUFSIZE	11		// Max. 10 Bit until Toggle
//#define DEBUG_CLOCK_RECOVER			// output filtered biased AF-input
#endif



// This Handler can be used to trigger the next data
// Set result to new bitcount (alert)
// alert = 1: after reload next data
// alert = databitlen-1: before reload
// alert > databitlen-1: (invalid) no trigger
typedef void (*tgmsk_reloadfunc)(void);

// Demodulator-Handler, set by gmsk_sethandler()
typedef void (*tgmsk_func)(void);


void	gmsk_init(void);
void	gmsk_exit(void);

void	gmsk_stoptimer(void);

void	gmsk_transmit(unsigned long *data, unsigned int length_in_bits, unsigned int alertbitpos);
void	gmsk_modulator_stop(void);

void	gmsk_set_mod_hub(short int level);	// free modulation-level adjustment
// hint: negative values -> inverted

void	gmsk_set_txdelay(unsigned int millisec);
void	gmsk_set_reloadfunc(tgmsk_reloadfunc newfunc);

unsigned int gmsk_get_txdelay(void);

unsigned int gmsk_bitsleft(void);
int	gmsk_nextpacket(void);		// Kann nächstes Paket angehängt werden?


void	gmsk_demodulator_start(void);

void	gmsk_demodulator_invert(int invert);

void	gmsk_set_synchandler(tgmsk_func syncstart, tgmsk_func syncstop, tgmsk_func framesync);
void	gmsk_set_receivefkt(tgmsk_func fkt_received);
void	gmsk_set_receivebuf(unsigned long *rxbuf, int bit_len);

int	gmsk_channel_idle(void);

void	gmsk_adjusttxperiod(signed int correction);

#endif // GMSK_FUNC_H_

