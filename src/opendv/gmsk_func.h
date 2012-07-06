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

#define GMSK_STDTXDELAY		120		// Time in [ms] to wait after gmsk_transmit()
#define GMSK_TXDRESOLUTION	2		// 2ms Steps

// *** GMSK-Demodulator ***
#define GMSK_BITSAMPLING	4

#define GMSK_DCDSHIFTDEC	6		// 64igstel Mittelwert


#define	GMSK_DEFAULT_BW		0x3F00		// 100% = 0x4000

// *** Debugging ***
#ifdef DEBUG
//#define DEMOD_DBG_BITBUFSIZE	32
//#define DEMOD_DBG_DCBUFSIZE	32
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

// Check-Pattern function
typedef int (*tpattern_func)(unsigned int, unsigned int);
// this function is called every received bit and shoult check the incoming bitstream (1st param)
// for a valid pattern. The 2nd parameter is the bitcounter
// to reset bitcounter return a value != 0:
// 1 = reset bitcounter, synchronize if not sync
// 2 = reset bitcounter, force a re-synchronize
// -1 = reset bitcounter, stop receiving bit into a buffer
// Attention:
// Don't forget to define a new buffer (gmsk_set_receivebuf()), if you return > 0!!!



void	gmsk_init(void);
void	gmsk_exit(void);

#if (DVRX_TIMER_CH == DVTX_TIMER_CH)
void	gmsk_stoptimer(void);
#endif

void	gmsk_transmit(const unsigned int *data, unsigned int length_in_bits, unsigned int alertbitpos);
void	gmsk_modulator_stop(void);

void	gmsk_set_mod_hub(short int level);	// free modulation-level adjustment
// hint: negative values -> inverted

void	gmsk_set_txdelay(unsigned int millisec);
void	gmsk_set_reloadfunc(tgmsk_reloadfunc newfunc);

unsigned int gmsk_get_txdelay(void);

unsigned int gmsk_bitsleft(void);
int	gmsk_nextpacket(void);		// Kann nächstes Paket angehängt werden?


void	gmsk_demodulator_start(void);
void	gmsk_demodulator_stop(void);

void	gmsk_demodulator_invert(int invert);
void	gmsk_demodulator_toggle_invert(void);

void	gmsk_set_patternfunc(tpattern_func patternhandler);
void	gmsk_set_unlockedfunc(tgmsk_func unlock_handler);

void	gmsk_set_receivefkt(tgmsk_func fkt_received);
void	gmsk_set_receivebuf(unsigned int *rxbuf, int bit_len);


int	gmsk_channel_idle(void);

unsigned short gmsk_get_rssi_avrge(void);
unsigned short gmsk_get_rssi_current(void);


void	gmsk_adjusttxperiod(signed int correction);

#endif // GMSK_FUNC_H_

