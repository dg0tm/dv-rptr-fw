/*
 * gmsk_func.c
 *
 * implements functions for half-duplex GMSK modulation and decoding
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
 *
 *
 * GMSK Modulator und Demodulator
 * Die zu sendenden Daten werden Bitweise ausgegeben, LSB first
 * (da so vom AMBE direkt zu übernehmen).
 *
 * ToDo:
 * - Demodulator verbessern, Einbezienung des vorherigen und nachfolgenden Bits
 * - Carrier-Detection
 * - Receive Tiefpass
 * - Receive PLL Finetuning
 *
 * Report:
 * 2011-07-01	TX-Delay / Modulator-Werte werden nicht mehr gmsk_init() auf Default gesetzt
 * 2011-07-27	Duplex-Version, using 2 Timer
 * 2011-08-28	Fehlerbehebung Duplex
 * 2011-09-06	Additional Interrupt-Handler keeps critical Timer-based ADC-start, DAC-out
 * 		with a minimum of jitter in the case of duplex operation
 * 2011-09-18	new gaussian shaping filter (BT=0.5) from G4KLX
 * 2011-09-19	modified shaping function (180° phase shifted)
 * 		replace dsp16_vect_copy() by dsp16_vect_move() if mem is overlapped
 * 2011-09-20	remove possible raise conditions on adc-buffer, new demodulator code
 * 		cleanup code (remove old demod function)
 * 2011-09-21	working sample clock-recover-algorithm
 * 2011-09-22	improved edge detect an bit-restore, need to be tested
 * 2011-10-11	fixing memory overflow on weak signals, if a sync-pattern detected
 * 2011-10-30	new PATTERN check API - handle pattern-matches oudside this module
 * 2011-10-31	bugfix on setting new RX PLL clock fixed
 * 2011-11-05	add a receive-pll-unlock handler: called if reception is faded out.
 * 		use a averaged DC-value (from raw AD-data) to compensate DC-jumps at the
 * 		start of a new transmission (only while unlocked or in sync)
 * 		unlocked state: uses +/- 1/4 bit-length-jumps to shift in phase quickly
 * 2012-07-03	gmsk_demodulator_toggle_invert(): Quick toggle inversion
 */


#include "gmsk_func.h"

#include "hw_defs.h"

#include "gpio_func.h"		// Set/Clr Debug
#include "adc_func.h"		// Receive via internal ADC
#include "dac_func.h"		// Transmit via external DAC

#include <intc.h>
#include <dsp.h>


// dsp16_vect_copy() [=memcpy()] can't be used for overlapping vectors! Under some
// conditions, *vect2 is overwritten!
#define dsp16_vect_move(v1, v2, size) memmove((v1), (v2), (size)*sizeof(dsp16_t))



#define GMSK_RX_TIMER		AVR32_TC.channel[DVRX_TIMER_CH]
#define GMSK_RX_TIMER_IRQ	(AVR32_TC_IRQ0+DVRX_TIMER_CH)
#define GMSK_GETSAMPLE_IRQ	AVR32_ADC_IRQ

#define GMSK_TX_TIMER		AVR32_TC.channel[DVTX_TIMER_CH]
#define GMSK_TX_TIMER_IRQ	(AVR32_TC_IRQ0+DVTX_TIMER_CH)


#define GMSK_SPSRATE		(GMSK_BITRATE*GMSK_BITSAMPLING)		// Receiver
#define GMSK_RCDEFAULT		(((MASTERCLOCK+1)/2)/GMSK_SPSRATE)
#define GMSK_RCMODULO		(((MASTERCLOCK+1)/2)%GMSK_SPSRATE)
#define GMSK_RCSTDPHASE		((GMSK_RCMODULO*0x10000L)/GMSK_SPSRATE)	// Nachkommaanteil.
#define GMSK_PLLDEFAULT		((GMSK_RCDEFAULT*0x10000L)|GMSK_RCSTDPHASE)

#define GMSK_CLOCK		(GMSK_BITRATE*GMSK_OVERSAMPLING)	// Transmitter
#define GMSK_MOD_DEFAULT	(((MASTERCLOCK+1)/2)/GMSK_CLOCK)
#define GMSK_MOD_MODULO		(((MASTERCLOCK+1)/2)%GMSK_CLOCK)
#define GMSK_MOD_PHASE		((GMSK_MOD_MODULO*0x10000L)/GMSK_CLOCK)


#define TIMER_CMR_VALUE	( AVR32_TC_NONE << AVR32_TC_BSWTRG_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_BEEVT_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_BCPC_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_BCPB_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_ASWTRG_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_AEEVT_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_ACPC_OFFSET | \
    AVR32_TC_NONE << AVR32_TC_ACPA_OFFSET | \
    1 << AVR32_TC_WAVE_OFFSET | \
    AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET | \
    FALSE << AVR32_TC_ENETRG_OFFSET | \
    AVR32_TC_EEVT_TIOB_INPUT << AVR32_TC_EEVT_OFFSET | \
    AVR32_TC_EEVTEDG_NO_EDGE << AVR32_TC_EEVTEDG_OFFSET | \
    FALSE << AVR32_TC_CPCDIS_OFFSET | \
    FALSE << AVR32_TC_CPCSTOP_OFFSET | \
    AVR32_TC_BURST_NOT_GATED << AVR32_TC_BURST_OFFSET | \
    0 << AVR32_TC_CLKI_OFFSET | \
    AVR32_TC_TCCLKS_TIMER_CLOCK2 << AVR32_TC_TCCLKS_OFFSET )



// *** DEMODULATOR ***

/*! \name GMSK Demodulator Interrupt-Functions
 */
//! @{

// FIR Filter used by the Demodulator:

/* same as in DV-Modem (C5), calculatet with FilterExpress
A_ALIGNED const dsp16_t RXLowPassCoeff[] = {
  DSP16_Q(0.0198974609375),
  DSP16_Q(0.025848388671875),
  DSP16_Q(0),
  DSP16_Q(-0.043792724609375),
  DSP16_Q(-0.05859375),
  DSP16_Q(0),
  DSP16_Q(0.127838134765625),
  DSP16_Q(0.261138916015625),
  DSP16_Q(0.3179931640625),
  DSP16_Q(0.261138916015625),
  DSP16_Q(0.127838134765625),
  DSP16_Q(0),
  DSP16_Q(-0.05859375),
  DSP16_Q(-0.043792724609375),
  DSP16_Q(0),
  DSP16_Q(0.025848388671875),
  DSP16_Q(0.0198974609375)
};
*/

// calculated by Torsten Schultze DG1HT
A_ALIGNED const dsp16_t RXLowPassCoeff[] = {
  DSP16_Q(0.019823264989714),
  DSP16_Q(0.025848388671875),
  DSP16_Q(0),
  DSP16_Q(-0.043792724609375),
  DSP16_Q(-0.05859375),
  DSP16_Q(0),
  DSP16_Q(0.127838134765625),
  DSP16_Q(0.261138916015625),
  DSP16_Q(0.3179931640625),
  DSP16_Q(0.261138916015625),
  DSP16_Q(0.127838134765625),
  DSP16_Q(0),
  DSP16_Q(-0.058593752349941),
  DSP16_Q(-0.041093832349941),
  DSP16_Q(0),
  DSP16_Q(0.026812695985862),
  DSP16_Q(0.018417607438616)
};

/*
// Filter-Coeff calculated by Torsten Schultze DG1HT
A_ALIGNED const dsp16_t RXLowPassCoeff[] = {
  DSP16_Q(0.000720827595098),  DSP16_Q(-0.000000000000000),  DSP16_Q(-0.000925046533099),
  DSP16_Q(-0.001626164182341), DSP16_Q(-0.001468460185275),  DSP16_Q(0.000000000000000),
  DSP16_Q(0.002426562140692),  DSP16_Q(0.004367597568280),   DSP16_Q(0.003889284576222),
  DSP16_Q(-0.000000000000000), DSP16_Q(-0.005974642332895),  DSP16_Q(-0.010326122606575),
  DSP16_Q(-0.008856412916505), DSP16_Q(0.000000000000000),   DSP16_Q(0.012823264989714),
  DSP16_Q(0.021730760111537),  DSP16_Q(0.018417607438616),   DSP16_Q(-0.000000000000000),
  DSP16_Q(-0.026812695985862), DSP16_Q(-0.046474506613524),  DSP16_Q(-0.041093832349941),
  DSP16_Q(0.000000000000000),  DSP16_Q(0.072660711830346),   DSP16_Q(0.156979998630249),
  DSP16_Q(0.224441556965367),  DSP16_Q(0.250199423719791),   DSP16_Q(0.224441556965367),
  DSP16_Q(0.156979998630249),  DSP16_Q(0.072660711830346),   DSP16_Q(0.000000000000000),
  DSP16_Q(-0.041093832349941), DSP16_Q(-0.046474506613524),  DSP16_Q(-0.026812695985862),
  DSP16_Q(-0.000000000000000), DSP16_Q(0.018417607438616),   DSP16_Q(0.021730760111537),
  DSP16_Q(0.012823264989714),  DSP16_Q(0.000000000000000),   DSP16_Q(-0.008856412916505),
  DSP16_Q(-0.010326122606575), DSP16_Q(-0.005974642332895),  DSP16_Q(-0.000000000000000),
  DSP16_Q(0.003889284576222),  DSP16_Q(0.004367597568280),   DSP16_Q(0.002426562140692),
  DSP16_Q(0.000000000000000),  DSP16_Q(-0.001468460185275),  DSP16_Q(-0.001626164182341),
  DSP16_Q(-0.000925046533099), DSP16_Q(-0.000000000000000),  DSP16_Q(0.000720827595098)
};
*/


// Defines (some depends on filter-length)
#define RXFILTCOEFFSIZE		(sizeof(RXLowPassCoeff)/sizeof(dsp16_t))
#define DEMOD_ADC_SIZE		(RXFILTCOEFFSIZE+GMSK_BITSAMPLING-1)

// a buffer of 2 bittimes filtered samples:
#define DEMOD_FILT_SIZE		(2*GMSK_BITSAMPLING)
// by define, the algothim shifts the zero-crossing to sample 0 of the new 4 (pos 4):
#define DEMOD_ZEROCR_POS	(GMSK_BITSAMPLING)
// a bit-max value should appear 1/2bittime later ZC (on position 6):
#define DEMOD_BITMAX_POS	(DEMOD_ZEROCR_POS+(GMSK_BITSAMPLING/2))	// last sample

// Sample-Buffers:

// first buffer to capture samples one bittime w/o affecting buffers needed for
// calculations (prevent raised conditions on buffer)
A_ALIGNED dsp16_t demod_capture[GMSK_BITSAMPLING];
// after finishing capture GMSK_BITSAMPLING times samples the capture buffer
// is copied to the end of demod_adcin[]. This buffer contains enough data for
// filtering (FIR)
A_ALIGNED dsp16_t demod_adcin[DEMOD_ADC_SIZE];
// demod_filt[] hold the filtered values (at the end) and a single sample from
// last bit
A_ALIGNED dsp16_t demod_filt[DEMOD_FILT_SIZE];	// Gefilterte ADC Werte (FIR Lowpass) etc

//A_ALIGNED dsp16_t demod_in[GMSK_BITSAMPLING];	// Bitwerte ohne DC für Entscheider

#ifdef DEMOD_DBG_BITBUFSIZE
A_ALIGNED dsp16_t demod_backbuf[DEMOD_DBG_BITBUFSIZE];
#endif
#ifdef DEMOD_DBG_DCBUFSIZE
A_ALIGNED dsp16_t demod_backdcbuf[DEMOD_DBG_DCBUFSIZE];
#endif


Union32 gmsk_rxpll_clk;
Union32 gmsk_rxpll_accu;


typedef void (*tgmsk_clockfkt)(S32);

typedef enum {
  DEMOD_unlocked, DEMOD_sync, DEMOD_locked, DEMOD_faded
} tDemodState;


// Demodulator-Handler Functions:
tgmsk_clockfkt	demod_clockrecover;	// Funktion zum Nachführen der RX-Clock PLL

tpattern_func	demod_pattern_fkt;
tgmsk_func	demod_unlocked_fkt;	// called, if RX-PLL detects no valid edges anymore
tgmsk_func	demod_received_fkt;

tDemodState	demod_state;


int	demod_bitphasecnt;	// counting GMSK_BITSAMPLING times up
U32	demod_shr;		// Demodulations-Bit-Shift-Register


#define DEMOD_BITSET_MASK	0x80000000L
U32	demod_pos_level = 0x00000000;		// "null"
U32	demod_neg_level = DEMOD_BITSET_MASK;	// "eins"

U32	demod_rxbitcnt;		// Receive Bit Counter
U32	demod_rxsize;		// Bit-Size of Receivebuffer, rxptr pointed to
U32	*demod_rxptr;		// Receive Buffer Pointer


// Demodulator Taktrückgewinnung-Entscheiderhilfen
// permanent values helping to recover clock
S16	gmsk_phase_corr;	// use this to do a single phase corr.
// attention: value should be much smaller than 30MHz/19200!

int	demod_korrcnt;		// counter of bits, since last call of clock_recover()
S32	demod_lastkorr;		// stored correction after call of clock_recover()
S32	demod_korr;		// use this th have a filtered value of clock drift
S32	demod_drift;		// avaraged differences between 2 corr values
int	demod_state_cnt;

U16	rssi_current;		// current rssi measure
U16	rssi_avrge;		// averaged rssi on data-block end
int	rssi_cnt;
U32	rssi_block;		// sum of cnt x rssi_current, reset on databock-set

dsp32_t	demod_dclevel;		// mid-level of noise and first sync


// Demodulator Int simple version

#define ADC_Middle_Voltage	(510<<4)	// default / real mid-level
#define DCLEVEL_AVRGE_FAK	0.025		// for averaging a DC-Value

// ToDo:
// -optimize average values


#define Average16(var_x, new_x, FAK)	\
  { var_x = dsp16_op_mul(var_x, DSP16_Q(1.0-FAK)) + dsp16_op_mul(new_x, DSP16_Q(FAK)); }

#define Average32(var_x, new_x, FAK)	\
  { var_x = dsp32_op_mul(var_x, DSP32_Q(1.0-FAK)) + dsp32_op_mul(new_x, DSP32_Q(FAK)); }


#define gmsk_calc_zerocorr(zero0, zero1)	\
  ((S32)(zero1)*GMSK_RCDEFAULT / (zero0-zero1))

// demod_filter_to() define: it can easy replaced by a copy-function for testing
#define demod_filter_to(dest)	 \
  dsp16_filt_fir(dest, demod_adcin, DEMOD_ADC_SIZE, (dsp16_t *)RXLowPassCoeff, RXFILTCOEFFSIZE)


/* demod_clockrecover() functions
 * parameter correction: times (in TIMER-ticks) of phase fails
 * possible values are from -1562 to 6250 (sample period: 30MHz/19200)
 * global demod_korrcnt: number of bits (non-alternating) since last call
 * These function modify the gmsk_rxpll_clk period variable (32bit) to get in sync
 * with rx data stream.
 *
 * Additional, a single shift of GMSK_RX_TIMER.rc can be used for fast syncing.
 *
 */

#define DEMOD_THERSHOLD_PHASE	(GMSK_RCDEFAULT*3/4)	// compare-value correction
#define DEMOD_BAD_EDGES		50	// Locked, if > -> faded
#define DEMOD_GOOD_EDGES	50	// Faded, if > -> locked
#define DEMOD_NOVALID_EDGES	300	// Faded, if > -> unlocked

#define PLL_LOCK_KORR_FAK	0.050	//slower, test [0.050 =  ok].
#define PLL_LOCK_DRIFT_FAK	0.005
#define DEMOD_THERSHOLD_PLL	(GMSK_RCDEFAULT<<11)

static void gmsk_faded_fkt(S32 correction);	// Forward

#define GMSK_PLLMINPERIOD	(GMSK_PLLDEFAULT-(GMSK_PLLDEFAULT/1000))	// ^= 4804.8baud
#define GMSK_PLLMAXPERIOD	(GMSK_PLLDEFAULT+(GMSK_PLLDEFAULT/1000))	// ^= 4795.2baud

/* generell scheint nach dem Ende von Lock ein Fehler aufzutauchen: gmsk_rxpll_clk immer etwas zu lang
 *
 */
static void gmsk_locked_fkt(S32 correction) {
  U32 new_rxpll_period = gmsk_rxpll_clk.u32;
  S32 clock_difference = ((correction-demod_lastkorr)<<16) / demod_korrcnt;
  Average32(demod_korr,  correction<<16, PLL_LOCK_KORR_FAK);
  Average32(demod_drift, clock_difference, PLL_LOCK_DRIFT_FAK);
  gmsk_phase_corr = __builtin_sats(correction, 4, 7);
  /*
  if (abs(demod_korr) > DEMOD_THERSHOLD_PLL) {
    debugpin_set(DEBUG_PIN2);
    debugpin_clr(DEBUG_PIN2);
  }
  */
  if (abs(demod_korr) < 0x00000FFF) {
    // noch funktioniert es nicht wie gedacht...
//    new_rxpll_period += __builtin_sats(demod_korr, 0, 11);	// Fine
  } else {
    new_rxpll_period +=  __builtin_sats(demod_korr, 13, 11);	// ok f�r +/- 20Hz
  }
  if (new_rxpll_period > GMSK_PLLMAXPERIOD)
    gmsk_rxpll_clk.u32 = GMSK_PLLMAXPERIOD;
  else if (new_rxpll_period < GMSK_PLLMINPERIOD)
    gmsk_rxpll_clk.u32 = GMSK_PLLMINPERIOD;
  else
    gmsk_rxpll_clk.u32 = new_rxpll_period;
//  } // fi correct clock
  if (abs(correction) > (DEMOD_THERSHOLD_PHASE)) {
    if (++demod_state_cnt > DEMOD_BAD_EDGES) {
      demod_clockrecover = &gmsk_faded_fkt;
      demod_state = DEMOD_faded;
      demod_state_cnt = 0;
    }
  } else if (demod_state_cnt > 0) {
    demod_state_cnt--;
  }
}


#define PLL_SYNC_KORR_FAK	0.05
#define DEMOD_EDGES_TO_LOCK	64

static void gmsk_sync_fkt(S32 correction) {
//  S32 clock_difference = ((correction-demod_lastkorr)<<16) / demod_korrcnt;
  Average32(demod_korr,  correction<<16, PLL_SYNC_KORR_FAK);
//  Average32(demod_drift, clock_difference, PLL_SYNC_KORR_FAK);
  gmsk_phase_corr = __builtin_sats(correction, 1, 8);
  // change state to sync if we are in phase:
  if ( ++demod_state_cnt > DEMOD_EDGES_TO_LOCK) {
    if (abs(demod_dclevel) < 0x00800000) {	// DC-Level < 128 absolute (^= AD-DC-Offest 32)
      demod_clockrecover = gmsk_locked_fkt;
      demod_state        = DEMOD_locked;
      demod_state_cnt    = 0;
    } else {
      demod_state_cnt -= DEMOD_EDGES_TO_LOCK/2;
    }
  }
//  dac_modulate(demod_korr/4);
}


// PLL-Korrektur-Funktion (3 Versionen):
// Bekommt Wert, der der Abweichung in Timer-Schritten entspricht
// Funktion wird bei jeder Flanke aufgerufen und berechnet einen Wert für die DCD.
static void gmsk_unlocked_fkt(S32 correction) {
  demod_korr  = 0;
  demod_drift = 0;
  demod_state_cnt = 0;
  // Schnell hin zum NDG / jitter fast to catch ZC
  if (correction > (3*GMSK_RCDEFAULT/2)) {
    demod_bitphasecnt = (demod_bitphasecnt+1)%GMSK_BITSAMPLING;
  } else if (correction < (-3*GMSK_RCDEFAULT/2)) {
    demod_bitphasecnt = (demod_bitphasecnt+(GMSK_BITSAMPLING-1))%GMSK_BITSAMPLING;
  } else
    gmsk_phase_corr = __builtin_sats(correction, 0, 9);
}


static void gmsk_demod_unlock(void) {
  demod_clockrecover = gmsk_unlocked_fkt;	// Standard-Recover-Fkt
  demod_state = DEMOD_unlocked;
  gmsk_rxpll_clk.u32  = GMSK_PLLDEFAULT;	// Set PLL-Clock to default
  gmsk_rxpll_accu.u32 = 0;
  demod_state_cnt = 0;
}


static void gmsk_faded_fkt(S32 correction) {
//  S32 clock_difference = ((correction-demod_lastkorr)<<16) / demod_korrcnt;
  Average32(demod_korr,  correction<<16, PLL_LOCK_KORR_FAK);
//  Average32(demod_drift, clock_difference, PLL_LOCK_DRIFT_FAK);
  if (abs(correction) < DEMOD_THERSHOLD_PHASE) {
    gmsk_phase_corr = __builtin_sats(correction, 4, 6);
    if (++demod_state_cnt > DEMOD_GOOD_EDGES) {
      gmsk_rxpll_clk.u32 = GMSK_PLLDEFAULT;
      demod_clockrecover = gmsk_sync_fkt;
      demod_state = DEMOD_sync;
      demod_state_cnt = 0;
    }
  } else {
    if (--demod_state_cnt < (-DEMOD_NOVALID_EDGES)) {
      gmsk_demod_unlock();
      if (demod_unlocked_fkt != NULL) {
	demod_rxbitcnt = 0;
	demod_rxptr    = NULL;
	demod_rxsize   = 0;
	demod_unlocked_fkt();
      } // fi have handler -> breakup receive
    }
  }
}



static __inline void gmsk_demod_sync(void) {
  demod_clockrecover = &gmsk_sync_fkt;
  gmsk_rxpll_clk.u32 = GMSK_PLLDEFAULT;
  demod_state = DEMOD_sync;
  demod_state_cnt = 0;
}


// if the edge is not on correct position, we must keep an eye on
// the previous and current bit-max.
static __inline void demod_correct_bits(int edge_zc) {
  U32 bits = demod_shr & 0xBFFFFFFF;
  if (edge_zc != DEMOD_ZEROCR_POS) {
    // ToDo
    // wo ist der Fehler!
    if (demod_filt[edge_zc-(GMSK_BITSAMPLING/2)] > 0) {
      bits |= (demod_pos_level>>1);
    } else {
      bits |= (demod_neg_level>>1);
    }
  }
  //demod_shr = bits; // <- sobald drin geht net
}


/* ToDo Demodulator Int:
 * dcd_level relativ zu Gesamtlevel
 * ISI im Nulldurchgang berücksichtigen für Korrekturwerte
 * Letztes Bit ändern, wenn plausibel
 * Betrachtung nächstes Bit (ein Bit hinterher decodieren)
 * PLL-Shift begrenzen z.B. 1000ppm
 * Soft-Decision-Werte ermitteln (für AMBE Soft-Dec Modus)
 */


/* gmsk_samplestart_int()
 * Timer-Interrupt, high priority. Starts AD-conversion and update RX-PLL
 * keep it as short as possible.
 */
INTERRUPT_FUNC gmsk_samplestart_int(void) {
  dsp16_t adc_val;
  GMSK_RX_TIMER.sr;				// Acknowledge IRQ
  adc_val = (HFIN << 4) - ADC_Middle_Voltage;	// Store last ADC (as a signed 14 bit value)
  adc_startconversion();			// Start next ADC
  // Update Period-Length (PLL-Value for exact RX-Clock):
  GMSK_RX_TIMER.rc = gmsk_rxpll_clk.u16[0] + gmsk_rxpll_accu.u16[0];
  if (demod_bitphasecnt == 0) {			// shifting-correction
    // alternate GMSK_RX_TIMER.rc here for phase shift:
    GMSK_RX_TIMER.rc += gmsk_phase_corr;
    gmsk_phase_corr = 0;
  } // fi
  gmsk_rxpll_accu.u16[0] = 0;	// Clear Alternate-Bit
  gmsk_rxpll_accu.u32 += gmsk_rxpll_clk.u16[1];
#ifdef DEBUG_CLOCK_RECOVER
  dac_modulate(demod_filt[demod_bitphasecnt+2]<<1);
#endif
  demod_capture[demod_bitphasecnt] = adc_val;	// fill up sample buffer
  if (++demod_bitphasecnt >= GMSK_BITSAMPLING ) {
    debugpin_set(DEBUG_PIN1);
    demod_bitphasecnt = 0;
    // copy new samples to the end of demod_adcin:
    dsp16_vect_copy(&demod_adcin[DEMOD_ADC_SIZE-GMSK_BITSAMPLING], demod_capture, GMSK_BITSAMPLING);
    AVR32_ADC.ier = HFDATA_INT_MASK;
  } // fi
  Average32(demod_dclevel, adc_val<<16, DCLEVEL_AVRGE_FAK);
  debugpin_clr(DEBUG_PIN1);
}



// simplified, PLL-recover w/o filter

INTERRUPT_FUNC gmsk_processbit_int(void) {
  S32 clockcorr;
  int cnt, flanke;
  // disable interrupt handling (it is enabled by timer interrupt if a bit-sampling finished)
  AVR32_ADC.idr	= HFDATA_INT_MASK;

  // RSSI /SQL-Line measurement - every 32 bits one additional measure:
  if ((demod_rxbitcnt&0x1F) == 0) {	// enable RSSI measure every 32 bit times
    adc_disable_hfin();
    adc_enable_rssi();
    adc_startconversion();			// Start AD conversion of RSSI /SQL
    // a conversion needs 14 ADC-Clocks (@4MHz -> 210 CPU clocks)
//    debugpin_set(DEBUG_PIN2);
  } // fi trigger reading RSSI

  // Filter FIR on demod_adcin -> Signal to demod_filt (first samples are from previous bit)
  demod_filter_to(&demod_filt[GMSK_BITSAMPLING]);

  // free space for new ADC data:
  dsp16_vect_move(demod_adcin, &demod_adcin[GMSK_BITSAMPLING], DEMOD_ADC_SIZE-GMSK_BITSAMPLING);

  // hard desition on bit-max position (should be last sample)
  // store bit in shift-register (SHR)
  demod_shr >>= 1;
  if (demod_state > DEMOD_sync) {
    if (demod_filt[DEMOD_BITMAX_POS] > 0) {	// hard decision decoding
      demod_shr |= demod_pos_level;		// pos. Auslenkung -> i.d.R. "0" empfangen
    } else {	// low_level
      demod_shr |= demod_neg_level;		// neg. Auslenkung -> i.d.R. "1" empfangen
    }
  } else {
    if ((demod_filt[DEMOD_BITMAX_POS]-(demod_dclevel>>16)) > 0) { // hard decision, eliminate DC
      demod_shr |= demod_pos_level;		// pos. Auslenkung -> i.d.R. "0" empfangen
    } else {	// low_level
      demod_shr |= demod_neg_level;		// neg. Auslenkung -> i.d.R. "1" empfangen
    }
  }
  // check, if bit is not the same like the bit before
  // if true a edge must be between these bits
  demod_korrcnt++;
  // detect phase of zero-crossings (ZC)
  clockcorr = 0;
  flanke = DEMOD_ZEROCR_POS;		// prevent warning
  if ( ((demod_shr&0xC0000000)==0x80000000) || ((demod_shr&0xC0000000)==0x40000000) ) {
    for (cnt = DEMOD_ZEROCR_POS; cnt < DEMOD_FILT_SIZE; cnt++) {
      if ((demod_filt[cnt-1]&0x8000) != (demod_filt[cnt]&0x8000)) {
	// find the "largest" edge (in noisy signals):
	S32 zc_slope = abs(demod_filt[cnt] - demod_filt[cnt-1]);
	if (zc_slope > clockcorr) {
	  flanke = cnt;		// position of an edge found
	  clockcorr = zc_slope;
  //	continue;	// un-comment: first edge only.
	}
      } // fi sign-compare
    } // rof
  } // fi
  if (clockcorr > 0) {	// a edge is in the new bit-samples
    // a ZC should occur on position 1 - than no correction is necssary
    clockcorr = ((flanke-DEMOD_ZEROCR_POS)*GMSK_RCDEFAULT) +
	        gmsk_calc_zerocorr(demod_filt[flanke-1], demod_filt[flanke]);
    // Position 1 is 2 samples before the bitmax-sample, see defines above
    demod_clockrecover(clockcorr);
    demod_correct_bits((clockcorr/GMSK_RCDEFAULT) + DEMOD_ZEROCR_POS);
    demod_korrcnt  = 0;
    demod_lastkorr = clockcorr;
  } // fi edge detected

  // keep samples of current bit:
  dsp16_vect_copy(demod_filt, &demod_filt[DEMOD_FILT_SIZE/2], DEMOD_FILT_SIZE/2);

  // RSSI /SQL-Line measurement - now more than 210 clocks processed...
  if (RSSI_CH_ENABLED()) {
    // Test conversion finished (function runs 210 cpu cycles)?
#ifdef DEBUG
//    if (AVR32_ADC.sr & 0x40) debugpin_clr(DEBUG_PIN2);
#endif
    rssi_current = RSSI_IN;
    adc_disable_rssi();
    adc_enable_hfin();
    rssi_block += rssi_current;
    rssi_cnt++;
    // store value
  } // fi RSSI measure

  // saving of SHR if a pointer define:
  demod_rxbitcnt++;
  if (demod_rxptr != NULL) {
    if ((demod_rxbitcnt&0x1F) == 0 ) {
      *demod_rxptr++ = swap32(demod_shr);
    } else if (demod_rxbitcnt==demod_rxsize) {	// fi Word full
      *demod_rxptr = swap32(demod_shr >> ((demod_rxsize|0x1F)-demod_rxsize+1));
    } // fi last Bits
    if (demod_rxbitcnt >= demod_rxsize) {
      demod_rxbitcnt = 0;
      demod_rxptr = NULL;	// vor ReceiveFkt.
      rssi_avrge = rssi_block / rssi_cnt;
      rssi_cnt   = 0;
      rssi_block = 0;
      if (demod_received_fkt != NULL) demod_received_fkt();
    } // fi fertig
  } // fi valid ptr to store data

  // Pattern checking - every bit
  cnt = demod_pattern_fkt(demod_shr, demod_rxbitcnt);
  // ToDo nach Return BusFetch Error !
  if (cnt > 0) {		// application matches pattern
    demod_rxbitcnt = 0;		// restart bitcouter
    if ((demod_state < DEMOD_sync)||(cnt > 1)) gmsk_demod_sync();
  } else if (cnt < 0) {		// means "Stop receiving now"
    gmsk_demod_unlock();
    demod_rxbitcnt = 0;
    demod_rxptr    = NULL;
    demod_rxsize   = 0;
  }

#ifdef DEMOD_DBG_BITBUFSIZE
  // a additional sample-buffer for debugging:
  dsp16_vect_move(demod_backbuf, &demod_backbuf[1], DEMOD_DBG_BITBUFSIZE-1);
  demod_backbuf[DEMOD_DBG_BITBUFSIZE-1] = demod_filt[DEMOD_BITMAX_POS];
#endif
#ifdef DEMOD_DBG_DCBUFSIZE
  dsp16_vect_move(demod_backdcbuf, &demod_backdcbuf[1], DEMOD_DBG_DCBUFSIZE-1);
  demod_backdcbuf[DEMOD_DBG_DCBUFSIZE-1] = demod_dclevel>>16;
#endif
}


// empty function -> no NULL check every received bit necesary
int no_patternhandler(unsigned int pattern, unsigned int bitpos) {
  return 0;
}

//! @}


// *** Modulator Variables ***

Union32 gmsk_txpll_accu;


// *** Transmitter / Receiver Gauss-FIR Filter and Sample-Buffer ***
#define GaussCoeffsSize		9
#define MOD_IN_SIZE		(GaussCoeffsSize+GMSK_OVERSAMPLING+1)
#define MOD_IN_FSIZE		(GaussCoeffsSize+GMSK_OVERSAMPLING-1)
#define GMSK_POSTAMBLEBITS	(MOD_IN_FSIZE)

/*
// Gaussian filter for a BT of 0.3, calculated with FilterExpress:
A_ALIGNED const dsp16_t GaussCoeffs[GaussCoeffsSize] = {
  DSP16_Q(0.000161675648356901),
  DSP16_Q(0.00490155244379313),
  DSP16_Q(0.0560638566547222),
  DSP16_Q(0.24193157868456),
  DSP16_Q(0.393878633534559),
  DSP16_Q(0.24193157868456),
  DSP16_Q(0.0560638566547222),
  DSP16_Q(0.00490155244379313),
  DSP16_Q(0.000161675648356901)
};
*/
// Gaussian filter for a BT of 0.5, calculated with MATLAB (from G4KLX):
A_ALIGNED const dsp16_t GaussCoeffs[GaussCoeffsSize] = {
  DSP16_Q(0.000304557692183),
  DSP16_Q(0.006860972327255),
  DSP16_Q(0.063476277434485),
  DSP16_Q(0.241183125455494),
  DSP16_Q(0.376350134181166),
  DSP16_Q(0.241183125455494),
  DSP16_Q(0.063476277434485),
  DSP16_Q(0.006860972327255),
  DSP16_Q(0.000304557692183)
};



// *** Transmitter Buffer and Variables ****

A_ALIGNED dsp16_t modulator_in[MOD_IN_SIZE];		// Eingangsbuffer für FIR Filter
A_ALIGNED dsp16_t modulator_out[GMSK_OVERSAMPLING];	// geGaußte DAC Daten
A_ALIGNED dsp16_t dac_out_buffer[GMSK_OVERSAMPLING];	// keep modulator_out double_puffered

U32 gmsk_txdelay = GMSK_STDTXDELAY;
U32 gmsk_overcnt, gmsk_tsr, gmsk_bitcnt, gmsk_bittimer;	// Zählervariablen Modulator
U32 gmsk_bitlen, gmsk_nextbitlen, gmsk_alertpos, gmsk_next_alertpos;
U32 *gmsk_dataptr, *gmsk_nextdataptr;		// Zeiger auf aktuelles Datum

dsp16_t dac_one_val  = DAC_MIDDLE + GMSK_DEFAULT_BW;
dsp16_t dac_zero_val = DAC_MIDDLE - GMSK_DEFAULT_BW;

U32 gmsk_txmod_clk = GMSK_MOD_PHASE;		// Sendetakt variabel f�r Tests

tgmsk_reloadfunc gmsk_reloadhandler;		// Functionvar



/*! \name GMSK Modulator Interrupt-Functions
 */
//! @{

static __inline void gmsk_nextdacval(volatile dsp16_t newval) {
  dsp16_t *newvals = &modulator_in[MOD_IN_SIZE-GMSK_OVERSAMPLING];
  dsp16_vect_move(modulator_in, &modulator_in[GMSK_OVERSAMPLING], MOD_IN_SIZE-GMSK_OVERSAMPLING);
  while (newvals < &modulator_in[MOD_IN_SIZE]) {
    *newvals++ = newval;
  } // ehliw
}


static __inline void gmsk_stop_txtimer(void) {
  GMSK_TX_TIMER.ccr = AVR32_TC_CLKDIS_MASK;	// Stop Timer
  GMSK_TX_TIMER.idr = 0xFF;
  GMSK_TX_TIMER.sr;				// Clear Pending INT-Requests
}


INTERRUPT_FUNC gmsk_begin_int(void);	// forward


INTERRUPT_FUNC gmsk_modulator_int(void) {
  dac_modulate(dac_out_buffer[gmsk_overcnt]);
  GMSK_TX_TIMER.sr;
  GMSK_TX_TIMER.rc = GMSK_MOD_DEFAULT + gmsk_txpll_accu.u16[0];
  gmsk_txpll_accu.u16[0] = 0;	// Clear Alternate-Bit
  gmsk_txpll_accu.u32 += gmsk_txmod_clk;	//GMSK_MOD_PHASE;
  if (++gmsk_overcnt >= GMSK_OVERSAMPLING) {
    dsp16_vect_copy(dac_out_buffer, modulator_out, GMSK_OVERSAMPLING);
    AVR32_SPI.ier = AVR32_SPI_IER_TXEMPTY_MASK;
    gmsk_overcnt = 0;
  } // fi every 4 times (oversampling)
}



INTERRUPT_FUNC gmsk_runidle_int(void) {
  AVR32_SPI.idr = AVR32_SPI_IER_TXEMPTY_MASK;
  if (gmsk_bittimer>0) {
    gmsk_bittimer--;
    // Verschiebe Daten in modulator_in (Vektor für FIR).
    gmsk_nextdacval(DAC_MIDDLE);
    // Gauss-Filter:
    dsp16_filt_fir(modulator_out, modulator_in, MOD_IN_FSIZE, (dsp16_t *)GaussCoeffs, GaussCoeffsSize);
  } else {
    if (gmsk_nextbitlen==0) {			// Kein neues Paket
#if (DVRX_TIMER_CH == DVTX_TIMER_CH)
      if (demod_pattern_fkt != no_patternhandler) {	// set to Demod
	gmsk_demodulator_start();
      } else				// No Sync-Func defined: Stop
#endif
      gmsk_stop_txtimer();
    } else {
      gmsk_dataptr = gmsk_nextdataptr;	// Lade nächstes Datapkt
      gmsk_bitlen  = gmsk_nextbitlen;
    }
#if (DVRX_TIMER_CH == DVTX_TIMER_CH)
    if (gmsk_nextbitlen > 0)
#endif
    {
      GMSK_TX_TIMER.rc = (((MASTERCLOCK+1)/2)*GMSK_TXDRESOLUTION+500)/1000;	// Warte 1ms nach Idle
      INTC_register_interrupt(gmsk_begin_int, GMSK_TX_TIMER_IRQ, DV_MODWRK_INTPRIO);
    }
    gmsk_nextdataptr = NULL;			// Datenzeiger
    gmsk_nextbitlen  = 0;
  } // esle (finish fading)
}


INTERRUPT_FUNC gmsk_calcnextbit_int(void) {
  AVR32_SPI.idr = AVR32_SPI_IER_TXEMPTY_MASK;
  AVR32_SPI.sr;
  // Setting Timer-Period - modulate to a exact Baudrate
  dsp16_t mod_new = dac_zero_val;

  // Daten laden, Pointer, Counter:
  if (gmsk_bitcnt < gmsk_bitlen) {		// Lade neue DAC-Daten
    if ((gmsk_bitcnt&0x1F) == 0) {		// bei jeden 32igsten Bit laden
      gmsk_tsr = swap32(*gmsk_dataptr++);
//        gmsk_tsr = GMSK_SYNCPATTERN;	// <<<<< TEST
    } // fi next word load
//      if (gmsk_tsr&0x80000000)		// Daten bitweise auswerten MSB first
    if (gmsk_tsr&0x00000001)			// Daten bitweise auswerten LSB first
      mod_new = dac_one_val;
    gmsk_tsr >>= 1;
    gmsk_bitcnt++;
  } // fi Bits zum aussenden
  if (gmsk_bitcnt >= gmsk_bitlen) {		// Ende Gelände?
    gmsk_bitcnt = 0;
    if (gmsk_nextbitlen > 0) { //&& (gmsk_nextdataptr != NULL)) {
      gmsk_dataptr = gmsk_nextdataptr;	// Lade nächstes Datapkt
      gmsk_bitlen  = gmsk_nextbitlen;
      gmsk_alertpos = gmsk_next_alertpos;
    } else {
      gmsk_bitlen = 0;
      gmsk_bittimer = GMSK_POSTAMBLEBITS;
      gmsk_alertpos = 0;
      INTC_register_interrupt(gmsk_runidle_int, AVR32_SPI_IRQ, DV_MODWRK_INTPRIO);
      mod_new = DAC_MIDDLE;
    } // esle "doch kein nächstes Paket"
    gmsk_nextdataptr = NULL;			// Datenzeiger
    gmsk_nextbitlen  = 0;
  } // fi alle Bits raus
  gmsk_nextdacval(mod_new);	// Verschiebe Daten in modulator_in (Vektor für FIR).
  // Gauss-Filter:
  dsp16_filt_fir(modulator_out, modulator_in, MOD_IN_FSIZE, (dsp16_t *)GaussCoeffs, GaussCoeffsSize);
  // Fast-Reload
  if ((gmsk_reloadhandler != NULL) && (gmsk_bitcnt==gmsk_alertpos)) {
    gmsk_reloadhandler();
  } // fi fast reload handler
}


INTERRUPT_FUNC gmsk_begin_int(void) {		// Starts Transfer after a TX-Delay
  GMSK_TX_TIMER.sr;
  if (gmsk_bittimer==0) {
    INTC_register_interrupt(gmsk_modulator_int, GMSK_TX_TIMER_IRQ, DV_MODOUT_INTPRIO);
    INTC_register_interrupt(gmsk_calcnextbit_int, AVR32_SPI_IRQ, DV_MODWRK_INTPRIO);
    GMSK_TX_TIMER.rc = GMSK_MOD_DEFAULT;	// Periode
    gmsk_txpll_accu.u32 = 0;
  } else {
    gmsk_bittimer--;
  }
} // end int


static __inline void gmsk_starttxdelay(void) {
  gmsk_stop_txtimer();
  INTC_register_interrupt(gmsk_begin_int, GMSK_TX_TIMER_IRQ, DV_MODWRK_INTPRIO);
  GMSK_TX_TIMER.rc  = (((MASTERCLOCK+1)/2)*GMSK_TXDRESOLUTION+500)/1000;	// 1ms
  GMSK_TX_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
  gmsk_bittimer     = gmsk_txdelay/GMSK_TXDRESOLUTION;
  GMSK_TX_TIMER.ier = AVR32_TC_CPCS_MASK;
}


//! @}



/*! \name Modulator API Functions
 */
//! @{

#define GMSK_SET_TX_MUTEX()	//Disable_global_interrupt()
#define GMSK_CLR_TX_MUTEX()	//Enable_global_interrupt()


void gmsk_transmit(const unsigned int *data, unsigned int length_in_bits, unsigned int alertbitpos) {
  if ((GMSK_TX_TIMER.sr&AVR32_TC_CLKSTA_MASK) == 0)	{ // if clock not enabled
    gmsk_nextbitlen  = 0;
    gmsk_nextdataptr = NULL;
    gmsk_dataptr = (U32 *)data;
    gmsk_bitcnt  = 0;
    gmsk_bitlen  = length_in_bits;
    gmsk_alertpos = alertbitpos;
    gmsk_starttxdelay();			// Start Timer (enable clock)
  } else if (gmsk_nextbitlen == 0) {
    gmsk_nextdataptr = (U32 *)data;
    gmsk_nextbitlen  = length_in_bits;
    gmsk_next_alertpos = alertbitpos;
  }
}


void gmsk_modulator_stop(void) {
  gmsk_nextdataptr = NULL;
  gmsk_stop_txtimer();			// Stop Timer (Clock)
  GMSK_TX_TIMER.idr = 0xFF;
  gmsk_reloadhandler = NULL;
  gmsk_alertpos = 0;
}


void gmsk_set_mod_hub(short int level) {
  dac_one_val  = DAC_MIDDLE+level;
  dac_zero_val = DAC_MIDDLE-level;
}


#ifdef GMSK_BANDWIDTHRECSIZE
void gmsk_set_bandwidth(unsigned char BW) {
  static const S16 GMSK_BandwidthRec[GMSK_BANDWIDTHRECSIZE] = {
    GMSK_BW_625, GMSK_BW_1000, GMSK_BW_1250, GMSK_BW_2500
  };
  if (BW >= GMSK_BANDWIDTHRECSIZE) BW = 0;
  dac_one_val  = +GMSK_BandwidthRec[BW]+DAC_MIDDLE;
  dac_zero_val = -GMSK_BandwidthRec[BW]+DAC_MIDDLE;
}
#endif


__inline void gmsk_set_txdelay(unsigned int millisec) {
  gmsk_txdelay = millisec;
}


__inline unsigned int gmsk_get_txdelay(void) {
  return gmsk_txdelay;
}


__inline void gmsk_set_reloadfunc(tgmsk_reloadfunc newfunc) {
  GMSK_SET_TX_MUTEX();
  gmsk_reloadhandler = newfunc;
  GMSK_CLR_TX_MUTEX();
}



__inline unsigned int gmsk_bitsleft(void) {
  return gmsk_bitlen - gmsk_bitcnt;
}


__inline int gmsk_nextpacket(void) {
  return (gmsk_nextbitlen == 0);
}


void gmsk_adjusttxperiod(signed int correction) {
  gmsk_txmod_clk = GMSK_MOD_PHASE + __builtin_sats(correction, 0, 17);
}


//! @}




/*! \name GMSK Demodulator API Functions
 */
//! @{

#define GMSK_SET_RX_MUTEX()	//Disable_global_interrupt()
#define GMSK_CLR_RX_MUTEX()	//Enable_global_interrupt()


__inline void gmsk_stop_rxtimer(void) {
  GMSK_RX_TIMER.ccr = AVR32_TC_CLKDIS_MASK;	// Stop Timer
  GMSK_RX_TIMER.idr = 0xFF;
  GMSK_RX_TIMER.sr;				// Clear Pending INT-Requests
}


void gmsk_demodulator_start(void) {
  gmsk_stop_rxtimer();
  demod_bitphasecnt = 0;
  gmsk_demod_unlock();
#if (DVRX_TIMER_CH==IDLE_TIMER_CH)
  GMSK_RX_TIMER.cmr = TIMER_CMR_VALUE;
#endif
#if (DVRX_TIMER_CH==IDLE_TIMER_CH)||(DVRX_TIMER_CH==DVTX_TIMER_CH)
  INTC_register_interrupt(gmsk_samplestart_int, GMSK_RX_TIMER_IRQ, DV_DEMODIN_INTPRIO);
#endif
  GMSK_RX_TIMER.sr;
  GMSK_RX_TIMER.ier = AVR32_TC_CPCS_MASK;
  GMSK_RX_TIMER.rc  = GMSK_RCDEFAULT;	// SPS-Rate: Periode für ADC
  GMSK_RX_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}


void gmsk_demodulator_stop(void) {
  gmsk_stop_rxtimer();
  demod_bitphasecnt = 0;
  gmsk_demod_unlock();
}


void gmsk_demodulator_invert(int invert) {
  demod_neg_level = (invert)?DEMOD_BITSET_MASK:0;
  demod_pos_level = (invert)?0:DEMOD_BITSET_MASK;  // set inverting
}


void gmsk_demodulator_toggle_invert(void) {
  U32 tmp_demod_level = demod_neg_level;
  demod_neg_level = demod_pos_level;
  demod_pos_level = tmp_demod_level;  // set inverting
}


void gmsk_set_patternfunc(tpattern_func patternhandler) {
  GMSK_SET_RX_MUTEX();
  if (patternhandler != NULL) {
    demod_pattern_fkt = patternhandler;
  } else {
    demod_pattern_fkt = no_patternhandler;
  }
  GMSK_CLR_RX_MUTEX();
}


void gmsk_set_unlockedfunc(tgmsk_func unlock_handler) {
  demod_unlocked_fkt = unlock_handler;
}


void gmsk_set_receivefkt(tgmsk_func fkt_received) {
  GMSK_SET_RX_MUTEX();
  demod_received_fkt  = fkt_received;
  GMSK_CLR_RX_MUTEX();
}


__inline void gmsk_set_receivebuf(unsigned int *rxbuf, int bit_len) {
  GMSK_SET_RX_MUTEX();
  demod_rxptr  = (U32 *)rxbuf;
  demod_rxsize = bit_len;
  GMSK_CLR_RX_MUTEX();
}


__inline int gmsk_channel_idle(void) {
  return (demod_state==DEMOD_unlocked);
}

__inline unsigned short gmsk_get_rssi_avrge(void) {
  return rssi_avrge;
}

__inline unsigned short gmsk_get_rssi_current(void) {
  return rssi_current;
}

//! @}



/*! \name GMSK Init/Exit API Functions
 */
//! @{


void gmsk_init(void) {
  int cnt;
  gmsk_stop_rxtimer();
  gmsk_modulator_stop();

  adc_init(HFIN_CHANNEL);
  dac_init();

  // *** Demodulator Global Vars ***
  dsp16_vect_zeropad(demod_adcin, DEMOD_ADC_SIZE, DEMOD_ADC_SIZE);
  gmsk_demod_unlock();
  demod_pattern_fkt  = no_patternhandler;
  demod_unlocked_fkt = NULL;
  demod_received_fkt = NULL;

  // *** Modulator Global Vars ***
  gmsk_txmod_clk = GMSK_MOD_PHASE;
  gmsk_bitcnt = gmsk_bitlen = gmsk_overcnt = 0;
//  gmsk_nextdataptr = NULL;
  gmsk_nextbitlen  = 0;
//  gmsk_alertpos = 0;
  gmsk_next_alertpos = 0;
  for (cnt=0; cnt<MOD_IN_SIZE; cnt++)	// Init mit Zero
    modulator_in[cnt] = DAC_MIDDLE;

  // *** Timer ***
  GMSK_RX_TIMER.cmr = TIMER_CMR_VALUE;
  GMSK_RX_TIMER.rc  = GMSK_RCDEFAULT;	// SPS-Rate: Periode für ADC
  INTC_register_interrupt(gmsk_samplestart_int, GMSK_RX_TIMER_IRQ, DV_DEMODIN_INTPRIO);
  INTC_register_interrupt(gmsk_processbit_int, GMSK_GETSAMPLE_IRQ, DV_DEMODWRK_INTPRIO);

#if (DVRX_TIMER_CH != DVTX_TIMER_CH)
  GMSK_TX_TIMER.cmr = TIMER_CMR_VALUE;
  GMSK_TX_TIMER.rc  = (((MASTERCLOCK+1)/2)*GMSK_TXDRESOLUTION+500)/1000;	// 1ms
  INTC_register_interrupt(gmsk_begin_int, GMSK_TX_TIMER_IRQ, DV_MODOUT_INTPRIO);
#endif
  dac_waitidle();
  dac_modulate(DAC_MIDDLE);
}


void gmsk_exit(void) {
  gmsk_stop_rxtimer();
  gmsk_modulator_stop();
  adc_exit();
  dac_exit();
}



#if (DVRX_TIMER_CH == DVTX_TIMER_CH)
__inline void gmsk_stoptimer(void) {
  gmsk_stop_txtimer();
}
#endif


//! @}

