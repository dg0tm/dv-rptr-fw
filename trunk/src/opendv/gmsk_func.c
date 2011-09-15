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
 */


#include "gmsk_func.h"

#include "hw_defs.h"

#include "gpio_func.h"		// Set/Clr Debug
#include "adc_func.h"		// Receive via internal ADC
#include "dac_func.h"		// Transmit via external DAC

#include <intc.h>
#include <dsp.h>


#define GMSK_RX_TIMER		AVR32_TC.channel[DVRX_TIMER_CH]
#define GMSK_RX_TIMER_IRQ	(AVR32_TC_IRQ0+DVRX_TIMER_CH)

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


#define GMSK_RCVARIANZ		0x14000L

#define SIGNAL_MIN_AMPLITUDE	60
#define	SIGNAL_MAX_AMPLITUDE	450

#define GMSK_SYNCPATTERN	0x55555555
#define GMSK_SYNCSTART		0x0A6EAAAA
#define GMSK_SYNCSTOP		0xF590AAAA	// inverted START Pattern
#define GMSK_FRAMESYNC		0x162D5500	// all >> direction
#define GMSK_FRAMESYNCMSK	0xFFFFFF00	// 24 bits



// *** Modulator Variables ***

Union32 gmsk_txpll_accu;


// *** Transmitter / Receiver Gauss-FIR Filter and Sample-Buffer ***
#define GaussCoeffsSize		9
#define MOD_IN_SIZE		(GaussCoeffsSize+GMSK_OVERSAMPLING-1)
#define GMSK_POSTAMBLEBITS	MOD_IN_SIZE

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


// *** Transmitter Buffer and Variables ****

A_ALIGNED dsp16_t modulator_in[MOD_IN_SIZE];		// Eingangsbuffer für FIR Filter
A_ALIGNED dsp16_t modulator_out[GMSK_OVERSAMPLING];	// geGaußte DAC Daten

U32 gmsk_txdelay = GMSK_STDTXDELAY;
U32 gmsk_overcnt, gmsk_tsr, gmsk_bitcnt, gmsk_bittimer;	// Zählervariablen Modulator
U32 gmsk_bitlen, gmsk_nextbitlen, gmsk_alertpos, gmsk_next_alertpos;
U32 *gmsk_dataptr, *gmsk_nextdataptr;		// Zeiger auf aktuelles Datum

dsp16_t dac_one_val  = DAC_MIDDLE + GMSK_DEFAULT_BW;
dsp16_t dac_zero_val = DAC_MIDDLE - GMSK_DEFAULT_BW;

tgmsk_reloadfunc gmsk_reloadhandler;		// Functionvar



/*! \name GMSK Modulator Interrupt-Functions
 */
//! @{

static __inline void gmsk_nextdacval(volatile dsp16_t newval) {
  dsp16_vect_copy(modulator_in, modulator_in+4, MOD_IN_SIZE-4);
  modulator_in[MOD_IN_SIZE-4] = newval;
  modulator_in[MOD_IN_SIZE-3] = newval;
  modulator_in[MOD_IN_SIZE-2] = newval;
  modulator_in[MOD_IN_SIZE-1] = newval;
}


static __inline void gmsk_stop_txtimer(void) {
  GMSK_TX_TIMER.ccr = AVR32_TC_CLKDIS_MASK;	// Stop Timer
  GMSK_TX_TIMER.idr = 0xFF;
  GMSK_TX_TIMER.sr;				// Clear Pending INT-Requests
}


INTERRUPT_FUNC gmsk_begin_int(void);	// forward


INTERRUPT_FUNC gmsk_runidle_int(void) {
  GMSK_TX_TIMER.sr;
  dac_modulate(modulator_out[gmsk_overcnt]);
  if (++gmsk_overcnt>=GMSK_OVERSAMPLING) {
    gmsk_overcnt = 0;
    if (gmsk_bittimer>0) {
      gmsk_bittimer--;
      // Verschiebe Daten in modulator_in (Vektor für FIR).
      gmsk_nextdacval(DAC_MIDDLE);
      // Gauss-Filter:
      dsp16_filt_fir(modulator_out, modulator_in, MOD_IN_SIZE, (dsp16_t *)GaussCoeffs, GaussCoeffsSize);
    } else {
      if (gmsk_nextbitlen==0) {			// Kein neues Paket
        gmsk_stop_txtimer();
      } else {
	gmsk_dataptr = gmsk_nextdataptr;	// Lade nächstes Datapkt
	gmsk_bitlen  = gmsk_nextbitlen;
      }
      GMSK_TX_TIMER.rc = (((MASTERCLOCK+1)/2)*GMSK_TXDRESOLUTION+500)/1000;	// Warte 1ms nach Idle
      INTC_register_interrupt(&gmsk_begin_int, GMSK_TX_TIMER_IRQ, DV_MOD_INTPRIO);
      gmsk_nextdataptr = NULL;			// Datenzeiger
      gmsk_nextbitlen  = 0;
    }
  } // fi oversampling
}



INTERRUPT_FUNC gmsk_modulator_int(void) {
  GMSK_TX_TIMER.sr;
  dac_modulate(modulator_out[gmsk_overcnt]);

  // Setting Timer-Period - modulate to a exact Baudrate
  GMSK_TX_TIMER.rc = GMSK_MOD_DEFAULT + gmsk_txpll_accu.u16[0];
  gmsk_txpll_accu.u16[0] = 0;	// Clear Alternate-Bit
  gmsk_txpll_accu.u32 += GMSK_MOD_PHASE;

  if (++gmsk_overcnt >= GMSK_OVERSAMPLING) {
    dsp16_t mod_new = dac_zero_val;
    gmsk_overcnt = 0;
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
	INTC_register_interrupt(&gmsk_runidle_int, GMSK_TX_TIMER_IRQ, DV_MOD_INTPRIO);
	mod_new = DAC_MIDDLE;
      } // esle "doch kein nächstes Paket"
      gmsk_nextdataptr = NULL;			// Datenzeiger
      gmsk_nextbitlen  = 0;
    } // fi alle Bits raus
    gmsk_nextdacval(mod_new);	// Verschiebe Daten in modulator_in (Vektor für FIR).
    // Gauss-Filter:
    dsp16_filt_fir(modulator_out, modulator_in, MOD_IN_SIZE, (dsp16_t *)GaussCoeffs, GaussCoeffsSize);
    // Fast-Reload
    if ((gmsk_reloadhandler != NULL) && (gmsk_bitcnt==gmsk_alertpos)) {
      gmsk_reloadhandler();
    } // fi fast reload handler
  } // fi oversampling
}


INTERRUPT_FUNC gmsk_begin_int(void) {		// Starts Transfer after a TX-Delay
  GMSK_TX_TIMER.sr;
  if (gmsk_bittimer==0) {
    INTC_register_interrupt(&gmsk_modulator_int, GMSK_TX_TIMER_IRQ, DV_MOD_INTPRIO);
    GMSK_TX_TIMER.rc = GMSK_MOD_DEFAULT;	// Periode
    gmsk_txpll_accu.u32 = 0;
  } else {
    gmsk_bittimer--;
  }
} // end int


static __inline void gmsk_starttxdelay(void) {
  gmsk_stop_txtimer();
  INTC_register_interrupt(&gmsk_begin_int, GMSK_TX_TIMER_IRQ, DV_MOD_INTPRIO);
  GMSK_TX_TIMER.rc  = (((MASTERCLOCK+1)/2)*GMSK_TXDRESOLUTION+500)/1000;	// 1ms
  GMSK_TX_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
  gmsk_bittimer     = gmsk_txdelay/GMSK_TXDRESOLUTION;
  GMSK_TX_TIMER.ier = AVR32_TC_CPCS_MASK;
}


//! @}



// *** DEMODULATOR ***

/*! \name GMSK Demodulator Interrupt-Functions
 */
//! @{

Union32 gmsk_rxpll_clk;
Union32 gmsk_rxpll_accu;


#define Average16(var_x, new_x, FAK)	{ var_x = dsp16_op_mul(var_x, DSP16_Q(1.0-FAK)) + dsp16_op_mul(new_x, DSP16_Q(FAK)); }
#define Average32(var_x, new_x, FAK)	{ var_x = dsp32_op_mul(var_x, DSP32_Q(1.0-FAK)) + dsp32_op_mul(new_x, DSP32_Q(FAK)); }

typedef void (*tgmsk_clockfkt)(S32, int);

typedef enum {
  DEMOD_unlocked, DEMOD_faded, DEMOD_sync, DEMOD_locked
} tDemodState;


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


#define RXFILTCOEFFSIZE		(sizeof(RXLowPassCoeff)/sizeof(dsp16_t))
#define DEMOD_ADC_SIZE		(RXFILTCOEFFSIZE+GMSK_BITSAMPLING-1)
#define DEMOD_FILT_SIZE		(2*GMSK_BITSAMPLING)

A_ALIGNED dsp16_t demod_adcin[DEMOD_ADC_SIZE];	// ADC-Buffer für eine Bitzeit
A_ALIGNED dsp16_t demod_filt[DEMOD_FILT_SIZE];	// Gefilterte ADC Werte (FIR Lowpass) etc
A_ALIGNED dsp16_t demod_in[GMSK_BITSAMPLING];	// Bitwerte ohne DC für Entscheider

#ifdef DEMOD_DBG_BITBUFSIZE
A_ALIGNED dsp16_t demod_backbuf[DEMOD_DBG_BITBUFSIZE*4];
#endif

// Demodulator-Handler
tgmsk_clockfkt demod_clockrecover;	// Funktion zum Nachführen der RX-Clock PLL
tgmsk_func demod_syncstart_fkt, demod_syncstop_fkt;
tgmsk_func demod_received_fkt, demod_framesync_fkt;

#define DEMOD_BITSET_MASK	0x80000000L

tDemodState	demod_state;

int	demod_bitphasecnt;
U32	demod_shr;		// Demodulations-Bit-Shift-Register
U32	demod_pos_level = 0x00000000;		// "null"
U32	demod_neg_level = DEMOD_BITSET_MASK;	// "eins"

U32	demod_rxbitcnt;		// Receive Bit Counter
U32	demod_rxsize;		// Bit-Size of Receivebuffer, rxptr pointed to
U32	*demod_rxptr;		// Receive Buffer Pointer


// Demodulator Entscheiderhilfen
int	demod_korrcnt, demod_fade_counter;

dsp16_t dc_level, high_level, low_level, meas_level;

dsp32_t demod_korr, demod_lastkorr;

dsp16_t dcd_clockval, dcd_levelval, dcd_value;


// Demodulator Int Einfach
// ToDo:
// Average-Werte optimieren
// Max-Werte DCD einführen

#define PLL_SYNC_KORR_FAK	0.1
#define PLL_LOCKED_KORR_FAK	0.095

#define DC_Level_Fak		0.01	//0.05
#define Meas_Level_Fak		0.02	//0.08, 0.2

#define DC_InvalBitsMask	0x7FFF8000	// MSB is zero ever (shifted previous)

#define DCD_CLOCK_FLT		0.02
#define DCD_LEVEL_FLT		0.01

#define DCD_CLOCK_BAL		1.0		// Gewicht Nulldurchgangsabweichung
#define DCD_LEVEL_BAL		0.3		// Gewicht Bitmax-Varianz

#define DCD_UNLOCK_VAL		1950
#define DCD_FADE_VAL		780
#define DCD_CAN_LOCK_VAL	50

#define DEMOD_MAX_FADE_BITTIME	(96*50*2)	// 2 Sekunden

#define gmsk_calc_zerocorr(zero0, zero1)	((S32)(zero1)*GMSK_RCDEFAULT / (zero0-zero1))

#define gmsk_reset_level()	{high_level=dc_level; low_level=dc_level;}


__inline void gmsk_stop_rxtimer(void) {
  GMSK_RX_TIMER.ccr = AVR32_TC_CLKDIS_MASK;	// Stop Timer
  GMSK_RX_TIMER.idr = 0xFF;
  GMSK_RX_TIMER.sr;				// Clear Pending INT-Requests
}


// PLL-Korrektur-Funktion (3 Versionen):
// Bekommt Wert, der der Abweichung in Timer-Schritten entspricht
// Funktion wird bei jeder Flanke aufgerufen und berechnet einen Wert für die DCD.
static void gmsk_unlocked_fkt(S32 correction, int bits_since_lastcorr) {
  // Schnell hin zum NDG
  GMSK_RX_TIMER.rc += __builtin_sats(correction, 3, 10);//3-8
}

static void gmsk_sync_fkt(S32 correction, int bits_since_lastcorr) {
  if (abs(correction) < (GMSK_RCDEFAULT/2)) {	// Wir sind in der richtigen Phase
    Average32(demod_korr,  correction/bits_since_lastcorr, PLL_SYNC_KORR_FAK);
//    gmsk_pllclk.u32 += demod_korr;
  }
  GMSK_RX_TIMER.rc += __builtin_sats(correction, 4, 7);//4-7
}

static void gmsk_locked_fkt(S32 correction, int bits_since_lastcorr) {
  if (abs(correction) < (GMSK_RCDEFAULT/2)) {	// Wir sind in der richtigen Phase
    Average32(demod_korr, correction/bits_since_lastcorr, PLL_LOCKED_KORR_FAK);
    // Berechnung ist falsch - besser:
    // Veränderung gegenüber Lastcorr
    // correction-last_corr / bitlen
    // das ganze dann Average...
//    gmsk_pllclk.u32 += demod_korr;
  }
  GMSK_RX_TIMER.rc += __builtin_sats(correction, 6, 6);
}

static void gmsk_faded_fkt(S32 correction, int bits_since_lastcorr) {
}


static void gmsk_demod_unlock(void) {
  demod_clockrecover = &gmsk_unlocked_fkt;	// Standard-Recover-Fkt
  demod_state = DEMOD_unlocked;
  gmsk_rxpll_clk.u32 = GMSK_PLLDEFAULT;		// Set PLL-Clock to default
  gmsk_rxpll_accu.u32 = 0;
  gmsk_reset_level();
  demod_korr = 0;
}


static __inline void gmsk_demod_sync(void) {
  demod_clockrecover = &gmsk_sync_fkt;
  demod_state = DEMOD_sync;
  ClrDebug1();
}

static __inline void gmsk_demod_lock(void) {
  demod_clockrecover = &gmsk_locked_fkt;
  demod_state = DEMOD_locked;
}

static __inline void gmsk_demod_fadeout(void) {
  demod_clockrecover = &gmsk_faded_fkt;
  demod_state = DEMOD_faded;
  demod_fade_counter = DEMOD_MAX_FADE_BITTIME;
  demod_korr = 0;
  SetDebug1();
}


static __inline void gmsk_calc_dcd(void) {
  dcd_value = dsp16_op_mul(dcd_clockval, DSP16_Q(DCD_CLOCK_BAL)) + \
              dsp16_op_mul(dcd_levelval, DSP16_Q(DCD_LEVEL_BAL));
  // dcd_level ist noch Amplitudenabhängig -> Verhältniss zw. Level und Ampl.
  switch (demod_state) {
  case DEMOD_unlocked:		// Tue nix: SYNC-Pattern oder FRAME-Sync nötig.
    break;
  case DEMOD_faded:
    // Warte auf ein Frame-Sync oder Sync-Pattern um nach SYNC zu wechseln
    if (demod_fade_counter > 0)
      demod_fade_counter--;
    else
      gmsk_demod_unlock();
    break;
  case DEMOD_sync:
    if (dcd_value < DCD_CAN_LOCK_VAL)
      gmsk_demod_lock();
    else if (dcd_value > DCD_UNLOCK_VAL)
      gmsk_demod_unlock();
    break;
  case DEMOD_locked:
    if (dcd_value > DCD_FADE_VAL)
      gmsk_demod_sync(); //fadeout();
    break;
  } // hctiws
}


static void dcd_init(void) {
  dcd_clockval = 0x7fff;
  dcd_levelval = 0x7fff;
}

/* ToDo Demodulator Int:
 * dcd_level relativ zu Gesamtlevel
 * ISI im Nulldurchgang berücksichtigen für Korrekturwerte
 * Letztes Bit ändern, wenn plausibel
 * Betrachtung nächstes Bit (ein Bit hinterher decodieren)
 * PLL-Shift begrenzen z.B. 1000ppm
 * Soft-Decision-Werte ermitteln (für AMBE Soft-Dec Modus)
 */


INTERRUPT_FUNC gmsk_demodulator_int(void) {
  dsp16_t adc_val;
  GMSK_RX_TIMER.sr;				// Acknowledge IRQ
  adc_val = HFIN << 4;				// Store last ADC (als 14 bit Wert)
  adc_startconversion();			// Start next ADC

  // Update Period-Length (PLL-Value for exact RX-Clock):
  GMSK_RX_TIMER.rc = gmsk_rxpll_clk.u16[0] + gmsk_rxpll_accu.u16[0];
  gmsk_rxpll_accu.u16[0] = 0;	// Clear Alternate-Bit
  gmsk_rxpll_accu.u32 += gmsk_rxpll_clk.u16[1];

#ifdef DEBUG_CLOCK_RECOVER
  dac_modulate(demod_in[demod_bitphasecnt]);
#endif
  // DC-Wert berechnen
  Average16(dc_level, adc_val, DC_Level_Fak);
  //dc_level = dsp16_op_mul(dc_level, DSP16_Q(1.0-DC_Level_Fak)) + dsp16_op_mul(adc_val, DSP16_Q(DC_Level_Fak));
  // Eingabebuffer für eine Bit-Länge füllen
  demod_adcin[DEMOD_ADC_SIZE-GMSK_BITSAMPLING+demod_bitphasecnt] = adc_val;

  if (++demod_bitphasecnt >= GMSK_BITSAMPLING ) {
    int cnt, flanke;
    dsp16_t bitval;
    demod_bitphasecnt = 0;
    SetDebug0();
    // Filter FIR -> Signal
    dsp16_filt_fir(demod_filt+GMSK_BITSAMPLING, demod_adcin, DEMOD_ADC_SIZE, (dsp16_t *)RXLowPassCoeff, RXFILTCOEFFSIZE);
    //dsp16_vect_copy(demod_filt+GMSK_BITSAMPLING, demod_adcin+DEMOD_ADC_SIZE-GMSK_BITSAMPLING, GMSK_BITSAMPLING);
    // Verschiebe alte Daten:
    dsp16_vect_copy(demod_adcin, demod_adcin+GMSK_BITSAMPLING, DEMOD_ADC_SIZE-GMSK_BITSAMPLING);

    // DC-Offset vom demod_adcin Vektor abziehen:
    for (cnt = 0; cnt<GMSK_BITSAMPLING; cnt++) {
      demod_in[cnt] = demod_filt[GMSK_BITSAMPLING+cnt]-meas_level;
    } // for Zero

    // Flanke erkennen:
    flanke = -1;
    for (cnt = 0; cnt < (GMSK_BITSAMPLING-1); cnt++) {
      if ((demod_in[cnt]&0x8000) != (demod_in[cnt+1]&0x8000)) {
	flanke = cnt;		// Position der Flanke
	continue;
      } // fi Vorzeichenvergleich
    }
    demod_korrcnt++;
    if (flanke > -1) {		// Nulldurchgang erkannt
      S32 clockcorr = (flanke*GMSK_RCDEFAULT) + gmsk_calc_zerocorr(demod_in[flanke], demod_in[flanke+1]);
      S32 corr_perbit = clockcorr/demod_korrcnt;		//16sats
      Average16(dcd_clockval, __builtin_sats(abs(corr_perbit), 0, 10), DCD_CLOCK_FLT);
      demod_clockrecover(clockcorr, demod_korrcnt);
      demod_lastkorr = clockcorr;
      demod_korrcnt  = 0;
    } // fi

    bitval = demod_filt[GMSK_BITSAMPLING+3];	// Bit-Wert nach Nulldurchg. mit DC
    // Bit in Shiftregister sichern (SHR)
    demod_shr >>= 1;
    if (demod_in[3] > 0) {
      Average16(dcd_levelval, abs(high_level-bitval), DCD_LEVEL_FLT);
      if (demod_state != DEMOD_faded) Average16(high_level, bitval, Meas_Level_Fak);
      if ((demod_shr&DC_InvalBitsMask)==DC_InvalBitsMask) {
	gmsk_reset_level();
      }
      demod_shr |= demod_pos_level;		// pos. Auslenkung -> i.d.R. "0" empfangen
    } else {	// low_level
      Average16(dcd_levelval, abs(low_level-bitval), DCD_LEVEL_FLT);
      if (demod_state != DEMOD_faded) Average16(low_level, bitval, Meas_Level_Fak);
      if ((demod_shr&DC_InvalBitsMask)==0) {
	gmsk_reset_level();
      }
      demod_shr |= demod_neg_level;		// neg. Auslenkung -> i.d.R. "1" empfangen
      //|= 0x80000000;
    }
    // berechne Gleichspannungsoffet fürs nächste Bit:
    meas_level = 	//(demod_state<DEMOD_sync)?dc_level:
      (low_level+((high_level-low_level)>>1));
    gmsk_calc_dcd();			// DCD durchführen

    // Speichern des SHR? Wenn Datenzeiger definert.
    if ((demod_state > DEMOD_unlocked) && (demod_rxptr!=NULL)) {
      demod_rxbitcnt++;
      if ((demod_rxbitcnt&0x1F)==0 ) {
        *demod_rxptr++ = swap32(demod_shr);
      } else if (demod_rxbitcnt==demod_rxsize) {	// fi Word full
	*demod_rxptr = swap32(demod_shr >> ((demod_rxsize|0x1F)-demod_rxsize+1));
      } // fi last Bits
      if (demod_rxbitcnt==demod_rxsize) {
	demod_rxptr = NULL;	// vor ReceiveFkt.
	if (demod_received_fkt != NULL) demod_received_fkt();
      } // fi fertig
    } // fi store shift-register

    if (demod_shr == GMSK_SYNCPATTERN) {
      gmsk_demod_sync();	// bedingungsloses Wechseln nach SYNC
      demod_rxbitcnt = 0;
    } else if (demod_shr == GMSK_SYNCSTART) {		// Header Receive Mode...
      demod_rxbitcnt = 0;
      if (demod_syncstart_fkt != NULL) demod_syncstart_fkt();
    } else if (demod_shr == GMSK_SYNCSTOP) {
      gmsk_demod_unlock();
      demod_rxbitcnt = 0;
      if (demod_syncstop_fkt != NULL) demod_syncstop_fkt();
    } else if ((demod_shr&GMSK_FRAMESYNCMSK) == GMSK_FRAMESYNC) {
      demod_rxbitcnt = 0;
      if (demod_state < DEMOD_sync) gmsk_demod_sync();
      if (demod_framesync_fkt != NULL) demod_framesync_fkt();
    } // fi
    // Altes Bit aufheben zum Vergleich
    dsp16_vect_copy(demod_filt, demod_filt+GMSK_BITSAMPLING, GMSK_BITSAMPLING);

#ifdef DEMOD_DBG_BITBUFSIZE
    // Für Test Back-Buffer:
    dsp16_vect_copy(demod_backbuf, demod_backbuf+GMSK_BITSAMPLING, DEMOD_DBG_BITBUFSIZE*4-GMSK_BITSAMPLING);
    dsp16_vect_copy(demod_backbuf+DEMOD_DBG_BITBUFSIZE*4-GMSK_BITSAMPLING, demod_in, GMSK_BITSAMPLING);
#endif
    ClrDebug0();
  } // fi für jedes Bit
}


//! @}


/*! \name Modulator API Functions
 */
//! @{


void gmsk_transmit(unsigned long *data, unsigned int length_in_bits, unsigned int alertbitpos) {
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


__inline void gmsk_set_txdelay(unsigned int millisec) {
  gmsk_txdelay = millisec;
}


__inline unsigned int gmsk_get_txdelay(void) {
  return gmsk_txdelay;
}


__inline void gmsk_set_reloadfunc(tgmsk_reloadfunc newfunc) {
  gmsk_reloadhandler = newfunc;
}



__inline unsigned int gmsk_bitsleft(void) {
  return gmsk_bitlen - gmsk_bitcnt;
}


__inline int gmsk_nextpacket(void) {
  return (gmsk_nextbitlen == 0);
}


//! @}




/*! \name GMSK Demodulator API Functions
 */
//! @{


void gmsk_demodulator_start(void) {
  gmsk_stop_rxtimer();
  demod_bitphasecnt = 0;
  gmsk_demod_unlock();
  INTC_register_interrupt(&gmsk_demodulator_int, GMSK_RX_TIMER_IRQ, DV_DEMOD_INTPRIO);
  GMSK_RX_TIMER.sr;
  GMSK_RX_TIMER.ier = AVR32_TC_CPCS_MASK;
  GMSK_RX_TIMER.rc  = GMSK_RCDEFAULT;	// SPS-Rate: Periode für ADC
  GMSK_RX_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}


void gmsk_demodulator_invert(int invert) {
  demod_neg_level = (invert)?DEMOD_BITSET_MASK:0;
  demod_pos_level = (invert)?0:DEMOD_BITSET_MASK;  // set inverting
}


void gmsk_set_synchandler(tgmsk_func syncstart, tgmsk_func syncstop, tgmsk_func framesync) {
  demod_syncstart_fkt = syncstart;
  demod_syncstop_fkt  = syncstop;
  demod_framesync_fkt = framesync;
}



void gmsk_set_receivefkt(tgmsk_func fkt_received) {
  demod_received_fkt  = fkt_received;
}


void gmsk_set_receivebuf(unsigned long *rxbuf, int bit_len) {
  demod_rxbitcnt = 0;
  demod_rxptr  = rxbuf;
  demod_rxsize = bit_len;
}


__inline int gmsk_channel_idle(void) {
  return (demod_state==DEMOD_unlocked);
}

//! @}



/*! \name GMSK Init/Exit API Functions
 */
//! @{

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


void gmsk_init(void) {
  int cnt;
  gmsk_stop_rxtimer();
  gmsk_modulator_stop();

  adc_init(HFIN_CHANNEL);
  dac_init();

  // *** Demodulator Global Vars ***
  dsp16_vect_zeropad(demod_adcin, DEMOD_ADC_SIZE, DEMOD_ADC_SIZE);
  gmsk_demod_unlock();
  demod_syncstart_fkt = NULL;
  demod_syncstop_fkt  = NULL;
  demod_received_fkt  = NULL;
  demod_framesync_fkt = NULL;
  dcd_init();

  // *** Modulator Global Vars ***
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
  INTC_register_interrupt(&gmsk_demodulator_int, GMSK_RX_TIMER_IRQ, DV_DEMOD_INTPRIO);
#if (DVRX_TIMER_CH != DVTX_TIMER_CH)
  GMSK_TX_TIMER.cmr = TIMER_CMR_VALUE;
  GMSK_TX_TIMER.rc  = (((MASTERCLOCK+1)/2)*GMSK_TXDRESOLUTION+500)/1000;	// 1ms
  INTC_register_interrupt(&gmsk_begin_int, GMSK_TX_TIMER_IRQ, DV_MOD_INTPRIO);
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

//! @}

