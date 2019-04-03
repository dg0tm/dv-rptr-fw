/*
 * ambe_pass.c
 *
 * Implements complex communication function include interrupt-handling and timer
 * based sleep / watchdog.
 *
 *  Created on: 17.01.2010
 *      Author: DO1FJN, Jan Alte
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
 * Report:
 * 2012-08-07	Rework AMBE-Timer Handling on Encode/Decode Calls
 * 2012-08-09	New Define for SSC-Clock, default 1.00MHz (prev 2.00MHz)
 * 2012-03-14	2.00MHz SSC
 *
 * ToDo:
 * -Check Power-Down vs. Hard-Reset Current
 * -Slip-Control
 *
 * Done:
 * +Bug: Kein Einschalten bei Decode nach langer Ruhe!
 * +PreDefined DTMF Tone Packets
 * +easy VAD on/off define
 * +ambe_powerdown()
 * +VoiceBuf System verbessern
 *
 */

#include "ambe_func.h"
#include "TLV320AIC.h"
#include "hw_defs.h"
#include "gpio_func.h"
#include "int_func.h"
#include "intc.h"
#include "compiler.h"
#include <string.h>


// AMBE passive framed mode:
// use a min. gap of 25 SSC-Clock-Cylces to prevent timing errors
#define SSC_xCMR_CLKBYMCU	0x13000404	// continous clock, Start falling RF, Period 40Clks
#define SSC_xFMR_CLKBYMCU	0x0020008F	// RF: Pos.Pulse, 16Bit Len, MSB First, 24 Data
#define SSC_xCMR_CKI_MASK	0x00000020

#define SSC_CLOCK_DIVIDER	15		// 2.00MHz SSC Clock (RK,TK Pins)


#define AMBE_TIMER		AVR32_TC.channel[AMBE_TIMER_CH]
#define AMBE_TIMER_IRQ		(AVR32_TC_IRQ0+AMBE_TIMER_CH)

#define ambe_set_reset()	tlv_set_gpio1()
#define ambe_clr_reset()	tlv_clr_gpio1()


#define AMBE_OPERATEID			0x0000
#define AMBE_PWRDOWNID			0x5500
#define AMBE_ID_MASK			0xFF00
#define AMBE_CTRL1_CNI_OFFSET		1
#define AMBE_CTRL1_LFI_OFFSET		7
#define AMBE_CTRL1_ID_OFFSET		8
#define AMBE_CTRL1_CNI_MASK		(1<<AMBE_CTRL1_CNI_OFFSET)
#define AMBE_CTRL1_LFI_MASK		(1<<AMBE_CTRL1_LFI_OFFSET)

#define AMBE_CTRL2_RIS_OFFSET		0
#define AMBE_CTRL2_EE_OFFSET		2
#define AMBE_CTRL2_SL_OFFSET		3
#define AMBE_CTRL2_VAD_OFFSET		5
#define AMBE_CTRL2_VOL_OFFSET		8

#define AMBE_CTRL2_RIS_BOTH		0x0
#define AMBE_CTRL2_RIS_ENCODER		0x1
#define AMBE_CTRL2_RIS_DECODER		0x2
#define AMBE_CTRL2_RIS_NONE		0x3
#define AMBE_CTRL2_RIS_MASK		0x0003

#define AMBE_CTRL2_EE_MASK		(1<<AMBE_CTRL2_EE_OFFSET)
#define AMBE_CTRL2_SL_MASK		(1<<AMBE_CTRL2_SL_OFFSET)
#define AMBE_CTRL2_VAD_MASK		(1<<AMBE_CTRL2_VAD_OFFSET)

#define AMBE_CTRL2_ENCODE		(AMBE_CTRL2_VAD_MASK|AMBE_CTRL2_EE_MASK)

#define AMBE_TIMER_CLOCK		((MASTERCLOCK+64)/128)

#define AMBE_TIMETOBOOT			134	// Zeit, der der AMBE max. zum Aufwachen benötigt
#define AMBE_FRAMETIMEOUT		24	// bei Überschreiten übernimmt Timer des TX
#define AMBE_TIMETORESET		2	// Zeit nach Powerdown, wenn AMBE erneut starten soll

// measures boot duration: 124,05ms (until first paket served)


// *** Globale Variablen und Buffer ***

tAMBEinframe	ambe_input;		// To-be-Transmit AMBE-Frame
tAMBEoutframe	ambe_output;		// Last Received AMBE-Frame

unsigned int	ambe_rxframe_count;
unsigned int	ambe_idle_count;
unsigned int	ambe_powerdown_to;
unsigned int	ambe_dtmf_count, ambe_2ndtone_count, ambe_tonepause_count;

unsigned char	ambe_2ndtone_val;

tambestate	AMBEstate = AMBEnoinit;		// Zustand AMBE-Chip
tambestate	prevAMBEstate = AMBEnoinit;	// Zustand bei letzter newstate Abfrage

unsigned short	BitErrorRate;

volatile bool	ambe_newencoded;

const char DtmfTone[16][AMBE_USEDDATA] = {
  {0xd3, 0xcb, 0x20, 0xd3, 0x03, 0x1d, 0x34, 0x41, 0x10},	// 1
  {0x83, 0xca, 0x24, 0xa2, 0x45, 0x11, 0x44, 0xc6, 0x2c},	// 4
  {0x83, 0x4b, 0x38, 0xc3, 0x04, 0x0d, 0x45, 0x8e, 0x10},	// 7
  {0xd3, 0xcb, 0x3c, 0xb2, 0x02, 0x05, 0x35, 0x0b, 0x2c},	// *
  {0xd3, 0xcb, 0x20, 0xd3, 0x03, 0x5d, 0x34, 0x41, 0x10},	// 2
  {0x83, 0xca, 0x24, 0xa2, 0x45, 0x51, 0x44, 0xc6, 0x2c},	// 5
  {0x83, 0x4b, 0x38, 0xc3, 0x04, 0x4d, 0x45, 0x8e, 0x10},	// 8
  {0xd3, 0xcb, 0x3c, 0xb2, 0x02, 0x45, 0x35, 0x0b, 0x2c},	// 0
  {0xd3, 0xcb, 0x20, 0xd3, 0x13, 0x1d, 0x34, 0x41, 0x10},	// 3
  {0x83, 0xca, 0x24, 0xa2, 0x55, 0x11, 0x44, 0xc6, 0x2c},	// 6
  {0x83, 0x4b, 0x38, 0xc3, 0x14, 0x0d, 0x45, 0x8e, 0x10},	// 9
  {0xd3, 0xcb, 0x3c, 0xb2, 0x12, 0x05, 0x35, 0x0b, 0x2c},	// #  
  {0xd3, 0xcb, 0x20, 0xd3, 0x13, 0x5d, 0x34, 0x41, 0x10},	// A
  {0x83, 0xca, 0x24, 0xa2, 0x55, 0x51, 0x44, 0xc6, 0x2c},	// B
  {0x83, 0x4b, 0x38, 0xc3, 0x14, 0x4d, 0x45, 0x8e, 0x10},	// C
  {0xd3, 0xcb, 0x3c, 0xb2, 0x12, 0x45, 0x35, 0x0b, 0x2c}	// D
};

const char SilenceFrame[AMBE_USEDDATA] = {
  0x8e, 0x4f, 0xb8, 0xb0, 0xd5, 0x5f, 0x2b, 0xa0, 0xe8
};


char 	VoiceBuf[AMBE_USEDDATA];	// zeitweiliger Sprachdaten-Buffer
char 	*VoicePktPtr;			// Zeiger auf aktuelle Daten

// *** Forward ***

static void ambe_start(void);
static void ambe_start_transmit(void);


// *** Timer-Routinen ***

static __inline void ambe_timer_start(void) {
  AMBE_TIMER.sr;
  AMBE_TIMER.ier = AVR32_TC_CPCS_MASK;
  AMBE_TIMER.ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}

static __inline void ambe_timer_stop(void) {  // Stop Watchdog-Timer
  AMBE_TIMER.idr = 0xFF;
  AMBE_TIMER.ccr = AVR32_TC_CLKDIS_MASK;
  AMBE_TIMER.sr;	// Clear Pending INT-Requests
}

static __inline void ambe_restart_timer(void) {
  AMBE_TIMER.ccr = AVR32_TC_SWTRG_MASK; // | AVR32_TC_CLKEN_MASK;
}

static __inline void ambe_timer_setms(unsigned int mscnt) {	// ms-max = 138!!!
  AMBE_TIMER.rc = (mscnt*AMBE_TIMER_CLOCK)/1000;
}

#define ambe_timer_resetwd()	ambe_timer_setms(AMBE_FRAMETIMEOUT)



// *** ISR für den Timer (Überwachung) ***

INTERRUPT_FUNC ambe_timer_TurnOff(void) {	// Ende Gelände: AMBE ausschalten
  ambe_stop();		// erledigt per timer_stop auch das INT-ACK (lesen SR)
}


static __inline void ambe_setboottimer(void) {
  ambe_timer_setms(AMBE_TIMETOBOOT);		// gib AMBE 135ms zum Booten (124.05ms nötig)
  INTC_register_interrupt(ambe_timer_TurnOff, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
  ambe_timer_start();
}


INTERRUPT_FUNC ambe_timer_TurnOn(void) {	// AMBE wieder einschalten
  ambe_timer_stop();				// TurnOff & acknowledge INT
  ambe_start();
}


INTERRUPT_FUNC ambe_timer_Sleep(void) {		// Abschalt-Timer für wartenden AMBE
  AMBE_TIMER.sr;				// Read Status to acknowledge INT
  ambe_idle_count++;
  if (ambe_input.Ctrl1&AMBE_PWRDOWNID) {	// Power-Down gesetzt
    tambestate beforestop = AMBEstate;
    ambe_stop();
    if (beforestop==AMBEbooting) {		// Sonderfall -> sofort wieder einschalten
      INTC_register_interrupt(ambe_timer_TurnOn, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
      ambe_timer_setms(AMBE_TIMETORESET);
      ambe_timer_start();
    } // fi restart
  } else if (ambe_idle_count > ambe_powerdown_to) {
    ambe_input.Ctrl1 = AMBE_PWRDOWNID;
    ambe_start_transmit();
    AMBEstate = AMBEpowerdown;
    eic_disableint(AMBE_EPR_INT);		// Keine weiteren AMBE-Pakete verarbeiten
  } // fi
}


INTERRUPT_FUNC ambe_timer_Decode(void) { // Decode-Timer Int (startet, wenn keine neuen Pakete eintreffen)
  AMBE_TIMER.sr;				// Read Status to acknowledge INT
  ambe_timer_setms(20);
  if (ambe_dtmf_count > 0) {			// DTMF statt Decoding
    ambe_input.Ctrl2 &= ~AMBE_CTRL2_SL_MASK;
    if (LSB(ambe_input.DtmfCtrl) == 0xFF) {	// Gap
      ambe_input.Ctrl2 |= AMBE_CTRL2_SL_MASK;	// Kein RX vom AMBE!
      ambe_dtmf_count--;
      if (ambe_dtmf_count == 0) {
	ambe_dtmf_count = ambe_2ndtone_count;
	LSB(ambe_input.DtmfCtrl) = ambe_2ndtone_val;
      } // fi Gap Zeit abgelaufen
    } // fi no tone
  } else if (ambe_input.Ctrl2&AMBE_CTRL2_SL_MASK) {
    INTC_register_interrupt(ambe_timer_Sleep, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
  }
  ambe_start_transmit();		// TimeOut, Decode LFI->CNI...
}




// *** ISR für SSC Synchronous Serial Controller Receive ***


INTERRUPT_FUNC ambe_rxfinish_int(void) {
  AVR32_SSC.cr  = AVR32_SSC_CR_RXDIS_MASK;	// Stop RX of SSC
  // Int bleibt aktiv solange tcr==0, daher schon mal init
  AVR32_PDCA.channel[AMBE_CHANNEL0].mar = (U32)&ambe_output.Ctrl1;
  AVR32_PDCA.channel[AMBE_CHANNEL0].tcr = AMBE_FRAMESIZE-1;
  eic_clrline(AMBE_EPR_INT);
  if (AMBEstate > AMBEpowerdown)		// No OFF-Req.
    eic_reenableint(AMBE_EPR_INT);		// Activate EPR-Int (sense next packet rdy)

  ambe_rxframe_count++;
  if ((ambe_input.Ctrl2&AMBE_CTRL2_RIS_MASK)==AMBE_CTRL2_RIS_NONE) {
    // (Rate ist gesetzt - Normalbetrieb)
    ambe_idle_count++;
    if (ambe_idle_count >= ambe_powerdown_to) ambe_standby();

    if (ambe_output.Ctrl2&AMBE_CTRL2_EE_MASK) {	// Arbeitet als Decoder oder DTMF-Geber?

      if (ambe_dtmf_count > 0) {		// DTMF statt Decoding
 	ambe_dtmf_count--;
        ambe_restart_timer();			// TimeOut-Timer zurücksetzen
	if ((ambe_input.DtmfCtrl&0xF0)==0x80) {	// one of the 16 defined tones
	  VoicePktPtr = (char *)DtmfTone[ambe_input.DtmfCtrl&0x0F];
	} else {		// Silence-Frame
          VoicePktPtr = (char *)SilenceFrame;
	}
	if (ambe_dtmf_count==0) {		// Ablauf der Ton/Pausen-Ausgabe
	  if (LSB(ambe_input.DtmfCtrl) == 0xFF) { // War Pause
            ambe_dtmf_count = ambe_2ndtone_count;
	    LSB(ambe_input.DtmfCtrl) = ambe_2ndtone_val;
	  } else {				// war erster Ton
            ambe_dtmf_count = ambe_tonepause_count; // lade "Pause"
            ambe_tonepause_count = 0;
	    LSB(ambe_input.DtmfCtrl) = 0xFF;
	  }
	  if (ambe_dtmf_count==0) switch (AMBEstate) {
	    case AMBEencoding:
	      LSB(ambe_input.Ctrl2) = AMBE_CTRL2_ENCODE;
	      break;
	    case AMBEgo2sleep:
	      AMBEstate = AMBEsleep;	// no break
	    case AMBEsleep:
	      ambe_input.Ctrl2 |= AMBE_CTRL2_SL_MASK;
	      break;
	    default:
	      break;
	  } // hctiws
	} // fi Ende
	ambe_start_transmit();
      } else if ( (ambe_input.Ctrl1&AMBE_CTRL1_CNI_MASK) == 0 ) {	// normal Decoding
	BitErrorRate = ambe_output.BER;
      } // fi fresh Voice

    } else {	// *** ist Encoder *** (Out-EE-Bit ist 1 bei DECODER!!!)
      ambe_restart_timer();			// TimeOut-Timer zurücksetzen
      VoicePktPtr = ambe_output.Data;
      ambe_newencoded = true;
      if (ambe_input.Ctrl2&AMBE_CTRL2_SL_MASK) { // Standby erwünscht
	INTC_register_interrupt(ambe_timer_Sleep, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
	ambe_timer_setms(20);
        ambe_start_transmit();			// zum AMBE
      } // fi stby
    } // esle

  } else {	// RateInfo!=NONE
    ambe_timer_stop();				// Verhindere Timer-Trigger durch RC-Ä
    // Vergleiche Rate von zuletzt gesendeten Paket:
    if ( (!memcmp(ambe_input.Rate, ambe_output.Rate, sizeof(ambe_output.Rate))) && \
      ((ambe_input.Ctrl2^ambe_output.Ctrl2)&AMBE_CTRL2_EE_MASK) ) {
      ambe_input.Ctrl2 |= AMBE_CTRL2_RIS_NONE;
      ambe_input.Ctrl2 &= ~AMBE_CTRL2_SL_MASK;
      ambe_input.Ctrl1 |= AMBE_CTRL1_CNI_MASK;
      if (AMBEstate > AMBEpowerdown) {		// No OFF-Req.
        AMBEstate = (ambe_output.Ctrl2&AMBE_CTRL2_EE_MASK)?AMBEdecoding:AMBEencoding;
        if (AMBEstate == AMBEencoding) {	// power-up ADC
          tlv_unmute_adc();
        } else {
          tlv_unmute_dac();
        }
      }
    } // fi Rate/Mode gleich
    if (ambe_input.Ctrl2&AMBE_CTRL2_EE_MASK) {	// Encoding-Mode?
      ambe_start_transmit();			// dann Sende gleich zum AMBE
      INTC_register_interrupt(ambe_timer_TurnOff, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
    } else {
      INTC_register_interrupt(ambe_timer_Decode, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
    }
    ambe_timer_setms(AMBE_FRAMETIMEOUT);	// TimeOut, wenn kein Pkt nach 24ms da ist
    ambe_timer_start();
  } // esle Rate/Mode not set

}



INTERRUPT_FUNC ambe_ssc_int(void) {
  U32 Status = AVR32_SSC.sr;// & AVR32_SSC.imr;	// Read+Reset Status Register
  if (Status&AVR32_SSC_SR_RXRDY_MASK) {		// Received a word?
    ambe_output.Header = AVR32_SSC.rhr;
    if (ambe_output.Header == AMBE_HEADER) {	// wenn Header empfangen...
      AVR32_SSC.idr = AVR32_SSC_RXRDY_MASK;	// ...SSC-RxRdy-Int de- ...
      AVR32_PDCA.channel[AMBE_CHANNEL0].cr = AVR32_PDCA_TEN_MASK; // ...und PDAC aktivieren.
      if (VoicePktPtr==ambe_output.Data) { 	// Sprachdaten-Zeiger auf Buffer
 	VoicePktPtr = VoiceBuf;			// setzen, wenn auf Data zeigt.
      } // fi
      memcpy(VoiceBuf, ambe_output.Data, AMBE_USEDDATA);
    } // fi Frame-Header
  } // fi RXRDY
  if (Status&AVR32_SSC_SR_TXEMPTY_MASK) { 	// Finish Transmitting?
    AVR32_SSC.idr = AVR32_SSC_TXEMPTY_MASK;	// Turn Off IRQ Event
    AVR32_SSC.cr  = AVR32_SSC_CR_TXDIS_MASK;	// Turn Off Transmitter
  } // fi txempty
}


// *** ISR für Packet-Ready Int ***

INTERRUPT_FUNC ambe_epr_int(void) {
  eic_disableint(AMBE_EPR_INT);
  AVR32_PDCA.channel[AMBE_CHANNEL0].cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  // PDAC vorbereiten: 23 weitere Words einlesen...
  AVR32_PDCA.channel[AMBE_CHANNEL0].mar = (U32)&ambe_output.Ctrl1;
  AVR32_PDCA.channel[AMBE_CHANNEL0].tcr = AMBE_FRAMESIZE-1;
  // einzelne Words empfangen, auf AMBE-Header warten:
  AVR32_SSC.ier = AVR32_SSC_RXRDY_MASK;
  AVR32_SSC.cr  = AVR32_SSC_CR_RXEN_MASK;	// start RX-Op of SSC
}



INTERRUPT_FUNC ambe_txfinish_int(void) {
  AVR32_PDCA.channel[AMBE_CHANNEL1].idr = AVR32_PDCA_IER_TRC_MASK;
  AVR32_SSC.ier = AVR32_SSC_TXEMPTY_MASK;
  if ((ambe_input.Ctrl2&AMBE_CTRL2_EE_MASK)==0) {	// Decoding-Mode?
    switch (ambe_input.Ctrl1&(AMBE_CTRL1_LFI_MASK|AMBE_CTRL1_CNI_MASK)) {
      case 0:	// No LFI & CNI (actual, valid voice data)?
	LSB(ambe_input.Ctrl1) = AMBE_CTRL1_LFI_MASK; // LFI on
	break;
      case AMBE_CTRL1_LFI_MASK:	// LFI off, CNI on
	LSB(ambe_input.Ctrl1) = AMBE_CTRL1_CNI_MASK; // CNI on
	break;
      } // hctiws
  } // fi encoding
}



static void ambe_start_transmit(void) {
  if ((AVR32_SSC.sr&AVR32_SSC_SR_TXEN_MASK)==0) {
    AVR32_PDCA.channel[AMBE_CHANNEL1].cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
    AVR32_PDCA.channel[AMBE_CHANNEL1].mar = (U32)&ambe_input;
    AVR32_PDCA.channel[AMBE_CHANNEL1].tcr = AMBE_FRAMESIZE;
    AVR32_PDCA.channel[AMBE_CHANNEL1].ier = AVR32_PDCA_IER_TRC_MASK;
    AVR32_SSC.cr  = AVR32_SSC_CR_TXEN_MASK;	// SSC: Transmit Enable
    AVR32_SSC.sr;
    AVR32_PDCA.channel[AMBE_CHANNEL1].cr  = AVR32_PDCA_TEN_MASK; // PDAC-TX aktiv
  }
}


static void ambe_start(void) {
  tambestate startstate = AMBEstate;
  if ((startstate==AMBEnoinit)||(startstate > AMBEbooting))
    return;
  ambe_timer_stop();
  AMBEstate = AMBEbooting;
  if (startstate==AMBEoff) {
    ambe_idle_count    = 0;
    ambe_rxframe_count = 0;
    ambe_setboottimer();
    ambe_set_reset();				// Starts AMBE2020 from Reset
    eic_clrline(AMBE_EPR_INT);
    eic_reenableint(AMBE_EPR_INT);		// Activate EPR-Int (sense next packet rdy)
    AVR32_SSC.sr;
    AVR32_PDCA.channel[AMBE_CHANNEL0].ier = AVR32_PDCA_IER_TRC_MASK;
  } // fi
}



/*! \name AMBE API-Functions
 */
//! @{


void ambe_stop(void) {
  eic_disableint(AMBE_EPR_INT);
  AMBEstate = AMBEpowerdown;
  tlv_mute_both();
  AVR32_SSC.idr = 0xFFFF;		// SSC Ints off
  AVR32_PDCA.channel[AMBE_CHANNEL0].idr = AVR32_PDCA_IDR_TERR_MASK|AVR32_PDCA_IDR_TRC_MASK|AVR32_PDCA_IDR_RCZ_MASK;
  AVR32_PDCA.channel[AMBE_CHANNEL1].idr = AVR32_PDCA_IDR_TERR_MASK|AVR32_PDCA_IDR_TRC_MASK|AVR32_PDCA_IDR_RCZ_MASK;
  AVR32_PDCA.channel[AMBE_CHANNEL0].cr  = AVR32_PDCA_TDIS_MASK;
  AVR32_PDCA.channel[AMBE_CHANNEL1].cr  = AVR32_PDCA_TDIS_MASK;
  AVR32_SSC.cr = 0x0202;		// Stop Both
  ambe_timer_stop();
  ambe_clr_reset();
  ambe_rxframe_count = 0;
  ambe_dtmf_count    = 0;
  ambe_idle_count    = 0;
  AMBEstate = AMBEoff;
}


void ambe_standby(void) {
  if (AMBEstate > AMBEpowerdown) {
    if (ambe_dtmf_count < 2) {
      ambe_input.Ctrl2 |= AMBE_CTRL2_SL_MASK;	// enable Frame-Based Sleep Mode
      AMBEstate = AMBEsleep;
    } else {
      AMBEstate = AMBEgo2sleep;
    }
    ambe_idle_count = 0;
  } // fi acticw
}


void ambe_encode(void) {
  if ((AMBEstate != AMBEencoding) && (AMBEstate != AMBEnoinit)) {
    ambe_idle_count = 0;
    //ambe_rxframe_count = 0;
    ambe_input.Ctrl1         = AMBE_OPERATEID;
    LSB(ambe_input.Ctrl2)    = AMBE_CTRL2_ENCODE;
    LSB(ambe_input.DtmfCtrl) = 0xFF;
    VoicePktPtr = (char *)SilenceFrame;
    if ((AMBEstate == AMBEoff) || (AMBEstate == AMBEpowerdown))
      ambe_start();
    else if (AMBEstate > AMBEbooting) {
      ambe_start_transmit();			// AMBE aufwecken
    }
  } // fi
}


void ambe_decode(void) {
  if ((AMBEstate != AMBEdecoding) && (AMBEstate != AMBEnoinit)) {
    ambe_idle_count = 0;
    //ambe_rxframe_count = 0;
    ambe_input.Ctrl1 = AMBE_OPERATEID|AMBE_CTRL1_CNI_MASK;	// CNI on
    LSB(ambe_input.Ctrl2)    = 0x00;		// Decoding, Set both rates
    LSB(ambe_input.DtmfCtrl) = 0xFF;
    if ((AMBEstate == AMBEoff) || (AMBEstate == AMBEpowerdown))
      ambe_start();
  } // fi allowed states
}


void ambe_powerdown(void) {	// Kontrolliertes "runterfahren" bei Op-Mode-Wechsel
  if (AMBEstate > AMBEpowerdown) {
    AMBEstate = AMBEpowerdown;
    ambe_timer_stop();
    ambe_input.Ctrl1      = AMBE_PWRDOWNID;
    LSB(ambe_input.Ctrl2) = AMBE_CTRL2_EE_MASK;	// Encoder-Enable
    ambe_start_transmit();
    INTC_register_interrupt(ambe_timer_TurnOff, AMBE_TIMER_IRQ, AMBE_TIMER_PRIO);
    ambe_timer_setms(AMBE_FRAMETIMEOUT);
    ambe_timer_start();
    eic_disableint(AMBE_EPR_INT);
  } // fi
}


void ambe_getvoice(unsigned char *dest) {
  memcpy(dest, VoicePktPtr, AMBE_USEDDATA);	// Encoded or Const Data copy
  ambe_idle_count = 0;
  ambe_newencoded = false;
}


__inline int ambe_have_new_encoded(void) {
  return ambe_newencoded;
}


__inline void ambe_getsilence(unsigned char *dest) {
  memcpy(dest, SilenceFrame, AMBE_USEDDATA);	// copy SilenceFrame (static)
}


void ambe_putvoice(unsigned char *voicedat) {
  if (AVR32_SSC.sr&AVR32_SSC_SR_TXEN_MASK) {
    // Wenn's noch nicht zu spät ist - überschreiben:
    if (AVR32_PDCA.channel[AMBE_CHANNEL1].tcr > (AMBE_FRAMESIZE/2-2))
      memcpy(ambe_input.Data, voicedat, AMBE_USEDDATA);
  } else {	// Transmitter Idle (default)
    memcpy(ambe_input.Data, voicedat, AMBE_USEDDATA);
    if (ambe_rxframe_count > 0) {		// AMBE arbeitet...
      ambe_input.Ctrl1 = AMBE_OPERATEID;	// CNI, LFI off
      ambe_start_transmit();
      ambe_restart_timer();		// reset Timeout for LFI/CNI
      ambe_idle_count = 0;		// new data -> Reset Timeout
    } // fi AMBE working
  }
}



__inline tambestate ambe_getstate(void) {
  return AMBEstate;
}


__inline tambestate ambe_getnewstate(void) {
  if (AMBEstate==prevAMBEstate)
    return AMBEnoinit;
  else {
    prevAMBEstate = AMBEstate;
    return AMBEstate;
  }
}


__inline int ambe_packetcount(void) {
  return ambe_rxframe_count;
}


unsigned short ambe_getber(void) {
  unsigned short ber = BitErrorRate;
  BitErrorRate |= 0x8000;	// LastError, must be updated with next voice pkt
  return ber;
}


__inline void ambe_set_vollin(unsigned char volume) {
  MSB(ambe_input.Ctrl2) = volume;
}


void ambe_set_dtmf(unsigned char tone, int len_frames) {
  if (len_frames==0)
    ambe_dtmf_count = 0xffffffff;
  else
    ambe_dtmf_count = len_frames;
  ambe_2ndtone_count = 0;
  ambe_tonepause_count = 0;
  ambe_2ndtone_val = 0xFF;
  LSB(ambe_input.DtmfCtrl) = tone;
  LSB(ambe_input.Ctrl2) = AMBE_CTRL2_RIS_NONE;
  if (AMBEstate==AMBEencoding) {
    ambe_start_transmit();
  }
}


void ambe_double_tone(unsigned char tone, int len_frames, int len_pause, unsigned char tone2nd) {
  ambe_dtmf_count = len_frames;
  ambe_2ndtone_count = len_frames;
  ambe_tonepause_count = len_pause;
  ambe_2ndtone_val = tone2nd;
  LSB(ambe_input.DtmfCtrl) = tone;
  LSB(ambe_input.Ctrl2) = AMBE_CTRL2_RIS_NONE;
  if (AMBEstate==AMBEencoding) {
    ambe_start_transmit();
  }
}


__inline void ambe_stop_dtmf(void) {
  ambe_dtmf_count = 1;
}


void ambe_set_timeout(unsigned int time2shutdown) {
  if (time2shutdown > 0)
    ambe_powerdown_to = time2shutdown;
}

//! @}



/*! \name AMBE Initialization
 */
//! @{

void ambe_init(void) {
  AMBEstate = AMBEnoinit;
  // first disable GPIO function on SSC pins:
  AVR32_GPIO.port[1].gperc = (1<<(AVR32_SSC_RX_CLOCK_0_PIN&0x1F))| \
                             (1<<(AVR32_SSC_RX_DATA_0_PIN&0x1F))| \
                             (1<<(AVR32_SSC_RX_FRAME_SYNC_0_PIN&0x1F))| \
                             (1<<(AVR32_SSC_TX_CLOCK_0_PIN&0x1F))| \
                             (1<<(AVR32_SSC_TX_DATA_0_PIN&0x1F))| \
                             (1<<(AVR32_SSC_TX_FRAME_SYNC_0_PIN&0x1F));
  // 2nd setting output GPIO control pins:
  AVR32_GPIO.port[0].ovrc  = (1<<AMBE_SOFTEN_PIN);	// soft decision disabled (low)
  AVR32_GPIO.port[0].ovrs  = (1<<AMBE_CSEL_PIN);	// passive framed channel mode
  AVR32_GPIO.port[0].oders = (1<<AMBE_CSEL_PIN)|(1<<AMBE_SOFTEN_PIN);

  // *** Synchronous Serial Controller init ***
  AVR32_SSC.cr   = 0x00008000;		// Do a software reset
  AVR32_SSC.cmr  = SSC_CLOCK_DIVIDER;	// SSC Clock Deiver for RK,TK Pins
  AVR32_SSC.rcmr = SSC_xCMR_CLKBYMCU;	// startdelay 1 clk, 20 Bits Period (9orig)
  AVR32_SSC.rfmr = SSC_xFMR_CLKBYMCU;	// RF used as Output
  AVR32_SSC.tcmr = SSC_xCMR_CLKBYMCU|SSC_xCMR_CKI_MASK;	// like RX, but bit shifted on raising edge
  AVR32_SSC.tfmr = SSC_xFMR_CLKBYMCU;	// 16 Bits + POS PULSE

  INTC_register_interrupt(&ambe_ssc_int, AVR32_SSC_IRQ, AMBE_TRXSTART_PRIO);

  // *** Init PDAC Receiver for SSC ***
  AVR32_PDCA.channel[AMBE_CHANNEL0].idr = 0x0FFF;
  AVR32_PDCA.channel[AMBE_CHANNEL0].cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  AVR32_PDCA.channel[AMBE_CHANNEL0].mar = (U32)&ambe_output;
  AVR32_PDCA.channel[AMBE_CHANNEL0].tcr = AMBE_FRAMESIZE;
  AVR32_PDCA.channel[AMBE_CHANNEL0].tcrr= 0;
  AVR32_PDCA.channel[AMBE_CHANNEL0].mr  = 0x01 << AVR32_PDCA_SIZE_OFFSET; // Half-Word
  AVR32_PDCA.channel[AMBE_CHANNEL0].psr = AVR32_PDCA_PID_SSC_RX;

  AVR32_PDCA.channel[AMBE_CHANNEL1].idr = 0x0FFF;
  AVR32_PDCA.channel[AMBE_CHANNEL1].cr  = AVR32_PDCA_ECLR_MASK|AVR32_PDCA_TDIS_MASK;
  AVR32_PDCA.channel[AMBE_CHANNEL1].mar = (U32)&ambe_input;
  AVR32_PDCA.channel[AMBE_CHANNEL1].tcr = AMBE_FRAMESIZE;
  AVR32_PDCA.channel[AMBE_CHANNEL1].tcrr= 0;
  AVR32_PDCA.channel[AMBE_CHANNEL1].mr  = 0x01 << AVR32_PDCA_SIZE_OFFSET; // Half-Word
  AVR32_PDCA.channel[AMBE_CHANNEL1].psr = AVR32_PDCA_PID_SSC_TX;

  INTC_register_interrupt(&ambe_rxfinish_int, AVR32_PDCA_IRQ_0+AMBE_CHANNEL0, AMBE_TRXEND_PRIO);
  INTC_register_interrupt(&ambe_txfinish_int, AVR32_PDCA_IRQ_0+AMBE_CHANNEL1, AMBE_TRXEND_PRIO);

  // *** Timer: Periodisch, Up bis RC-Compare, AutoTrg ***
  AMBE_TIMER.cmr =  AVR32_TC_NONE << AVR32_TC_BSWTRG_OFFSET |
    AVR32_TC_NONE << AVR32_TC_BEEVT_OFFSET |
    AVR32_TC_NONE << AVR32_TC_BCPC_OFFSET |
    AVR32_TC_NONE << AVR32_TC_BCPB_OFFSET |
    AVR32_TC_NONE << AVR32_TC_ASWTRG_OFFSET |
    AVR32_TC_NONE << AVR32_TC_AEEVT_OFFSET |
    AVR32_TC_NONE << AVR32_TC_ACPC_OFFSET |
    AVR32_TC_NONE << AVR32_TC_ACPA_OFFSET |
    1 << AVR32_TC_WAVE_OFFSET |
    AVR32_TC_WAVSEL_UP_AUTO << AVR32_TC_WAVSEL_OFFSET |
    FALSE << AVR32_TC_ENETRG_OFFSET |
    AVR32_TC_EEVT_TIOB_INPUT << AVR32_TC_EEVT_OFFSET |
    AVR32_TC_EEVTEDG_NO_EDGE << AVR32_TC_EEVTEDG_OFFSET |
    FALSE << AVR32_TC_CPCDIS_OFFSET |
    FALSE << AVR32_TC_CPCSTOP_OFFSET |
    AVR32_TC_BURST_NOT_GATED << AVR32_TC_BURST_OFFSET |
    0 << AVR32_TC_CLKI_OFFSET |
    AVR32_TC_TCCLKS_TIMER_CLOCK5 << AVR32_TC_TCCLKS_OFFSET;	// MASTERCLK/128

  // *** EPR - Packet-Ready-Int (one-shoot-trigger rx) ***
  eic_clrline(AMBE_EPR_INT);
  eic_enableint(AMBE_EPR_INT, &ambe_epr_int, AMBE_EPR_PRIO, EICm_FALLING_EDGE);
  AMBEstate = AMBEoff;
}


void ambe_setup(void) {
  ambe_powerdown_to = 0xFFFFFFF0;	// Disable Power-Down TO (huge Timeout)
  ambe_input.Header  = AMBE_HEADER;
  ambe_input.Rate[0] = AMBE_RATE0;
  ambe_input.Rate[1] = AMBE_RATE1;
  ambe_input.Rate[2] = AMBE_RATE2;
  ambe_input.Rate[3] = AMBE_RATE3;
  ambe_input.Rate[4] = AMBE_RATE4;
  ambe_input.Ctrl1   = (0x00<<AMBE_CTRL1_ID_OFFSET)|AMBE_CTRL1_CNI_MASK;
  ambe_input.Unused[0] = 0;
  ambe_input.Unused[1] = 0;
  ambe_input.Unused[2] = 0;
  ambe_input.DtmfCtrl = 0xFDFF;	// -3dB DTMFVol, Inactive
  ambe_input.Ctrl2 = (0x80<<AMBE_CTRL2_VOL_OFFSET)|AMBE_CTRL2_ENCODE|AMBE_CTRL2_RIS_BOTH;
  memset(&ambe_input.Data, 0, sizeof(ambe_input.Data));	// Clear Data
  VoicePktPtr = (char *)SilenceFrame;
}

//! @}
