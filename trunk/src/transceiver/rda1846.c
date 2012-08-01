/*
 * rda1846.c
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
 * Report:
 * 2012-08-01	Changed to reg_blocking_write()
 */


#include "rda1846.h"
#include "transceiver.h"
#include "twi_func.h"
#include "gpio_func.h"		// SLEEP() on init

#include "hw_defs.h"

#include "compiler.h"


#define RDA_TWI_ADR	0x71	// SEN pin low
#define RDA_TWI_2ndADR	0x2E	// SEN pin high


// Register naming, if known...
#define RDAREG_CTRL		0x30
// bits in control register:
#define RDAREG_CTRL_CHW25	0x3000		// select 25kHz channel width
#define RDAREG_CTRL_CHW12_5	0x0000		// select 12.kHz channel width
#define RDAREG_CTRL_TAIL_ELIM	0x0800		// enable tail elimination
#define RDAREG_CTRL_TRX_AUTO	0x0200		// ST-Mode: txon_rf & rxon_rf auto
#define RDAREG_CTRL_RX_AUTO	0x0100		// ST-Mode: txon_rf manual / rxon_rf auto
#define RDAREG_CTRL_TRX_MANU	0x0000		// ST-Mode: txon_rf & rxon_rf manual
#define RDAREG_CTRL_MUTE	0x0080		// Mute if RX on
#define RDAREG_CTRL_TXON	0x0040		// transmitting
#define RDAREG_CTRL_RXON	0x0020		// receiving
#define RDAREG_CTRL_VOXON	0x0010		// VOX logic on
#define RDAREG_CTRL_SQON	0x0008		// Squelch logic on
#define RDAREG_CTRL_PWRON	0x0004		// power for the internal circuits on
#define RDAREG_CTRL_CAL		0x0002		// chip calibration done (must be '1' after init)
#define RDAREG_CTRL_RESET	0x0001		// do a global register reset

#define RDAREG_CLKMODE		0x04		// select between 12-14 and 24-28MHz clock
#define RDAREG_CLKMODE_KEY	0x0F10

#define RDAREG_PABIAS		0x0A		// PA-BIAS voltage out adjust
#define RDAREG_PABIAS_MASK	0x001F
#define RDAREG_PABIAS_KEY	0x0400

#define RDAREG_RFBAND		0x0F
#define RDAREG_RFBAND_MASK	0x00C0
#define RDAREG_RFBAND_OFS	6
#define RDAREG_RFBAND_KEY	0x0024

#define RDAREG_FREQ_H		0x29
#define RDAREG_FREQ_L		0x2A
#define RDAREG_XTAL		0x2B
#define RDAREG_ADCLK		0x2C

#define RDAREG_FILTER		0x58
#define RDAREG_FILTER_DIS	0x002A		// disable the most of DSP-Filters in both paths

#define RDAREG_UPPER128		0x7F
#define RDAREG_UPPER128_EN	0x0001
#define RDAREG_UPPER128_DIS	0x0000

// Upper Register (special handling):
#define	RDAREG_RFOUT		0x85


#if (RDA_CLOCK>=12000000L)&&(RDA_CLOCK<=14000000L)
#define RDA_2B_XTAL_VALUE	(RDA_CLOCK/1000)
#define RDA_2C_ADCLK_VALUE	(RDA_CLOCK/2000)
#define RDA_04_CLKMODE_VALUE	(RDAREG_CLKMODE_KEY|1)

#elif (RDA_CLOCK>=24000000L)&&(RDA_CLOCK<=28000000L)
#define RDA_2B_XTAL_VALUE	(RDA_CLOCK/2000)
#define RDA_2C_ADCLK_VALUE	(RDA_CLOCK/4000)
#define RDA_04_CLKMODE_VALUE	RDAREG_CLKMODE_KEY
#else
 "RDA1846 don't support this crystal / osc frequency"
#endif


volatile bool rda_init_reply;
U32	rda_capabilities;
U16 	rda_ctrlreg;
U16	rda_rfband;
U32	rda_rx_freq, rda_tx_freq;



void reset_rda_handler(tTWIresult res, unsigned int len) {
  if (res == TWIok)
    rda_capabilities |= TRXCAP_AVAIL;
  rda_init_reply = true;
}


#ifdef SUPPORT_2ND_IC
void reset_rda2_handler(tTWIresult res, unsigned int len) {
  if (res == TWIok)
    rda_capabilities |= TRXCAP_DUPLEX;
}
#endif


void rda_reset(void) {
  rda_init_reply   = false;
  rda_capabilities = 0;
  reg_blocking_write(RDA_TWI_ADR, RDAREG_CTRL, RDAREG_CTRL_RESET, 2, reset_rda_handler);
#ifdef SUPPORT_2ND_IC
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_CTRL, RDAREG_CTRL_RESET, 2, reset_rda2_handler);
#endif
}


void rda_init_xtal(void) {
  reg_blocking_write(RDA_TWI_ADR, RDAREG_CLKMODE, RDA_04_CLKMODE_VALUE, 2, NULL);
  reg_blocking_write(RDA_TWI_ADR, RDAREG_XTAL, RDA_2B_XTAL_VALUE, 2, NULL);
  reg_blocking_write(RDA_TWI_ADR, RDAREG_ADCLK, RDA_2C_ADCLK_VALUE, 2, NULL);
#ifdef SUPPORT_2ND_IC
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_CLKMODE, RDA_04_CLKMODE_VALUE, 2, NULL);
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_XTAL, RDA_2B_XTAL_VALUE, 2, NULL);
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_ADCLK, RDA_2C_ADCLK_VALUE, 2, NULL);
#endif
}


void rda_rfoutput(char value) {
  // see register table:
  // value = 0x1F ->  8dbm
  // value = 0x18 ->  6dbm
  // value = 0x17 -> -3dbm
  reg_blocking_write(RDA_TWI_ADR, RDAREG_UPPER128, RDAREG_UPPER128_EN, 2, NULL);
  reg_blocking_write(RDA_TWI_ADR, RDAREG_RFOUT&0x7F, value, 2, NULL);
  reg_blocking_write(RDA_TWI_ADR, RDAREG_UPPER128, RDAREG_UPPER128_DIS, 2, NULL);
#ifdef SUPPORT_2ND_IC
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_UPPER128, RDAREG_UPPER128_EN, 2, NULL);
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_RFOUT&0x7F, value, 2, NULL);
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_UPPER128, RDAREG_UPPER128_DIS, 2, NULL);
#endif
}


void rda_update_ctrl(void) {
  reg_blocking_write(RDA_TWI_ADR, RDAREG_CTRL, rda_ctrlreg, 2, NULL);
#ifdef SUPPORT_2ND_IC
  reg_blocking_write(RDA_TWI_2ndADR, RDAREG_CTRL, rda_ctrlreg2, 2, NULL);
#endif
}



unsigned int rda_init(void) {
  rda_reset();
  rda_rx_freq = 0;
  rda_tx_freq = 0;
  while ((twi_busy()) && (!rda_init_reply)) {
    SLEEP();	// wait reset-answer
  }
  if (rda_init_reply && (rda_capabilities&TRXCAP_AVAIL)) {
#ifdef DEBUG
    LED_Set(LED_GREEN);
#endif
    reg_blocking_write(RDA_TWI_ADR, RDAREG_CTRL, RDAREG_CTRL_PWRON, 2, NULL);
    reg_blocking_write(RDA_TWI_ADR, 0x09, 0x03AC, 2, NULL);	// Set GPIO voltages from 2.7V to VCC
    reg_blocking_write(RDA_TWI_ADR, 0x0B, 0x1A10, 2, NULL);	// nix Ahnung
    rda_init_xtal();
//    while (twi_busy()) SLEEP();
    /*
    reg_blocking_write(RDA_TWI_ADR, 0x32, 0x627C, 2, NULL);	// no Ahnung, nicht relevant bei openDV
    reg_blocking_write(RDA_TWI_ADR, 0x33, 0x0AF2, 2, NULL);	// no Ahnung, nicht relevant bei openDV
    reg_blocking_write(RDA_TWI_ADR, 0x47, 0x1AEA, 2, NULL);	// no Ahnung, nicht relevant bei openDV
    reg_blocking_write(RDA_TWI_ADR, 0x4e, 0x293A, 2, NULL);	// no Ahnung, nicht relevant bei openDV
    reg_blocking_write(RDA_TWI_ADR, 0x54, 0x1D40, 2, NULL);	// SQ out sel - but invalid value here
    reg_blocking_write(RDA_TWI_ADR, 0x56, 0x0652, 2, NULL);	// no Ahnung, nicht relevant bei openDV
    reg_blocking_write(RDA_TWI_ADR, 0x71, 0x6C1E, 2, NULL);	// no Ahnung, nicht relevant bei openDV
    SLEEP();
    */
    // disable DSP Filters and Pre/Deemphasis:
    reg_blocking_write(RDA_TWI_ADR, RDAREG_FILTER, RDAREG_FILTER_DIS, 2, NULL);
    rda_rfoutput(0x1F);
//    while (twi_busy()) SLEEP();
    rda_ctrlreg = RDAREG_CTRL_PWRON|RDAREG_CTRL_CAL;
    rda_update_ctrl();
    rda_capabilities |= TRXCAP_VHF|TRXCAP_UHF;
  } // fi
  return rda_capabilities;
}



char rda_set_currfreq(U32 curr_freq) {
  U16 rfband, ctrlreg_save = rda_ctrlreg;
  // checking...
  if ((curr_freq >= 3200000)&&(curr_freq <= 4160000)) {		// 400-520MHz
    rfband = RDAREG_RFBAND_KEY;
  } else if ((curr_freq >= 1600000)&&(curr_freq <= 2080000)) {	// 200-260MHz
    rfband = (2<<RDAREG_RFBAND_OFS) | RDAREG_RFBAND_KEY;
  } else if ((curr_freq >= 1072000)&&(curr_freq <= 1392000)) {	// 134-174MHz
    rfband = (3<<RDAREG_RFBAND_OFS) | RDAREG_RFBAND_KEY;;
  } else return false;	// Error
  if (rda_ctrlreg & (RDAREG_CTRL_RXON|RDAREG_CTRL_TXON)) {	// Disable...
    rda_ctrlreg &= ~(RDAREG_CTRL_RXON|RDAREG_CTRL_TXON);
    rda_update_ctrl();
  } // fi
  if (rda_rfband != rfband) {
    rda_rfband = rfband;
    reg_blocking_write(RDA_TWI_ADR, RDAREG_RFBAND, rda_rfband, 2, NULL);
  } // fi
  reg_blocking_write(RDA_TWI_ADR, RDAREG_FREQ_H, curr_freq>>16, 2, NULL);
  reg_blocking_write(RDA_TWI_ADR, RDAREG_FREQ_L, curr_freq&0xFFFF, 2, NULL);

  if (rda_ctrlreg != ctrlreg_save) {
    rda_ctrlreg = ctrlreg_save;
    rda_update_ctrl();
  }
  return true;
}


unsigned int rda_setfreq(unsigned int rx_freq, unsigned int tx_freq) {
  rda_rx_freq  = rx_freq/125;
  rda_tx_freq  = tx_freq/125;	// /125 prevent overflow
  if (rda_set_currfreq((rda_ctrlreg & RDAREG_CTRL_TXON)?rda_tx_freq:rda_rx_freq))
    return rda_rx_freq*125;
  else
    return 0;
}


void rda_standby(void) {
  if ((rda_ctrlreg & RDAREG_CTRL_PWRON) == 0) {
    rda_ctrlreg &= ~RDAREG_CTRL_PWRON;
    rda_update_ctrl();
  }
}


void rda_transmit(void) {
  if ((rda_tx_freq > 0) && ((rda_ctrlreg & (RDAREG_CTRL_TXON|RDAREG_CTRL_RXON)) != RDAREG_CTRL_TXON)) {
    rda_ctrlreg &= ~RDAREG_CTRL_RXON;
    rda_ctrlreg |= RDAREG_CTRL_TXON|RDAREG_CTRL_PWRON;
    if (rda_rx_freq != rda_tx_freq)
      rda_set_currfreq(rda_tx_freq);	// this die a ctrl_reg update too
    else
      rda_update_ctrl();
  }
}


void rda_receive(void) {
  if ((rda_rx_freq > 0) && ((rda_ctrlreg & (RDAREG_CTRL_TXON|RDAREG_CTRL_RXON)) != RDAREG_CTRL_RXON)) {
    rda_ctrlreg &= ~RDAREG_CTRL_TXON;
    rda_ctrlreg |= RDAREG_CTRL_RXON|RDAREG_CTRL_PWRON;
    if (rda_rx_freq != rda_tx_freq)
      rda_set_currfreq(rda_rx_freq);	// this die a ctrl_reg update too
    else
      rda_update_ctrl();
  }
}


#ifdef SUPPORT_2ND_IC
void	rda2nd_transmit(void) {

}


void	rda2nd_receive(void) {

}
#endif
