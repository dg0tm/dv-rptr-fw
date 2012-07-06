/*
 * TLV320AIC.c
 *
 *  Created on: 05.04.2012
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
 * Report:
 * 2012-06-30	Config C6 added, stores 28x16bit Coeeffs in LittleEndian Format.
 * 2012-07-06	TLV_present bool enabled I²C register write only if HW found on init()
 */

#include "TLV320AIC.h"

#include "compiler.h"
#include "defines.h"
#include "hw_defs.h"
#include "gpio_func.h"

#include "twi_func.h"
#include <string.h>

#ifdef AMBE_PROTOTYPE
#define TLV_MASTERCLOCK		12288
#else
#define TLV_MASTERCLOCK		16384	// in kHz (old = 12288)
#endif


#define TLV_SAMPLINGCLOCK	4096	// in kHz (used for oversampling)

#define TLV_MDIVIDER		(TLV_MASTERCLOCK/TLV_SAMPLINGCLOCK)

#define TLV_TWI_ADR		0x18

#define TLV_REG_PAGECTRL	0x00	// 8bit in every Page the same
// Page0 Registers
#define TLV_REG_SOFTRESET	0x0001	// MSB = Page, LSB = Register in Page

#define TLV_REG_NDAC		0x000B
#define TLV_REG_MDAC		0x000C
#define TLV_REG_DOSRMSB		0x000D
#define TLV_REG_DOSRLSB		0x000E

#define TLV_REG_NADC		0x0012
#define TLV_REG_MADC		0x0013
#define TLV_REG_AOSR		0x0014

#define TLV_REG_IFACECTRL	0x001B
#define TLV_REG_DOUTOFS		0x001C

#define TLV_REG_IFACECTL2	0x001D
#define TLV_REG_BCLK_NDIV	0x001E

#define TLV_REG_GPIO1CTRL	0x0033
#define TLV_REG_DOUTCTRL	0x0035
#define TLV_REG_DINCTRL		0x0036

#define TLV_REG_DACPBLOCK	0x003C
#define TLV_REG_ADCPBLOCK	0x003D
#define TLV_REG_DACPATHSET	0x003F
#define TLV_REG_DACVOLCTRL	0x0040
#define TLV_REG_DACVOLUME	0x0041
#define TLV_REG_DRC_CTRL1	0x0044
#define TLV_REG_DRC_CTRL2	0x0045
#define TLV_REG_DRC_CTRL3	0x0046
#define TLV_REG_ADC_MIC		0x0051
#define TLV_REG_ADCVOLFINEADJ	0x0052	// ADC Digital Volume Control Fine Adjust
#define TLV_REG_ADCVOLUMEADJ	0x0053	// ADC Digital Volume Control Coarse Adjust
#define TLV_REG_AGC_CTRL1	0x0056	// AGC Control 1
#define TLV_REG_AGC_CTRL2	0x0057	// AGC Control 2
#define TLV_REG_AGC_MAXGAIN	0x0058	// AGC Maximum Gain
#define TLV_REG_AGC_ATTACK	0x0059	// AGC Attack TIme
#define TLV_REG_AGC_DECAY	0x005A	// AGC Decay Time
#define TLV_REG_AGC_NOISEDEB	0x005B	// AGC Noise Debounce
#define TLV_REG_AGC_SIGDEB	0x005C	// AGC Signal Debounce
#define TLV_REG_AGC_GAINREAD	0x005D	// AGC Gain-Applied Reading (RO)

#define TLV_REG_SARADCVOLCTRL	0x0074	// VOL/MICDET-Pin SAR ADC Volume Control


// Page 1 Registers
#define TLV_REG_AMP_ERRCTRL	0x011E	// Headphone + Speaker Amp Error Control
#define TLV_REG_HP_DRIVER	0x011F	// Headphone Drivers
#define TLV_REG_SPEAKERAMP	0x0120	// Class-D Speaker Amplifier
#define TLV_REG_DACROUTING	0x0123
#define TLV_REG_HPOUTANAVOL	0x0124
#define TLV_REG_SPKRANAVOL	0x0126
#define TLV_REG_HPOUTDRIVER	0x0128	// Headphone (HPOUT pin) Driver
#define TLV_REG_SPKRDRIVER	0x012A	// Class-D Output Driver

#define TLV_REG_MICBIAS		0x012E	// Microphone Bias Settings
#define TLV_REG_MICPGA		0x012F	// Microphone PGA
#define TLV_REG_MIC_P_TERM	0x0130	// DeltaSigma MonoADC Ch Fine-Gain InputSel for P-Terminal
#define TLV_REG_MIC_M_TERM	0x0131	// DeltaSigma MonoADC Ch Fine-Gain InputSel for M-Terminal
#define TLV_REG_MIC_INPUTS	0x0132	// Input CM Settings

// Page 4 ADC Digital Filters
#define TLV_REG_DFC_BASE	0x0400

#define TLV_SOFTRESET_WAIT	(MASTERCLOCK/10000*20)	// 2.0ms (Minimum 1ms)



// configuration of analog frontend
typedef struct PACKED_DATA {
  unsigned char	mic_pga;	// Microphone PGA Gain 0..59.5dB (0.5dB steps)
  unsigned char	mic_imp;	// Microphone Impedance Value
  signed   char	adc_gain;	// ADC Gain Value -12..20dB (0.5dB steps)
  unsigned char adc_filter;	// Selection between PB4,PB5 and PB6
  unsigned char	spkr_out;	// Ext. Speaker on/off bit7; 6,12,18,24dB (6dB steps)
  signed   char hs_out;		// handset speaker on/off bit7; gain 0..9dB (1dB steps)
  signed   char volume;		// DAC volume (-63.5dB .. +24dB; 0.5dB steps) 0x7F = Poti
  unsigned char dac_filter;	// DAC filter selection
} t_config_4;


typedef struct PACKED_DATA {
  unsigned char agc_ctrl1;	// Automatic Gain Control (Bit7 enable)
  unsigned char agc_ctrl2;	// see TLV320AIC datasheet (register pge0,0x56ff)
  unsigned char agc_maxgain;	// maximum gain 0..59.5dB (0.5dB steps)
  unsigned char agc_attack;	// attack time
  unsigned char agc_decay;	// decay time
  unsigned char agc_noisedeb;	// noise debounce
  unsigned char agc_signaldeb;	// signal debounce
  unsigned char rsvd1;
  unsigned char drc_ctrl1;	// Dynamic Range Compression (Bit6 enable)
  unsigned char drc_ctrl2;	// see TLV320AIC datasheet (register pge0,0x44,0x45,0x46)
  unsigned char drc_ctrl3;
  unsigned char rsvd2;
} t_config_5;


t_config_4 CONFIG_C4;
t_config_5 CONFIG_C5;



volatile U8		TLV_page;	// I²C Page value (set page only, when changes
volatile tTWIresult	TLV_res;
bool			TLV_present = false;


#ifdef AMBE_PROTOTYPE

#define pll_set_strobe()	gpio1_set(PLL_STROBE_PIN)
#define pll_clr_strobe()	gpio1_clr(PLL_STROBE_PIN)
#define pll_set_data()		gpio1_set(PLL_DATA_PIN)
#define pll_clr_data()		gpio1_clr(PLL_DATA_PIN)
#define pll_set_clock()		gpio1_set(PLL_CLOCK_PIN)
#define pll_clr_clock()		gpio1_clr(PLL_CLOCK_PIN)


void pll_write16(U16 data) {
  int bitcnt;
  pll_clr_clock();
  pll_clr_strobe();
  for (bitcnt=0; bitcnt<16; bitcnt++) {
    if (data&0x8000)
      pll_set_data();
    else
      pll_clr_data();
    pll_set_clock();
    data <<= 1;
    pll_clr_clock();
  } // rof
  pll_set_strobe();
  pll_set_data();
}


#define PLL_MODEREG_VAL		0x70A6	// enable SCKO1 and 23 only, setting 64kHz SRate
#define PLL_CONFREG_VAL		0x6C10	// set SCKO1 to 12.288MHz


/* Initialization (Prototype only)
 */
void init_pll1708(void) {
  AVR32_GPIO.port[1].ovr   = GPIO1_OVRINIT|(1<<PLL_DATA_PIN)|(1<<PLL_CLOCK_PIN)|(1<<PLL_STROBE_PIN);
  AVR32_GPIO.port[1].oder  = GPIO1_ODER|(1<<PLL_DATA_PIN)|(1<<PLL_CLOCK_PIN)|(1<<PLL_STROBE_PIN);
  AVR32_GPIO.port[1].gpers = (1<<PLL_DATA_PIN)|(1<<PLL_CLOCK_PIN)|(1<<PLL_STROBE_PIN);  // all Pins are GPIOs
  pll_write16(PLL_MODEREG_VAL);
  pll_write16(PLL_CONFREG_VAL);
  AVR32_GPIO.port[1].ovr   = GPIO1_OVRINIT;		// restore PortB to default
  AVR32_GPIO.port[1].oder  = GPIO1_ODER;
  AVR32_GPIO.port[1].gperc = GPIO1_DISABLE_MASK;
}
#endif // Prototype (with PLL1708)



void tlv_writereg(unsigned short reg, unsigned char value) {
  if (TLV_present) {
    if (MSB(reg) != TLV_page) {
      TLV_page = MSB(reg);
      reg_write(TLV_TWI_ADR, TLV_REG_PAGECTRL, TLV_page, 1, NULL);
    } // fi change page
    reg_write(TLV_TWI_ADR, LSB(reg), value, 1, NULL);
  } // fi fond HW
}


void tlv_getpage_hnd(tTWIresult twires, unsigned int len) {
  TLV_res = twires;
}


void tlv_set_gpio1(void) {
  tlv_writereg(TLV_REG_GPIO1CTRL, 0x0D);
}


void tlv_clr_gpio1(void) {
  tlv_writereg(TLV_REG_GPIO1CTRL, 0x0C);
}


// Simple Wait Function
void tlv_tickwait(U32 ticks_to_wait) {
  U32 systick;
  U32 strttick = Get_system_register(AVR32_COUNT);
  systick = strttick + ticks_to_wait;
  if (systick < strttick) while (Get_system_register(AVR32_COUNT) > strttick) {
    watchdog();
  }
  while (Get_system_register(AVR32_COUNT) < systick) {
    watchdog();
  } // elihw
}



void tlv_twiwait(void) {
  int maxw;
  for (maxw=5000; (maxw>0)&&(twi_busy()); maxw--) {
    SLEEP();
    watchdog();
  }
}


void tlv_mute_both(void) {
  tlv_writereg(TLV_REG_DACVOLCTRL, 0x0C);	// mute DAC
  tlv_writereg(TLV_REG_ADCVOLFINEADJ, 0x80);	// mute ADC
  tlv_writereg(TLV_REG_ADC_MIC, 0x00);		// PowerDown ADC
  // (DAC must be enabled for keeping FramePulses)
}


void tlv_unmute_adc(void) {
  tlv_writereg(TLV_REG_ADC_MIC, 0x80);		// PowerUp ADC (SoftStepping enabled, AnalogMic)
  tlv_writereg(TLV_REG_ADCVOLFINEADJ, 0x00);	// unmute ADC
}


void tlv_unmute_dac(void) {
  tlv_writereg(TLV_REG_DACVOLCTRL, 0x04);	// unmute DAC
}


/* tlv_set_adcgain()
 * gain in 0.5dB steps (-12dB..20dB ^= -24..40 allowed)
 */
char tlv_set_adcgain(signed char gain) {
  if (gain > 40) gain = 40; else if (gain < -24) gain = -24;
  tlv_writereg(TLV_REG_ADCVOLUMEADJ, 0x40 + gain);	// set coarse Gain to value
  return gain;
}


void tlv_set_DACvolume(signed char vol) {
  if (vol != 0x7F) {		// not controlled by PIN?
    if (vol > 48) vol = 48;	// max. +24dB
    tlv_writereg(TLV_REG_DACVOLUME, vol);
    tlv_writereg(TLV_REG_SARADCVOLCTRL, 0x40);	// disable VOL/MICDET-Pin
  } else {
    tlv_writereg(TLV_REG_SARADCVOLCTRL, 0xC0);	// enable VOL/MICDET-Pin
  }
}



void tlv_set_SPKRvolume(signed char vol) {
  if (vol > 36) vol = 36;
  if (vol >= 0) {
    char stage_gain = vol / 12;
    char stage_attn = vol % 12;
    if ((stage_attn == 0) || (stage_gain == 3)) {
      tlv_writereg(TLV_REG_SPKRDRIVER, 0x04 | (stage_gain << 3));
      tlv_writereg(TLV_REG_SPKRANAVOL, 0x80);
    } else {
      tlv_writereg(TLV_REG_SPKRDRIVER, 0x04 | ((stage_gain+1) << 3));
      tlv_writereg(TLV_REG_SPKRANAVOL, 12-stage_attn);
    }
  } else {
   tlv_writereg(TLV_REG_SPKRANAVOL, -vol);
   tlv_writereg(TLV_REG_SPKRDRIVER, 0x0C);
  }
  tlv_writereg(TLV_REG_SPEAKERAMP, 0x06 | ((vol != -127)?0x80:0) );	// PowerUP
}


void tlv_set_HSvolume(signed char vol) {
  if (vol > 18) vol = 18;
  if (vol >= 0) {
    tlv_writereg(TLV_REG_HPOUTANAVOL, vol & 0x01);	// 0dB or -0.5dB analog volume
    if (vol < 18) vol++;
    tlv_writereg(TLV_REG_HPOUTDRIVER, 0x06 | (vol<<2));	// headphone PGA
  } else {
    tlv_writereg(TLV_REG_HPOUTANAVOL, -vol);
    tlv_writereg(TLV_REG_HPOUTDRIVER, 0x06);	// headphone: PGA=0dB, not muted
  }
  tlv_writereg(TLV_REG_HP_DRIVER, 0x1C | ((vol != -127)?0x80:0) );	// PowerUP
}



int tlv_init(void) {
  TLV_present = false;
#ifdef AMBE_PROTOTYPE
  init_pll1708();
#endif
  TLV_res = TWIbusy;
  reg_read(TLV_TWI_ADR, TLV_REG_PAGECTRL, (char *)&TLV_page, 1, tlv_getpage_hnd);
  // init structures (part 1)
  memset(&CONFIG_C4, 0, sizeof(CONFIG_C4));
  memset(&CONFIG_C5, 0, sizeof(CONFIG_C5));
  while (TLV_res == TWIbusy) {
    SLEEP();
  }
  if (TLV_res != TWIok) return false;		// exit, if no response from TLV320AIC (no board)
  TLV_present = true;

  tlv_writereg(TLV_REG_SOFTRESET, 0x01);
  tlv_tickwait(TLV_SOFTRESET_WAIT);		// wait >1ms before TLC is accessible again
  tlv_writereg(TLV_REG_IFACECTRL, 0x4C);	// set DSP-mode, Word+Bitclock outputs
  tlv_writereg(TLV_REG_NDAC, 0x81);		// enable divider, divider=1 (=MASTERCLOCK)
  tlv_writereg(TLV_REG_MDAC, 0x80|TLV_MDIVIDER); // enable MDAC divider, divider=3 or 4
  tlv_writereg(TLV_REG_NADC, 0x81);		// enable divider, divider=1
  tlv_writereg(TLV_REG_MADC, 0x80|TLV_MDIVIDER); // enable MADC divider, divider=3 or 4
  tlv_writereg(TLV_REG_DOUTOFS, 0x01);		// 1 BCLK offset OUT
  tlv_writereg(TLV_REG_IFACECTL2, 0x05);	// Clock ever from DAC_MOD_CLK (4.096MHz)
  tlv_writereg(TLV_REG_BCLK_NDIV, 0x81);	// BCLK divider active (by 1)
  // Select processings blocks:
  tlv_writereg(TLV_REG_DACPBLOCK, 0x04);	// select PRB_R4 for DAC
  tlv_writereg(TLV_REG_ADCPBLOCK, 0x04);	// select PRB_R4 for ADC/Microphone

  // Setup and enable speaker (testing)
  tlv_writereg(TLV_REG_HP_DRIVER, 0x9C);	// PowerUP Headphone, Common Voltage 1.8V
  tlv_writereg(TLV_REG_DACROUTING, 0x40);	// DAC routed to mixer amplifier
  tlv_writereg(TLV_REG_HPOUTANAVOL, 0x00);	// 0dB Headphone
  tlv_writereg(TLV_REG_HPOUTDRIVER, 0x36);	// headphone: PGA=6dB, not muted
  tlv_writereg(TLV_REG_SPEAKERAMP, 0x86);	// PowerUP Class-D Amplifier
  tlv_writereg(TLV_REG_SPKRANAVOL, 0x80);	// routing Analog Volume Control Output to Speaker
  tlv_writereg(TLV_REG_SPKRDRIVER, 0x0C);	// Class-D amplifier: 12dB, not muted

  // Microphone PGA and Input Settings (testing)
  tlv_writereg(TLV_REG_MICBIAS, 0x0A);		// Microphone Bias: 2.5V independent of HeadphoneDET
  tlv_writereg(TLV_REG_MICPGA, 0x3F);		// Microphone PGA = 31.5dB
  tlv_writereg(TLV_REG_MIC_P_TERM, 0x40 );	// P-Terminal: MIC1LP with 10kOhm
  tlv_writereg(TLV_REG_MIC_M_TERM, 0x20 );	// M-Terminal: MIC1LM selected with 20kOhm
  tlv_writereg(TLV_REG_MIC_INPUTS, 0x00 );	// Input CM Settings

  tlv_tickwait(TLV_SOFTRESET_WAIT);

  // DAC settings:
  tlv_writereg(TLV_REG_DACPATHSET, 0x94);	// PowerUp DAC (for LEFT channel only)
  tlv_writereg(TLV_REG_DACVOLUME, 12);		// 6dB digital GAIN
  tlv_writereg(TLV_REG_SARADCVOLCTRL, 0xC0);	// enable VOL/MICDET-Pin for DAC volume control

  // ADC (Microphone) settings:
  tlv_writereg(TLV_REG_ADCVOLUMEADJ, 0x40);	// set coarse Gain to 0dB (-12..20dB in 0.5dB Steps)

  // init structures (part 2)
  CONFIG_C4.mic_pga = 0x3F;
  CONFIG_C4.mic_imp = 0x2A;	// BIAS + Impedance
  CONFIG_C4.volume  = 0x7F;	// DAC volume knob-controlled
  CONFIG_C5.agc_maxgain = 119;
  CONFIG_C5.drc_ctrl1   = 0x6F;
  CONFIG_C5.drc_ctrl2   = 0x38;

  tlvfilter_default_lowpass();			// apply a 240Hz lowpass by default

  return true;
}


char *cfg_read_c4(char *config_buffer) {
  config_buffer[0] = 0xC4;			// identifier byte
  config_buffer[1] = CONFIG_C4_SIZE;		// length
  memcpy(config_buffer + 2, &CONFIG_C4, CONFIG_C4_SIZE);
  return config_buffer + 2 + CONFIG_C4_SIZE;
}


char *cfg_read_c5(char *config_buffer) {
  config_buffer[0] = 0xC5;			// identifier byte
  config_buffer[1] = CONFIG_C5_SIZE;		// length
  memcpy(config_buffer + 2, &CONFIG_C5, CONFIG_C5_SIZE);
  return config_buffer + 2 + CONFIG_C5_SIZE;
}



void cfg_write_c4(const char *config_data) {
  U8 mic_m_term, mic_p_term;
  memcpy(&CONFIG_C4, config_data, sizeof(CONFIG_C4));
  if (TLV_present) {
    // Apply:
    if (CONFIG_C4.mic_pga > 119) CONFIG_C4.mic_pga = 119;
    tlv_writereg(TLV_REG_MICPGA, CONFIG_C4.mic_pga);	// Microphone PGA gain

    // Microphone Impedance + BIAS Value
    // Bits 0..1 = MicBIAS:
    // 00 = no BIAS (dynamic microphones)
    // 01 = 2V
    // 10 = 2.5V (default)
    // 11 = 3.3V
    // all: independent of HeadphoneDET
    tlv_writereg(TLV_REG_MICBIAS, 0x08 | (CONFIG_C4.mic_imp & 3) );
    // Bits 2..4 = P-Terminal Impedance
    switch ((CONFIG_C4.mic_imp>>2) & 0x07) {
    default:
    case 0:
      mic_p_term = 0xC0;	// P-Terminal: MIC1LP with 40kOhm
      break;
    case 1:
      mic_p_term = 0x80;	// P-Terminal: MIC1LP with 20kOhm
      break;
    case 2:
      mic_p_term = 0x40;	// P-Terminal: MIC1LP with 10kOhm
      break;
    case 3:
      mic_p_term = 0xF0;	// P-Terminal: MIC1L+R with 40kOhm
      break;
    case 4:
      mic_p_term = 0xA0;	// P-Terminal: MIC1L+R with 20kOhm
      break;
    case 5:
      mic_p_term = 0x50;	// P-Terminal: MIC1L+R with 10kOhm
      break;
    } // hctiws impedance
    // Bits 5..7 = M-Terminal Impedance
    switch ((CONFIG_C4.mic_imp>>5)) {
    default:
    case 0:
      mic_m_term = 0x30;	// M-Terminal: MIC1LM with 40kOhm
      break;
    case 1:
      mic_m_term = 0x20;	// M-Terminal: MIC1LM with 20kOhm
      break;
    case 2:
      mic_m_term = 0x10;	// M-Terminal: MIC1LM with 10kOhm
      break;
    } // hctiws impedance

    tlv_writereg(TLV_REG_MIC_P_TERM, mic_p_term );
    tlv_writereg(TLV_REG_MIC_M_TERM, mic_m_term );

    if (CONFIG_C4.hs_out > 18) CONFIG_C4.hs_out = 18;
    tlv_set_HSvolume(CONFIG_C4.hs_out);

    if (CONFIG_C4.spkr_out > 48) CONFIG_C4.spkr_out = 48;
    tlv_set_SPKRvolume(CONFIG_C4.spkr_out);

    // page0 access
    CONFIG_C4.adc_gain = tlv_set_adcgain(CONFIG_C4.adc_gain);

    if ((CONFIG_C4.volume != 0x7F) && (CONFIG_C4.volume > 48))
      CONFIG_C4.volume = 48; // max. +24dB

    tlv_set_DACvolume(CONFIG_C4.volume);

    if (CONFIG_C4.adc_filter > 2) CONFIG_C4.adc_filter = 0;
    tlv_writereg(TLV_REG_ADCPBLOCK, 0x04 + CONFIG_C4.adc_filter);

    if (CONFIG_C4.dac_filter > 2) CONFIG_C4.dac_filter = 0;
    tlv_writereg(TLV_REG_DACPBLOCK, 0x04 + CONFIG_C4.dac_filter);
  } // fi present
}


void cfg_write_c5(const char *config_data) {
  memcpy(&CONFIG_C5, config_data, sizeof(CONFIG_C5));
  // first limit values:
  CONFIG_C5.agc_ctrl1 &= 0xF0;	// Automatic Gain Control
  CONFIG_C5.agc_ctrl2 &= 0xFE;
  CONFIG_C5.agc_noisedeb  &= 0x1F;	// noise debounce
  CONFIG_C5.agc_signaldeb &= 0x0F;	// signal debounce
  if (CONFIG_C5.agc_maxgain > 119) CONFIG_C5.agc_maxgain = 119;
  CONFIG_C5.drc_ctrl1 &= 0x7F;	// Dynamic Range Compression
  CONFIG_C5.drc_ctrl2 &= 0x7F;
  if (TLV_present) {
    tlv_writereg(TLV_REG_AGC_CTRL1, CONFIG_C5.agc_ctrl1);
    tlv_writereg(TLV_REG_AGC_CTRL2, CONFIG_C5.agc_ctrl2);
    tlv_writereg(TLV_REG_AGC_MAXGAIN, CONFIG_C5.agc_maxgain);
    tlv_writereg(TLV_REG_AGC_ATTACK, CONFIG_C5.agc_attack);
    tlv_writereg(TLV_REG_AGC_DECAY, CONFIG_C5.agc_decay);
    tlv_writereg(TLV_REG_AGC_NOISEDEB, CONFIG_C5.agc_noisedeb);
    tlv_writereg(TLV_REG_AGC_SIGDEB, CONFIG_C5.agc_signaldeb);
    tlv_writereg(TLV_REG_DRC_CTRL1, CONFIG_C5.drc_ctrl1);
    tlv_writereg(TLV_REG_DRC_CTRL2, CONFIG_C5.drc_ctrl2);
    tlv_writereg(TLV_REG_DRC_CTRL3, CONFIG_C5.drc_ctrl3);
  } // fi present
}



/*! \name TLV320AIC Filter Section
 */
//! @{

// all filters calculated with "TIBQ"

static const S16 FirstOrderIIR_240Hz[3] = {	// format N0,N1,D1
  0x7D0C,
  0x82F4,
  0x7A1A
};

#define DEFAULT_LP_IIR		FirstOrderIIR_240Hz


typedef struct PACKED_DATA {
  S16	FirstOrderIIR[3];
  S16	BQBlockA[5];
  S16	BQBlockB[5];
  S16	BQBlockC[5];
  S16	BQBlockD[5];
  S16	BQBlockE[5];
} tfilter_config;

tfilter_config	CONFIG_C6;


/* 1s order IIR: this is the last filter-block (after digital volume control)
 * active in every filter-selection
 */
void tlvfilter_load_iir(const S16 *coeffs) {
  tlv_twiwait();
  tlv_writereg(TLV_REG_DFC_BASE+ 8, MSB(coeffs[0]));
  tlv_writereg(TLV_REG_DFC_BASE+ 9, LSB(coeffs[0]));
  tlv_writereg(TLV_REG_DFC_BASE+10, MSB(coeffs[1]));
  tlv_writereg(TLV_REG_DFC_BASE+11, LSB(coeffs[1]));
  tlv_writereg(TLV_REG_DFC_BASE+12, MSB(coeffs[2]));
  tlv_writereg(TLV_REG_DFC_BASE+13, LSB(coeffs[2]));
}


void tlvfilter_default_lowpass(void) {
  memset(&CONFIG_C6, 0, sizeof(CONFIG_C6));
  memcpy(CONFIG_C6.FirstOrderIIR, DEFAULT_LP_IIR, 3);
  tlvfilter_load_iir(DEFAULT_LP_IIR);
  CONFIG_C6.BQBlockA[0] = 0x7FFF;	// init value, identical to TLV320 reset-value
  CONFIG_C6.BQBlockB[0] = 0x7FFF;	// init value, identical to TLV320 reset-value
  CONFIG_C6.BQBlockC[0] = 0x7FFF;	// init value, identical to TLV320 reset-value
  CONFIG_C6.BQBlockD[0] = 0x7FFF;	// init value, identical to TLV320 reset-value
  CONFIG_C6.BQBlockE[0] = 0x7FFF;	// init value, identical to TLV320 reset-value
}



/* all five biquad filter-blocks used on filter selection PRB_R5 (1)
 * the same function loads 25 FIR coefficients if filter selection PRB_R6 (2)
 */
void tlvfilter_load_bqfir(int nr, const S16 *coeffs) {
  U16 BiquadRegStart;
  if (nr > 4) return; // only Biquad A,B,C,D,E possible
  BiquadRegStart = TLV_REG_DFC_BASE + 14 + (nr * 10);
  tlv_twiwait();
  tlv_writereg(BiquadRegStart  , MSB(coeffs[0]));	// N0
  tlv_writereg(BiquadRegStart+1, LSB(coeffs[0]));
  tlv_writereg(BiquadRegStart+2, MSB(coeffs[1]));	// N1
  tlv_writereg(BiquadRegStart+3, LSB(coeffs[1]));
  tlv_writereg(BiquadRegStart+4, MSB(coeffs[2]));	// N2
  tlv_writereg(BiquadRegStart+5, LSB(coeffs[2]));
  tlv_writereg(BiquadRegStart+6, MSB(coeffs[3]));	// D1
  tlv_writereg(BiquadRegStart+7, LSB(coeffs[3]));
  tlv_writereg(BiquadRegStart+8, MSB(coeffs[4]));	// D2
  tlv_writereg(BiquadRegStart+9, LSB(coeffs[4]));
}



char *cfg_read_c6(char *config_buffer) {
  int i;
  *config_buffer++ = 0xC6;			// identifier byte
  *config_buffer++ = CONFIG_C6_SIZE;		// length

  for (i=0; i<3; i++) {
    *config_buffer++ = LSB(CONFIG_C6.FirstOrderIIR[i]);
    *config_buffer++ = MSB(CONFIG_C6.FirstOrderIIR[i]);
  }
  for (i=0; i<5; i++) {
    *config_buffer++ = LSB(CONFIG_C6.BQBlockA[i]);
    *config_buffer++ = MSB(CONFIG_C6.BQBlockA[i]);
  }
  for (i=0; i<5; i++) {
    *config_buffer++ = LSB(CONFIG_C6.BQBlockB[i]);
    *config_buffer++ = MSB(CONFIG_C6.BQBlockB[i]);
  }
  for (i=0; i<5; i++) {
    *config_buffer++ = LSB(CONFIG_C6.BQBlockC[i]);
    *config_buffer++ = MSB(CONFIG_C6.BQBlockC[i]);
  }
  for (i=0; i<5; i++) {
    *config_buffer++ = LSB(CONFIG_C6.BQBlockD[i]);
    *config_buffer++ = MSB(CONFIG_C6.BQBlockD[i]);
  }
  for (i=0; i<5; i++) {
    *config_buffer++ = LSB(CONFIG_C6.BQBlockE[i]);
    *config_buffer++ = MSB(CONFIG_C6.BQBlockE[i]);
  }
  return config_buffer;
}


void cfg_write_c6(const char *config_data) {
  int i;
  for (i=0; i<3; i++) {
    LSB(CONFIG_C6.FirstOrderIIR[i]) = *config_data++;
    MSB(CONFIG_C6.FirstOrderIIR[i]) = *config_data++;
  }
  tlvfilter_load_iir(CONFIG_C6.FirstOrderIIR);
  for (i=0; i<5; i++) {
    LSB(CONFIG_C6.BQBlockA[i]) = *config_data++;
    MSB(CONFIG_C6.BQBlockA[i]) = *config_data++;
  }
  tlvfilter_load_bqfir(0, CONFIG_C6.BQBlockA);
  for (i=0; i<5; i++) {
    LSB(CONFIG_C6.BQBlockB[i]) = *config_data++;
    MSB(CONFIG_C6.BQBlockB[i]) = *config_data++;
  }
  tlvfilter_load_bqfir(1, CONFIG_C6.BQBlockB);
  for (i=0; i<5; i++) {
    LSB(CONFIG_C6.BQBlockC[i]) = *config_data++;
    MSB(CONFIG_C6.BQBlockC[i]) = *config_data++;
  }
  tlvfilter_load_bqfir(2, CONFIG_C6.BQBlockC);
  for (i=0; i<5; i++) {
    LSB(CONFIG_C6.BQBlockD[i]) = *config_data++;
    MSB(CONFIG_C6.BQBlockD[i]) = *config_data++;
  }
  tlvfilter_load_bqfir(3, CONFIG_C6.BQBlockD);
  for (i=0; i<5; i++) {
    LSB(CONFIG_C6.BQBlockE[i]) = *config_data++;
    MSB(CONFIG_C6.BQBlockE[i]) = *config_data++;
  }
  tlvfilter_load_bqfir(4, CONFIG_C6.BQBlockE);
}


//! @}
