/*
 * TLV320AIC.h
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
 */


#ifndef TLV320AIC_H_
#define TLV320AIC_H_

#define CONFIG_C4_SIZE		8			// NF-Codec Adj
#define CONFIG_C5_SIZE		12			// NF-Codec AGC and DRC
#define CONFIG_C6_SIZE		56			// Filter Coeeffs


#define PLL_DATA_PIN		(AVR32_PIN_PB02-32)	// High-Active / shared with USART-TxD
#define PLL_CLOCK_PIN		(AVR32_PIN_PB03-32)	// High-Active / shared with USART-RxD
#define PLL_STROBE_PIN		(AVR32_PIN_PB04-32)	// High-Active / shared with USART-CTRS


int	tlv_init(void);		// Initialization of NF-COdec, returns 'true' if exist

/* GPIO1 functions
 * controls general purpose pin on device (as output)
 */
void	tlv_set_gpio1(void);
void	tlv_clr_gpio1(void);


void	tlv_mute_both(void);
void	tlv_unmute_dac(void);
void	tlv_unmute_adc(void);

/* tlv_set_adcgain()
 * gain in 0.5dB steps (-12dB..20dB ^= -24..40 allowed)
 * returns applied gain
 */
void	tlv_set_adcgain(signed char gain);


// set MASTER volume (independent), range -127..48, ~ 0.5dB steps
// 127: turn on PIN (knop) control
void	tlv_set_DACvolume(signed char vol);

// set analog speaker volume (independent), range -127..36, ~ 0.5dB steps
// -128: turn off ClassD amplifier
void	tlv_set_SPKRvolume(signed char vol);

// set analog headphone volume (independent), range -127..18, ~ 0.5dB steps
// -128: turn off headphone driver
void	tlv_set_HSvolume(signed char vol);


/* configuration access
 * read / write CONFIG 4 (analog setup) and CONFIG 5 (AGC, DRC setup)
 */
char *	cfg_read_c4(char *config_buffer);
char *	cfg_read_c5(char *config_buffer);
char *	cfg_read_c6(char *config_buffer);

void	cfg_write_c4(const char *config_data);
void	cfg_write_c5(const char *config_data);
void	cfg_write_c6(const char *config_data);



// Filter functions:

void	tlvfilter_default_lowpass(void);

/* 1s order IIR: this is the last filter-block (after digital volume control)
 * active in every filter-selection
 * Parameters: coeffs: array with 3 coeffizies N0,N1,D1
 */
void	tlvfilter_load_iir(const signed short *coeffs);


/* all five biquad filter-blocks used on filter selection PRB_R5 (1)
 * the same function loads 25 FIR coefficients if filter selection PRB_R6 (2)
 * Parameters: nr = 0..4 (A..E), coeffs: array with 5 coeffizies N0,N1,N2,D1,D2
 */
void	tlvfilter_load_bqfir(int nr, const signed short *coeffs);




#endif // TLV320AIC_H_
