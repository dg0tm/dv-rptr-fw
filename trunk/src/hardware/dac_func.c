/*
 * dac_func.c
 *
 * Basic functions of the AD5620 connected via SPI interface.
 *
 *  Created on: 11.03.2009
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


#include "dac_func.h"
#include "hw_defs.h"
#include "gpio_func.h"

#include <compiler.h>

#define DAC_MIDDLE_LEVEL	0x07FFF		// Value for 1/2 Vref

#ifdef DVATRX	// DAC is Dual-DAC AD5312
#define DAC_PWRDOWN_VAL		0x6000		// PD = 10 100K->GND
int	dac_select_mask = 0x0000;
#else		// DAC is a AD5620
#define DAC_PWRDOWN_VAL		0x8000		// Powerdown 100k-GND
#endif

#define spi_csr		((0<<AVR32_SPI_DLYBCT_OFFSET)|(1<<AVR32_SPI_DLYBS_OFFSET)|	\
    (((MASTERCLOCK+DAC_CLOCK/2)/DAC_CLOCK)<<AVR32_SPI_SCBR_OFFSET)|			\
    (8<<AVR32_SPI_BITS_OFFSET)|AVR32_SPI_CPOL_MASK|AVR32_SPI_NCPHA_MASK)



void dac_init(void) {
  // ToDo: pm_enable-SPI()...
  AVR32_SPI.cr = AVR32_SPI_CR_SWRST_MASK;	// Reset
  // Master Mode, no Peripheral Selected
  AVR32_SPI.mr = (1<<AVR32_SPI_MR_DLYBCS_OFFSET)|(0x0f<<AVR32_SPI_MR_PCS_OFFSET)|AVR32_SPI_MSTR_MASK;
#if (SPI_CS==0)
  AVR32_SPI.csr0 = spi_csr;
#elif (SPI_CS==1)
  AVR32_SPI.csr1 = spi_csr;
#elif (SPI_CS==2)
  AVR32_SPI.csr2 = spi_csr;
#elif (SPI_CS==3)
  AVR32_SPI.csr3 = spi_csr;
#endif
  AVR32_SPI.mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + SPI_CS));
  AVR32_SPI.cr = AVR32_SPI_CR_SPIEN_MASK;

#ifdef DVATRX
  gpio0_clr(DACLD_PIN);
#endif
}


// dac_exit() schaltet die SPI (DAC) Schnittstelle kontrolliert ab.
void dac_exit(void) {
  if (AVR32_SPI.sr&AVR32_SPI_SR_SPIENS_MASK) {
    dac_waitidle();
#ifdef DVATRX
    AVR32_SPI.tdr = dac_select_mask | DAC_PWRDOWN_VAL | DAC_MIDDLE_LEVEL;
#else
    AVR32_SPI.tdr = DAC_PWRDOWN_VAL | DAC_MIDDLE_LEVEL;
#endif
    dac_waitidle();
  } // fi SPI enabled
#ifdef DVATRX
  gpio0_set(DACLD_PIN);
#endif
  AVR32_SPI.cr = AVR32_SPI_CR_SPIDIS_MASK;
  // ToDo:
  // PM-Disable-SPI()... to save some µA power
}


__inline void dac_transmit(unsigned short int data) {
  AVR32_SPI.tdr = data;
}


__inline void dac_modulate(int value) {
#ifdef DVATRX
  AVR32_SPI.tdr = dac_select_mask | (__builtin_sats(value, 4, 12)+0x0800);
#else
  AVR32_SPI.tdr = __builtin_sats(value, 2, 14)+0x2000;
#endif
}


void dac_waitidle(void) {
  U32 spi_sr, death_cnt;
  for (death_cnt=0; death_cnt<1000; death_cnt++) {
    spi_sr = AVR32_SPI.sr;
    if ((spi_sr&AVR32_SPI_SR_SPIENS_MASK) && ((spi_sr&AVR32_SPI_SR_TXEMPTY_MASK)==0) ) {
// NOP no operation needed
    } else return;
  } // rof
}


#if DVATRX
void dac_set_active_ch(char no) {
  // Powerdown opposite DAC-Channel:
  AVR32_SPI.tdr = (no)?0x4000:0xC000 | DAC_PWRDOWN_VAL | DAC_MIDDLE_LEVEL;
  // Select new Channel (with BUF enabled)
  dac_select_mask = (no)?0xC000:0x4000;
  dac_waitidle();
}


__inline char dac_get_active_ch(void) {
  return (dac_select_mask&0x8000)?1:0;
}

#endif

