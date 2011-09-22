/*
 * dac_func.h
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
 * 2011-09-22 JA  Added power-down feature
 */


#ifndef DAC_FUNC_H_
#define DAC_FUNC_H_

#define DAC_CLOCK	2048000L
#define SPI_CS		0

void	dac_init(void);
void	dac_exit(void);

void	dac_waitidle(void);


// Ausgabe eines Spannungswertes auf den aktiven Kanal
// Der Wert 'value' wird als 16bit vorzeichenbehaftet angenommen, größere Werte
// führen zur Sättigung. Je nach DAC-Auflösung werden die LSBs verworfen.
void	dac_modulate(int value);


// dac_set_active_ch()
// Beim Dual-DAC des DV-ATRX bestimmt die Funktion, auf welchem der beiden Kanäle die
// Daten von "dac_modulate()" landen.
#if DVATRX
#define	DAC_CHANNEL_INT_XTAL	1
#define	DAC_CHANNEL_INT_PLL	0
#define	DAC_CHANNEL_EXT_FSK	0
  // für Sonderanwendungen wird der MOD_AFSK über einen Kondensator mit
  // AFOUT (intere Modulation XTAL) verbunden.
#define	DAC_CHANNEL_EXT_AFSK	1
// Im buffered mode sollte die Referenz nicht kleiner als 1V werden.
// Wirt nun extern (FSK-Pin) < 1V benötigt, so wird der Hub angepasst.
#define DAC_BUFREF_MINVOLTAGE	52	// ^= 1V 256^=4.90V

#endif	// DV-ATRX


void	dac_set_active_ch(char no);	// Auswahl des Kanals bei dem DualDAC für dac_modulate()

char	dac_get_active_ch(void);

void	dac_power_ctrl(char powermode);	// if "0" the current channel switched to pwr down

// Low-Level:

// dac_transmit() dient zur Ausgabe eines beliebigens Datenwortes an den DAC.
// Die Funktion ist vom DAC-Typ abhänig und dient zum steuern des DACs.
void	dac_transmit(unsigned short int data);


#endif // DAC_FUNC_H_
