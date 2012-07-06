/*
 * rda1846.h
 *
 *  Created on: 13.11.2011
 *      Author: Jan Alte, DO1FJN
 *
 *       * This file is part of the DV-modem firmware (DVfirmware).
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

#ifndef RDA1846_H_
#define RDA1846_H_

#define RDA_CLOCK	26000000L	// crystal or oscillator frequency

//#define SUPPORT_2ND_IC

// definitions for 'outside' use:
#define RDA_INVERTED_RX	true
#define RDA_DV_TXDELAY	30


unsigned int rda_init(void);	// returns capabilities flags


unsigned int rda_setfreq(unsigned int rx_freq, unsigned int tx_freq);
// returns exact recalculated rx frequency in Hz

void	rda_standby(void);
void	rda_transmit(void);
void	rda_receive(void);


#ifdef SUPPORT_2ND_IC
void	rda2nd_transmit(void);
void	rda2nd_receive(void);
#endif


#endif // RDA1846_H_
