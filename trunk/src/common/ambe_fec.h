/*
 * ambe_fec.h
 *
 *  Created on: 14.10.2011
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
 * original source created by Jonathan Naylor, G4KLX
 */

#ifndef AMBE_FEC_H_
#define AMBE_FEC_H_

typedef char tambevoicefec[9];
typedef char tambevoice[6];


void	ambefec_regenerate(tambevoicefec voice);

void	ambe_removefec(tambevoice result, const tambevoicefec voice);
void	ambe_addfec(tambevoicefec result, const tambevoice voice);


#endif // AMBE_FEC_H_
