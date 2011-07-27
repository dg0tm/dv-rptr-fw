/*
 * common_defs.h
 *
 *  Created on: 07.05.2010
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

#ifndef COMMON_DEFS_H_
#define COMMON_DEFS_H_


typedef enum {
  DVdisabled=0, DVreception,
  DVtxpreamble, DVtxstartdata, DVtxheader, DVtxvoice, DVtxvoicesync, DVtxdata, DVtxdatasync,
  DVtxlastframe, DVtxstop, DVtxemergency
} tDV_State;


typedef enum {
  DVsuccess, DVerror, DVbusy
} tDV_returncode;


typedef enum {
  DVnorx, DVsilence, DVrxheader, DVrxvoice
} tDV_RXstate;


// Receiver-State
typedef enum {
  RXunlocked, RXfastsync, RXunknowndata, RXheader, RXvoice, RXoff
} trxmode;


#endif // COMMON_DEFS_H_
