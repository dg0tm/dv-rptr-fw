/*
 * slowdata.h
 *
 * Interface-Function to handle various slow-data formats
 *
 *  Created on: 29.05.2010
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
 */

#ifndef SLOWDATA_H_
#define SLOWDATA_H_

#define ICOM_CTEXT_SIZE		20	// Length of the "Free-Form-Text"
#define MESSAGE_RESEND_INTERVAL	12	// mal 420ms (Syncs) ^= ca. 5 Sekunden


// *** Interface to Receive and Transmit handler functions ****

// write a SYNC pattern into databuffer
void	slowdata_get_sync(unsigned char *databuffer);

// returns 3 scrambled bytes (into databuffer) from slowdata transmit logic
void	slowdata_get_txdata(unsigned int framecnt, unsigned char *databuffer);


// *** Interface to program logic / user interface ***

void	Set_SlowData_TXMsg(const char *msg);	// set own C-Text (20chars) for transmit
char	*Get_SlowData_TXMsg(void);

// Set a DSQ value (00 disables DSQ, 1..99 enables it)
void	Set_DigitalCodeSquelch(unsigned char number);


#endif // SLOWDATA_H_
