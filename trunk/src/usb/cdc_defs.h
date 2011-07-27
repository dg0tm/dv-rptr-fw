/*
 * cdc_defs.c
 *
 * This file manages the generic CDC definitions and features.
 * This file is created from various files of the CDC-Example
 * (ATMEL AVR32-UC3-SoftwareFramework)
 *
 *  Created on: 23.07.2010
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

 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef _CDC_H_
#define _CDC_H_


/*! \name Vendor ID (VID)
 */
//! @{
#define ATMEL_VID		0x03EB
//! @}

/*! \name USB Product ID (PID)
 */
//! @{
#define CDC_EXAMPLE_PID		0x2307
//! @}


/*! \name CDC specific definitions (Class, subclass and protocol)
 */
//! @{
#define CDC_COMM_DEVICE_CLASS			0x02
#define CDC_COMM_CLASS				0x02
#define CDC_COMM_DIRECT_LINE_CM_SUBCLASS	0x01
#define CDC_COMM_ABSTRACT_CM_SUBCLASS		0x02
#define CDC_COMM_TELEPHONE_CM_SUBCLASS		0x03
#define CDC_COMM_MULTICHANNEL_CM_SUBCLASS	0x04
#define CDC_COMM_CAPI_CM_SUBCLASS		0x05
#define CDC_COMM_ETHERNET_NETWORKING_CM_SUBCLASS 0x06
#define CDC_COMM_ATM_NETWORKING_CM_SUBCLASS	0x07
#define CDC_COMM_V25ter_PROTOCOL		0x01
#define CDC_DATA_CLASS				0x0A
#define CDC_DATA_SUBCLASS			0x00
#define CDC_DATA_PHYS_ISDN_BRI_PROTOCOL		0x30
#define CDC_DATA_HDLC_PROTOCOL			0x31
#define CDC_DATA_TRANSPARENT_PROTOCOL		0x32
//! @}


/*! \name CDC Requests
 */
//! @{
#define SET_LINE_CODING               0x20
#define GET_LINE_CODING               0x21
#define SET_CONTROL_LINE_STATE        0x22
#define SEND_BREAK                    0x23
#define SEND_ENCAPSULATED_COMMAND     0x00
#define GET_ENCAPSULATED_COMMAND      0x01
//! @}


#endif  // _CDC_H_
