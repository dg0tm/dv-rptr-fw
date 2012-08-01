/*
 * usb_descriptors.c
 *
 * This file contains the USB parameters that uniquely identify the USB
 * application through descriptor tables.
 * This file is created from CDC-Example (ATMEL AVR32-UC3-SoftwareFramework)
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

//_____ I N C L U D E S ____________________________________________________

#include "usb_descriptors.h"
#include "conf_usb.h"


#if USB_DEVICE_FEATURE == ENABLED

#include "usb_drv.h"
#include "usb_task.h"
#include "usb_standard_request.h"

//_____ M A C R O S ________________________________________________________


#define LANGUAGE_ID           0x0409



//_____ D E F I N I T I O N S ______________________________________________

// usb_user_device_descriptor
const S_usb_device_descriptor usb_dev_desc = {
  sizeof(S_usb_device_descriptor),
  DEVICE_DESCRIPTOR,
  Usb_format_mcu_to_usb_data(16, USB_SPECIFICATION),
  DEVICE_CLASS,
  DEVICE_SUB_CLASS,
  DEVICE_PROTOCOL,
  EP_CONTROL_LENGTH,
  Usb_format_mcu_to_usb_data(16, VENDOR_ID),
  Usb_format_mcu_to_usb_data(16, PRODUCT_ID),
  Usb_format_mcu_to_usb_data(16, RELEASE_NUMBER),
  MAN_INDEX,
  PROD_INDEX,
  SN_INDEX,
  NB_CONFIGURATION
};


// usb_user_configuration_descriptor FS
const S_usb_user_configuration_descriptor usb_conf_desc_fs = {
  { sizeof(S_usb_configuration_descriptor),
  CONFIGURATION_DESCRIPTOR,
  Usb_format_mcu_to_usb_data(16, sizeof(S_usb_user_configuration_descriptor)),
  NB_INTERFACE,
  CONF_NB,
  CONF_INDEX,
  CONF_ATTRIBUTES,
  MAX_POWER }, {
  sizeof(S_usb_interface_descriptor),	// Interface 0: CDC-COMM-CLASS
  INTERFACE_DESCRIPTOR,
  INTERFACE_NB_0,
  ALTERNATE_0,
  NB_ENDPOINT_0,
  INTERFACE_CLASS_0,
  INTERFACE_SUB_CLASS_0,
  INTERFACE_PROTOCOL_0,
  INTERFACE_INDEX_0 },  {

     0x05,       // Size of structure
                 // -----------------
     0x24,       // CS_INTERFACE
     0x00,       // Header Functional Descriptor
     0x10, 0x01, // USB Class Definitions for Communication Devices Specification release number in
                 // binary-coded decimal.

     0x05,       // Size of structure
                 // -----------------
     0x24,       // CS_INTERFACE
     0x01,       // Call Management Functional Descriptor
     0x03,       // The capabilities that this configuration supports:
                 // - Device handles call management itself.
                 // - Device can send/receive call management information over a Data Class interface.
     0x01,       // Interface number of Data Class interface optionally used for call management.

     0x04,       // Size of structure
                 // -----------------
     0x24,       // CS_INTERFACE
     0x02,       // Abstract Control Management Functional Descriptor.
     0x06,       // Abstract Control Management functional descriptor subtype:
                 // - Union Functional descriptor

     0x05,       // Size of structure
                 // -----------------
     0x24,       // CS_INTERFACE
     0x06,       // Union Functional descriptor
     0x00,       // The interface number of the Communication or Data Class interface, designated as
                 // the master or controlling interface for the union.
     0x01        // Interface number of first slave or associated interface in the union.
  }, {
  sizeof(S_usb_endpoint_descriptor),	// Description of INT_EP
  ENDPOINT_DESCRIPTOR,
  ENDPOINT_NB_3,
  EP_ATTRIBUTES_3,
  Usb_format_mcu_to_usb_data(16, EP_SIZE_3),
  EP_INTERVAL_3},
  // END Interface 0 description
  {
  sizeof(S_usb_interface_descriptor),	// Interface 1: CDC-DATA-CLASS
  INTERFACE_DESCRIPTOR,
  INTERFACE_NB_1,
  ALTERNATE_1,
  NB_ENDPOINT_1,
  INTERFACE_CLASS_1,
  INTERFACE_SUB_CLASS_1,
  INTERFACE_PROTOCOL_1,
  INTERFACE_INDEX_1 }, {
  sizeof(S_usb_endpoint_descriptor),	// Description of TX_EP
  ENDPOINT_DESCRIPTOR,
  ENDPOINT_NB_1,
  EP_ATTRIBUTES_1,
  Usb_format_mcu_to_usb_data(16, EP_SIZE_1),
  EP_INTERVAL_1}, {
  sizeof(S_usb_endpoint_descriptor),	// Description of RX_EP
  ENDPOINT_DESCRIPTOR,
  ENDPOINT_NB_2,
  EP_ATTRIBUTES_2,
  Usb_format_mcu_to_usb_data(16, EP_SIZE_2),
  EP_INTERVAL_2 }
};



// usb_user_language_id
const S_usb_language_id usb_user_language_id = {
  sizeof(S_usb_language_id),
  STRING_DESCRIPTOR,
  Usb_format_mcu_to_usb_data(16, LANGUAGE_ID)
};


// usb_user_manufacturer_string_descriptor
const S_usb_manufacturer_string_descriptor usb_user_manufacturer_string_descriptor = {
  sizeof(S_usb_manufacturer_string_descriptor),
  STRING_DESCRIPTOR,
  USB_MANUFACTURER_NAME
};


// usb_user_product_string_descriptor
const S_usb_product_string_descriptor usb_user_product_string_descriptor = {
  sizeof(S_usb_product_string_descriptor),
  STRING_DESCRIPTOR,
  USB_PRODUCT_NAME
};


// usb_user_serial_number, filled on request
S_usb_serial_number usb_user_serial_number;


// Inferface0-Bezeichung
const S_usb_interface_str_descr usb_user_interface0_str_descr = {
  sizeof(S_usb_interface_str_descr),
  STRING_DESCRIPTOR,
  USB_INTERFACE0_NAME
};

// Interface1-Bezeichung
const S_usb_interface_str_descr usb_user_interface1_str_descr = {
  sizeof(S_usb_interface_str_descr),
  STRING_DESCRIPTOR,
  USB_INTERFACE1_NAME
};


#endif  // USB_DEVICE_FEATURE == ENABLED
