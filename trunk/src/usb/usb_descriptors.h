/*
 * usb_descriptors.h
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


#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "compiler.h"
#include "defines.h"



#define Usb_unicode(c)                    (Usb_format_mcu_to_usb_data(16, (U16)(c)))
#define Usb_get_dev_desc_pointer()        (&(usb_dev_desc.bLength))
#define Usb_get_dev_desc_length()         (sizeof(usb_dev_desc))
#define Usb_get_conf_desc_pointer()       (&(usb_conf_desc_fs.cfg.bLength))
#define Usb_get_conf_desc_length()        (sizeof(usb_conf_desc_fs))
#define Usb_get_conf_desc_hs_pointer()    (&(usb_conf_desc_hs.cfg.bLength))
#define Usb_get_conf_desc_hs_length()     (sizeof(usb_conf_desc_hs))
#define Usb_get_conf_desc_fs_pointer()    (&(usb_conf_desc_fs.cfg.bLength))
#define Usb_get_conf_desc_fs_length()     (sizeof(usb_conf_desc_fs))
#define Usb_get_qualifier_desc_pointer()  (&(usb_qualifier_desc.bLength))
#define Usb_get_qualifier_desc_length()   (sizeof(usb_qualifier_desc))


//_____ U S B    D E F I N E S _____________________________________________

            // USB Device descriptor
#define USB_SPECIFICATION		0x0200
#define DEVICE_CLASS			CDC_COMM_DEVICE_CLASS
#define DEVICE_SUB_CLASS		0	//! Each configuration has its own subclass
#define DEVICE_PROTOCOL			0	//! Each configuration has its own protocol
#define EP_CONTROL_LENGTH		64
#define VENDOR_ID			ATMEL_VID
#define PRODUCT_ID			CDC_EXAMPLE_PID
#define RELEASE_NUMBER			0x1000
#define MAN_INDEX			0x01	//#define MAN_INDEX             0x00
#define PROD_INDEX			0x02	//#define PROD_INDEX            0x00
#define SN_INDEX			0x03	//#define SN_INDEX              0x00

#define NB_CONFIGURATION		1

            // CONFIGURATION
#define NB_INTERFACE			2     //! The number of interfaces for this configuration
#define CONF_NB				1     //! Number of this configuration
#define CONF_INDEX			0
#define CONF_ATTRIBUTES			USB_CONFIG_SELFPOWERED
#define MAX_POWER			50    // 100 mA

            // Interface 0 descriptor
#define INTERFACE_NB_0			0                  //! The number of this interface
#define ALTERNATE_0			0                  //! The alt setting nb of this interface
#define NB_ENDPOINT_0			1                  //! The number of endpoints this interface has
#define INTERFACE_CLASS_0		CDC_COMM_CLASS     //! CDC ACR Com Class
#define INTERFACE_SUB_CLASS_0		0x02
#define INTERFACE_PROTOCOL_0		0x01
#define INTERFACE_INDEX_0		0x04	// alt: 0

            // Interface 1 descriptor
#define INTERFACE_NB_1			1                  //! The number of this interface
#define ALTERNATE_1			0                  //! The alt setting nb of this interface
#define NB_ENDPOINT_1			2                  //! The number of endpoints this interface has
#define INTERFACE_CLASS_1		CDC_DATA_CLASS     //! CDC ACR Data Class
#define INTERFACE_SUB_CLASS_1		0
#define INTERFACE_PROTOCOL_1		0
#define INTERFACE_INDEX_1		0x05	// alt 0:

             // USB Endpoint 1 descriptor
             // Bulk IN
#define ENDPOINT_NB_1			( TX_EP | MSK_EP_DIR )
#define EP_ATTRIBUTES_1			TYPE_BULK
#define EP_SIZE_1			0x40
#define EP_INTERVAL_1			0x00		//! Interrupt polling interval from host

             // USB Endpoint 2 descriptor
             // Bulk OUT
#define ENDPOINT_NB_2			RX_EP
#define EP_ATTRIBUTES_2			TYPE_BULK
#define EP_SIZE_2			0x40		// 64 Bytes
#define EP_INTERVAL_2			0x00		//! Interrupt polling interval from host

             // USB Endpoint 3 descriptor
             // Interrupt IN
#define ENDPOINT_NB_3			( INT_EP | MSK_EP_DIR )
#define EP_ATTRIBUTES_3			TYPE_INTERRUPT
#define EP_SIZE_3			0x20
#define EP_INTERVAL_3			0xFF		//! Interrupt polling interval from host


#define DEVICE_STATUS			SELF_POWERED
#define INTERFACE_STATUS		0x00 // TBD

#define LANG_ID				0x00


#define USB_MN_LENGTH		21
#define USB_MANUFACTURER_NAME	{ \
  Usb_unicode('d'),\
  Usb_unicode('i'),\
  Usb_unicode('g'),\
  Usb_unicode('i'),\
  Usb_unicode('s'),\
  Usb_unicode('o'),\
  Usb_unicode('l'),\
  Usb_unicode('u'),\
  Usb_unicode('t'),\
  Usb_unicode('i'),\
  Usb_unicode('o'),\
  Usb_unicode('n'),\
  Usb_unicode('s'),\
  Usb_unicode(' '),\
  Usb_unicode('/'),\
  Usb_unicode(' '),\
  Usb_unicode('A'),\
  Usb_unicode('T'),\
  Usb_unicode('M'),\
  Usb_unicode('E'),\
  Usb_unicode('L') \
}

#define USB_PN_LENGTH		11
#define USB_PRODUCT_NAME	{ \
  Usb_unicode('D'),\
  Usb_unicode('V'),\
  Usb_unicode('-'),\
  Usb_unicode('R'),\
  Usb_unicode('P'),\
  Usb_unicode('T'),\
  Usb_unicode('R'),\
  Usb_unicode(' '),\
  Usb_unicode('C'),\
  Usb_unicode('D'),\
  Usb_unicode('C') \
}


#define USB_IF_LENGTH		4
#define USB_INTERFACE0_NAME	{ \
  Usb_unicode('C'), Usb_unicode('O'), Usb_unicode('M'), Usb_unicode('M')	\
}
#define USB_INTERFACE1_NAME	{ \
  Usb_unicode('D'), Usb_unicode('A'), Usb_unicode('T'), Usb_unicode('A')	\
}



#if (FIRMWAREVERSION&0xF)
#define USB_FIRMWARE_VERSION	{ \
  Usb_unicode(((char)(FIRMWAREVERSION>>12)+'0')),\
  Usb_unicode('.'),\
  Usb_unicode(((char)((FIRMWAREVERSION>>8)&0xF)+'0')),\
  Usb_unicode(((char)((FIRMWAREVERSION>>4)&0xF)+'0')),\
  Usb_unicode(((char)(FIRMWAREVERSION&0xF)+'a'-1)) \
}
#else
#define USB_FIRMWARE_VERSION	{ \
  Usb_unicode(((char)(FIRMWAREVERSION>>12)+'0')),\
  Usb_unicode('.'),\
  Usb_unicode(((char)((FIRMWAREVERSION>>8)&0xF)+'0')),\
  Usb_unicode(((char)((FIRMWAREVERSION>>4)&0xF)+'0')),\
  Usb_unicode(' ') \
}
#endif


// 'S_Usb' Type Definitions:

//! USB Request
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bmRequestType;        //!< Characteristics of the request
  U8      bRequest;             //!< Specific request
  U16     wValue;               //!< Field that varies according to request
  U16     wIndex;               //!< Field that varies according to request
  U16     wLength;              //!< Number of bytes to transfer if Data
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_UsbRequest;


//! USB Device Descriptor
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bLength;              //!< Size of this descriptor in bytes
  U8      bDescriptorType;      //!< DEVICE descriptor type
  U16     bscUSB;               //!< Binay Coded Decimal Spec. release
  U8      bDeviceClass;         //!< Class code assigned by the USB
  U8      bDeviceSubClass;      //!< Subclass code assigned by the USB
  U8      bDeviceProtocol;      //!< Protocol code assigned by the USB
  U8      bMaxPacketSize0;      //!< Max packet size for EP0
  U16     idVendor;             //!< Vendor ID. ATMEL = 0x03EB
  U16     idProduct;            //!< Product ID assigned by the manufacturer
  U16     bcdDevice;            //!< Device release number
  U8      iManufacturer;        //!< Index of manu. string descriptor
  U8      iProduct;             //!< Index of prod. string descriptor
  U8      iSerialNumber;        //!< Index of S.N.  string descriptor
  U8      bNumConfigurations;   //!< Number of possible configurations
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_device_descriptor;


//! USB Configuration Descriptor
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bLength;              //!< Size of this descriptor in bytes
  U8      bDescriptorType;      //!< CONFIGURATION descriptor type
  U16     wTotalLength;         //!< Total length of data returned
  U8      bNumInterfaces;       //!< Number of interfaces for this conf.
  U8      bConfigurationValue;  //!< Value for SetConfiguration resquest
  U8      iConfiguration;       //!< Index of string descriptor
  U8      bmAttributes;         //!< Configuration characteristics
  U8      MaxPower;             //!< Maximum power consumption
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_configuration_descriptor;


//! USB Interface Descriptor
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bLength;              //!< Size of this descriptor in bytes
  U8      bDescriptorType;      //!< INTERFACE descriptor type
  U8      bInterfaceNumber;     //!< Number of interface
  U8      bAlternateSetting;    //!< Value to select alternate setting
  U8      bNumEndpoints;        //!< Number of EP except EP 0
  U8      bInterfaceClass;      //!< Class code assigned by the USB
  U8      bInterfaceSubClass;   //!< Subclass code assigned by the USB
  U8      bInterfaceProtocol;   //!< Protocol code assigned by the USB
  U8      iInterface;           //!< Index of string descriptor
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_interface_descriptor;


//! USB Endpoint Descriptor
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bLength;              //!< Size of this descriptor in bytes
  U8      bDescriptorType;      //!< ENDPOINT descriptor type
  U8      bEndpointAddress;     //!< Address of the endpoint
  U8      bmAttributes;         //!< Endpoint's attributes
  U16     wMaxPacketSize;       //!< Maximum packet size for this EP
  U8      bInterval;            //!< Interval for polling EP in ms
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_endpoint_descriptor;


//! USB Device Qualifier Descriptor
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bLength;              //!< Size of this descriptor in bytes
  U8      bDescriptorType;      //!< Device Qualifier descriptor type
  U16     bscUSB;               //!< Binay Coded Decimal Spec. release
  U8      bDeviceClass;         //!< Class code assigned by the USB
  U8      bDeviceSubClass;      //!< Subclass code assigned by the USB
  U8      bDeviceProtocol;      //!< Protocol code assigned by the USB
  U8      bMaxPacketSize0;      //!< Max packet size for EP0
  U8      bNumConfigurations;   //!< Number of possible configurations
  U8      bReserved;            //!< Reserved for future use, must be zero
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_device_qualifier_descriptor;


//! USB Language Descriptor
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8      bLength;              //!< Size of this descriptor in bytes
  U8      bDescriptorType;      //!< STRING descriptor type
  U16     wlangid;              //!< Language id
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_language_id;


//_____ U S B   M A N U F A C T U R E R   D E S C R I P T O R _______________

//! struct usb_st_manufacturer
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8  bLength;                  //!< Size of this descriptor in bytes
  U8  bDescriptorType;          //!< STRING descriptor type
  U16 wstring[USB_MN_LENGTH];   //!< Unicode characters
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_manufacturer_string_descriptor;


//_____ U S B   P R O D U C T   D E S C R I P T O R _________________________

//! struct usb_st_product
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8  bLength;                  //!< Size of this descriptor in bytes
  U8  bDescriptorType;          //!< STRING descriptor type
  U16 wstring[USB_PN_LENGTH];   //!< Unicode characters
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_product_string_descriptor;


//_____ U S B   S E R I A L   N U M B E R   D E S C R I P T O R _____________

#define USB_SN_LENGTH		8


//! struct usb_st_serial_number
typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  U8  bLength;                  //!< Size of this descriptor in bytes
  U8  bDescriptorType;          //!< STRING descriptor type
  U16 wstring[USB_SN_LENGTH];   //!< Unicode characters
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_serial_number;


//_____ U S B   I N T E R F A C E   D E S C R I P T O R (DO1FJN) _______________

//! struct usb_st_manufacturer
typedef
#if __ICCAVR32__
#pragma pack(1)
#endif
struct
#if __GNUC__
__attribute__((__packed__))
#endif
{
  U8  bLength;                  //!< Size of this descriptor in bytes
  U8  bDescriptorType;          //!< STRING descriptor type
  U16 wstring[USB_IF_LENGTH];   //!< Unicode characters
}
#if __ICCAVR32__
#pragma pack()
#endif
S_usb_interface_str_descr;



//_____ U S B   D E V I C E   C D C   D E S C R I P T O R ___________________

typedef
#if (defined __ICCAVR32__)
#pragma pack(1)
#endif
struct
#if (defined __GNUC__)
__attribute__((__packed__))
#endif
{
  S_usb_configuration_descriptor cfg;
  S_usb_interface_descriptor     ifc0;
  U8 CS_INTERFACE[19];
  S_usb_endpoint_descriptor      ep3;
  S_usb_interface_descriptor     ifc1;
  S_usb_endpoint_descriptor      ep1;
  S_usb_endpoint_descriptor      ep2;
}
#if (defined __ICCAVR32__)
#pragma pack()
#endif
S_usb_user_configuration_descriptor;


//_____ D E F I N I T I O N S ______________________________________________


extern const S_usb_device_descriptor              usb_dev_desc;
extern const S_usb_user_configuration_descriptor  usb_conf_desc;
extern const S_usb_device_qualifier_descriptor    usb_qualifier_desc;
extern const S_usb_user_configuration_descriptor  usb_conf_desc_fs;
extern const S_usb_user_configuration_descriptor  usb_conf_desc_hs;
extern const S_usb_manufacturer_string_descriptor usb_user_manufacturer_string_descriptor;
extern const S_usb_product_string_descriptor      usb_user_product_string_descriptor;
extern       S_usb_serial_number                  usb_user_serial_number;
extern const S_usb_language_id                    usb_user_language_id;

extern const S_usb_interface_str_descr		  usb_user_interface0_str_descr;
extern const S_usb_interface_str_descr		  usb_user_interface1_str_descr;


void	usb_sof_action(void);
//void	usb_vbus_up(void);		// Connect function
void	usb_vbus_down(void);		// Disconnect function

void	usb_setconfig_fct(void);	// Action after config

#endif  // _USB_DESCRIPTORS_H_
