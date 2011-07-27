/*
 * usb_specific_request.c
 *
 * Processing of USB device specific enumeration requests.
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

#include "conf_usb.h"

#if USB_DEVICE_FEATURE == ENABLED

#include "usb_specific_request.h"

#include "usb_drv.h"
#include "usb_task.h"
#include "usb_descriptors.h"



// ____ T Y P E  D E F I N I T I O N _______________________________________

typedef struct {
   U32 dwDTERate;
   U8 bCharFormat;
   U8 bParityType;
   U8 bDataBits;
} S_line_coding;



//_____ P R I V A T E   D E C L A R A T I O N S ____________________________

S_line_coding		line_coding;

extern const void	*pbuffer;
extern U8		data_to_transfer;



//_____ D E C L A R A T I O N S ____________________________________________

//! @brief This function manages reception of line coding parameters (baudrate...).
//!
//void	cdc_get_line_coding(void);

//! @brief This function manages reception of line coding parameters (baudrate...).
//!
//void	cdc_set_line_coding(void);

//! @brief This function manages the SET_CONTROL_LINE_LINE_STATE CDC request.
//!
//! @todo Manages here hardware flow control...
//!
//void	cdc_set_control_line_state (void);





void cdc_get_line_coding(void) {
  Usb_ack_setup_received_free();

  Usb_reset_endpoint_fifo_access(EP_CONTROL);
  Usb_write_endpoint_data(EP_CONTROL, 8, LSB0(line_coding.dwDTERate));
  Usb_write_endpoint_data(EP_CONTROL, 8, LSB1(line_coding.dwDTERate));
  Usb_write_endpoint_data(EP_CONTROL, 8, LSB2(line_coding.dwDTERate));
  Usb_write_endpoint_data(EP_CONTROL, 8, LSB3(line_coding.dwDTERate));
  Usb_write_endpoint_data(EP_CONTROL, 8, line_coding.bCharFormat);
  Usb_write_endpoint_data(EP_CONTROL, 8, line_coding.bParityType);
  Usb_write_endpoint_data(EP_CONTROL, 8, line_coding.bDataBits  );

  Usb_ack_control_in_ready_send();
  while (!Is_usb_control_in_ready());

  while(!Is_usb_control_out_received());
  Usb_ack_control_out_received_free();
}



void cdc_set_line_coding (void) {
  Usb_ack_setup_received_free();

  while(!Is_usb_control_out_received());
  Usb_reset_endpoint_fifo_access(EP_CONTROL);

  LSB0(line_coding.dwDTERate) = Usb_read_endpoint_data(EP_CONTROL, 8);
  LSB1(line_coding.dwDTERate) = Usb_read_endpoint_data(EP_CONTROL, 8);
  LSB2(line_coding.dwDTERate) = Usb_read_endpoint_data(EP_CONTROL, 8);
  LSB3(line_coding.dwDTERate) = Usb_read_endpoint_data(EP_CONTROL, 8);
  line_coding.bCharFormat = Usb_read_endpoint_data(EP_CONTROL, 8);
  line_coding.bParityType = Usb_read_endpoint_data(EP_CONTROL, 8);
  line_coding.bDataBits   = Usb_read_endpoint_data(EP_CONTROL, 8);
  Usb_ack_control_out_received_free();

  Usb_ack_control_in_ready_send();
  while (!Is_usb_control_in_ready());

   // Set the baudrate of the USART
   {
     /*
      static usart_options_t dbg_usart_options;
      U32 stopbits, parity;

      if     ( line_coding.bCharFormat==0 )   stopbits = USART_1_STOPBIT;
      else if( line_coding.bCharFormat==1 )   stopbits = USART_1_5_STOPBITS;
      else                                    stopbits = USART_2_STOPBITS;

      if     ( line_coding.bParityType==0 )   parity = USART_NO_PARITY;
      else if( line_coding.bParityType==1 )   parity = USART_ODD_PARITY;
      else if( line_coding.bParityType==2 )   parity = USART_EVEN_PARITY;
      else if( line_coding.bParityType==3 )   parity = USART_MARK_PARITY;
      else                                    parity = USART_SPACE_PARITY;

      // Options for debug USART.
      dbg_usart_options.baudrate    = line_coding.dwDTERate;
      dbg_usart_options.charlength  = line_coding.bDataBits;
      dbg_usart_options.paritytype  = parity;
      dbg_usart_options.stopbits    = stopbits;
      dbg_usart_options.channelmode = USART_NORMAL_CHMODE;

      // Initialize it in RS232 mode.
      usart_init_rs232(DBG_USART, &dbg_usart_options, pcl_freq_param.pba_f);
      */
   }
}


void cdc_set_control_line_state (void) {
  Usb_ack_setup_received_free();
  Usb_ack_control_in_ready_send();
  while (!Is_usb_control_in_ready());
}



unsigned char select_descriptor_string(unsigned char stringid, const void **dest) {
  switch (stringid) {
  case LANG_ID:
    *dest = &usb_user_language_id;
    return sizeof(usb_user_language_id);
  case MAN_INDEX:
    *dest = &usb_user_manufacturer_string_descriptor;
    return sizeof(usb_user_manufacturer_string_descriptor);
  case PROD_INDEX:
    *dest = &usb_user_product_string_descriptor;
    return sizeof(usb_user_product_string_descriptor);
  case SN_INDEX:
    *dest = &usb_user_serial_number;
    return sizeof(usb_user_serial_number);
  case INTERFACE_INDEX_0:
    *dest = &usb_user_interface0_str_descr;
    return sizeof(usb_user_interface0_str_descr);
  case INTERFACE_INDEX_1:
    *dest = &usb_user_interface1_str_descr;
    return sizeof(usb_user_interface1_str_descr);
  default:
    return 0;
  } // hctiws STRING
}







//! This function returns the size and the pointer on a user information
//! structure
//!
Bool usb_user_get_descriptor(U8 type, U8 string) {
  pbuffer = NULL;
  switch (type) {
  case STRING_DESCRIPTOR:
    data_to_transfer = select_descriptor_string(string, &pbuffer);
    break;
/*  case DFU_FUNCT_DESCR:		// added by DO1FJN
    data_to_transfer = sizeof(usb_conf_desc.dfu);
    pbuffer = &usb_conf_desc.dfu;
    break;*/
  default:
    break;
  }
  return pbuffer != NULL;
}



//! This function is called by the standard USB read request function when
//! the USB request is not supported. This function returns TRUE when the
//! request is processed. This function returns FALSE if the request is not
//! supported. In this case, a STALL handshake will be automatically
//! sent by the standard USB read request function.
//!
Bool usb_user_read_request(U8 type, U8 request) {
  switch (request) {
    case GET_LINE_CODING:
      cdc_get_line_coding();
      return TRUE;
    case SET_LINE_CODING:
      cdc_set_line_coding();
      return TRUE;
    case SET_CONTROL_LINE_STATE:
      cdc_set_control_line_state();
      return TRUE;
    default:
      return FALSE;
  } // hctiws
}


//! @brief This function configures the endpoints of the device application.
//! This function is called when the set configuration request has been received.
//!
void usb_user_endpoint_init(U8 conf_nb) {
  (void)Usb_configure_endpoint(INT_EP, EP_ATTRIBUTES_3, DIRECTION_IN, EP_SIZE_3, SINGLE_BANK);
  (void)Usb_configure_endpoint(TX_EP,  EP_ATTRIBUTES_1, DIRECTION_IN, EP_SIZE_1, DOUBLE_BANK);
  (void)Usb_configure_endpoint(RX_EP,  EP_ATTRIBUTES_2, DIRECTION_OUT, EP_SIZE_2, DOUBLE_BANK);
}



#endif  // USB_DEVICE_FEATURE == ENABLED
