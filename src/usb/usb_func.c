/*
 * usb_func.c
 *
 * general managing functions to enable / disable USB (power / clock)
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
 *
 * Defines:
 * NOTIFY_USBHOST (DV-Modem only): Shows a Message on Display, if Host connects or discon.
 *
 * Report:
 * 2011-09-29	Fix a bug (wrong memcpy source @ bufferwrap) in cdc_copyblock().
 * 2012-07-14	BugFix 64Byte EP Fifo Wrap Bug
 * 2012-08-01	USB Serial Number build from Bootloader-SN in usb_init()
 */

//#define NOTIFY_USBHOST

#include "usb_func.h"

#include "compiler.h"

#include "usb_drv.h"
#include "usb_task.h"
#if USB_DEVICE_FEATURE == ENABLED
#include "usb_device_task.h"
#endif
#if USB_HOST_FEATURE == ENABLED
#include "usb_host_task.h"
#endif

#include "../hardware/hw_defs.h"
#include "../hardware/gpio_func.h"

#ifdef NOTIFY_USBHOST
#include "../hardware/bf_func.h"
#include "../common/bf_cmd.h"
#endif

#include "usb_descriptors.h"

#include <string.h>


extern Bool usb_connected;

char	*cdc_txbuffer;
U32	cdc_txbuffer_len;

int	cdc_lastbytes_rx;

U32	cdc_sof_counter;		// jede ms erhöht (sof)
U32	cdc_rxidle_pos;
int	cdc_timeoutval;
Bool	cdc_timeout_enabled;

volatile U8 cdc_rxbuffer[CDC_RXBUFFERSIZE];

U32	cdc_rdpos, cdc_wrpos;

#define cdc_rxlen	((cdc_wrpos-cdc_rdpos)%CDC_RXBUFFERSIZE)
#define cdc_rxleft	(CDC_RXBUFFERSIZE-cdc_rxlen-1)


static void usb_disabled_func(void) { }


tusbfunction usb_handler = usb_disabled_func;

tusb_to_func cdc_timeout_fct;	// TimeOut Routine, called by cdc_received()


#ifdef NOTIFY_USBHOST
// Connect-Messages:
static const char USBConMsg[16] =	"USB-Hostconneced";
static const char USBDisconMsg[16] =	"USB-Hostdiscon. ";
#endif


static void usb_wait_for_pll(void) {
  if (Is_usb_vbus_high()) {
    if (init_usb_hardware()) {	// First start PLL, then wait for lock
      Usb_unfreeze_clock();
      usb_task_init();
#if USB_DEVICE_FEATURE == ENABLED && USB_HOST_FEATURE == ENABLED
       usb_handler = usb_task;
#elif USB_DEVICE_FEATURE == ENABLED
       usb_handler = usb_device_task;
#elif USB_HOST_FEATURE == ENABLED
       usb_handler = usb_host_task;
#endif
#ifdef NOTIFY_USBHOST
      bf_send2C5(DVsendMessage, 16, USBConMsg);
#endif
    } // fi init...
  } else { // fi VBUS Pin high
    exit_usb_hardware();
  }
}



static void usb_wait_for_connect(void) {
  if (Is_usb_vbus_high()) {
    Usb_freeze_clock();
    init_usb_hardware();
    usb_handler = usb_wait_for_pll;
    // CDC Transmitbuffer Init:
    cdc_txbuffer_len = 0;
    cdc_txbuffer = NULL;
  } // fi VBUS Pin high
}


static void usb_shutdown_func(void) {
  AVR32_USBB.usbcon = AVR32_USBB_UIMOD_MASK | AVR32_USBB_OTGPADE_MASK; // Disable USB-Ints
  exit_usb_hardware();
  usb_handler = usb_wait_for_connect;
  rptr_reset_inferface();
}


__inline void usb_sof_action(void) {
  cdc_sof_counter++;
}


__inline void usb_vbus_down(void) {
  Usb_freeze_clock();
  cdc_timeout_enabled = FALSE;
  usb_handler = usb_shutdown_func;
}

// Empfangen von USB-Daten auf dem RX_EP (leert USB-Doppel-Fifo in eigenen FIFO)
void cdc_receiving(void);	// forward

__inline void usb_setconfig_fct(void) {
  cdc_sof_counter = 0;
  usb_handler = cdc_receiving;
  Usb_enable_sof_interrupt();
}


void usb_init(void) {
  int i;
  U8 *sn = (U8 *)SERIALNUMBER_ADDRESS;
  // fill out SN from Bootloader
  usb_user_serial_number.bLength = sizeof(usb_user_serial_number);
  usb_user_serial_number.bDescriptorType = STRING_DESCRIPTOR;
  for (i=0; i<USB_SN_LENGTH; i+=2) {
    char z = *sn >> 4;
    if (z < 9) z += '0'; else z += 'A'-10;
    usb_user_serial_number.wstring[i] = Usb_unicode(z);
    z = *sn & 0x0F;
    if (z < 9) z += '0'; else z += 'A'-10;
    usb_user_serial_number.wstring[i+1] = Usb_unicode(z);
    sn++;
  } // rof
  AVR32_USBB.usbcon = AVR32_USBB_UIMOD_MASK | AVR32_USBB_OTGPADE_MASK;
  cdc_timeout_fct = NULL;
  usb_handler = usb_wait_for_connect;
}


void usb_exit(void) {
  if ((usb_handler != usb_wait_for_connect)&&(usb_handler != usb_disabled_func)) {
    Usb_freeze_clock();
    usb_shutdown_func();
  }
  Usb_disable_otg_pad();
//  Usb_disable();
  usb_handler = usb_disabled_func;
  cdc_timeout_fct  = NULL;
}



void cdc_transmit(const char *data, int len) {
  if (usb_connected) {
    if (Is_usb_write_enabled(TX_EP)) {
      Usb_reset_endpoint_fifo_access(TX_EP);
      cdc_txbuffer_len = usb_write_ep_txpacket(TX_EP, data, len, (const void **)&cdc_txbuffer);
      Usb_ack_in_ready_send(TX_EP);
    } // fi TX_EP empty
  }
}


void cdc_receiving(void) {		// return: Anzahl empfangener Bytes im Fifo
  int bytes_rx, bytes_left;
#if USB_DEVICE_FEATURE == ENABLED && USB_HOST_FEATURE == ENABLED
   usb_task();
#elif USB_DEVICE_FEATURE == ENABLED
   usb_device_task();
#elif USB_HOST_FEATURE == ENABLED
   usb_host_task();
#endif
  if ( usb_connected && Is_usb_out_received(RX_EP) ) {
    bytes_rx = Usb_byte_count(RX_EP);
    if ((bytes_rx > 0)&&(bytes_rx < cdc_rxleft)) {
      bytes_left = CDC_RXBUFFERSIZE-cdc_wrpos;
      Usb_reset_endpoint_fifo_access(RX_EP);
      if (bytes_rx <= bytes_left) {	// Normalfall: kein Wrap
        usb_read_ep_rxpacket(RX_EP, (void *)cdc_rxbuffer+cdc_wrpos, bytes_rx, NULL);
      } else {
        usb_read_ep_rxpacket(RX_EP, (void *)cdc_rxbuffer+cdc_wrpos, bytes_left, NULL);
        usb_read_ep_rxpacket(RX_EP, (void *)cdc_rxbuffer, bytes_rx-bytes_left, NULL);
      }
      cdc_wrpos = (cdc_wrpos+bytes_rx)%CDC_RXBUFFERSIZE;
      // End copy in own fifo
      cdc_rxidle_pos = cdc_sof_counter + cdc_timeoutval;
      cdc_timeout_enabled = (cdc_timeoutval>0)&&(cdc_timeout_fct!=NULL);
    } // fi copy received
    Usb_ack_out_received_free(RX_EP);	// moved outside of if-con
  } else if (cdc_timeout_enabled) {
    if ((int)(cdc_sof_counter - cdc_rxidle_pos) >= 0) {
      cdc_timeout_fct(cdc_rxlen);
      cdc_timeout_enabled = FALSE;
    }
  } // esle fi to
  // chain-tx (>64 byte):
  if (cdc_txbuffer_len > 0) {
    cdc_transmit(cdc_txbuffer, cdc_txbuffer_len);
  } // fi
}


int cdc_received(void) {
  return(cdc_rxlen);
}


void cdc_flushrx(void) {		// löscht den Inhalt des Rx
  if (usb_connected) {
    Usb_reset_endpoint_fifo_access(RX_EP);
    Usb_ack_out_received_free(RX_EP);
  }
  cdc_rdpos = cdc_wrpos;
//  cdc_rxfifo_empty = TRUE;
}


int cdc_copyblock(char *destbuffer, int len) {
  int bytes_left;
  if (len <= CDC_RXBUFFERSIZE) {
    bytes_left = CDC_RXBUFFERSIZE-cdc_rdpos;
    if (len <= bytes_left) {	// Normalfall: kein Wrap
      memcpy(destbuffer, (const char *)&cdc_rxbuffer[cdc_rdpos], len);
    } else {
      memcpy(destbuffer, (const char *)&cdc_rxbuffer[cdc_rdpos], bytes_left);
      memcpy(destbuffer+bytes_left, (const char *)cdc_rxbuffer, len-bytes_left);
    }
    cdc_rdpos = (cdc_rdpos+len)%CDC_RXBUFFERSIZE;
    return len;
  } else return 0;
}


unsigned char cdc_look_byte(int pos) {
  return cdc_rxbuffer[(cdc_rdpos+pos) % CDC_RXBUFFERSIZE];
}


unsigned short cdc_look_leword(int pos) {
  U16 result;
  result = ( cdc_rxbuffer[(cdc_rdpos+pos+1) % CDC_RXBUFFERSIZE] << 8) | \
      cdc_rxbuffer[(cdc_rdpos+pos) % CDC_RXBUFFERSIZE];
  return result;
}


void cdc_enabletimeout(tusb_to_func Function, int ms) {
  cdc_timeout_enabled = FALSE;
  cdc_timeoutval  = ms;
  cdc_timeout_fct = Function;
}


__inline int cdc_isactive(void) {
  return (usb_handler == cdc_receiving);
}
