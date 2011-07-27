/*
 * hw_defs.h
 *
 * Definition of AVR32 peripherals, GPIO pin functions and Int-Usages.
 * This file only for HW1.0 and later. For HW0.9PT a redirection to "hw-def09.h" is
 * included.
 *
 *  Created on: 08.03.2009
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
 *  Report:
 *  2010-01-16	Ausgemistet.
 *  2010-02-14  RS232 (USART 1) hinzu
 *  2011-07-26	Version für DV-RPTR
 */

#ifndef HW_DEFS_H_
#define HW_DEFS_H_

#if __GNUC__

#define INTERRUPT_FUNC	__attribute__((__interrupt__)) static void

#elif __ICCAVR32__

#define INTERRUPT_FUNC	__interrupt static void

#endif


// PDCA Channels for AMBE:
#define AMBE_CHANNEL0		0
#define AMBE_CHANNEL1		1
// PDCA Channel for Serial BF-Bus:
#define BF_CHANNEL		2	// one Channel for RX and TX
// PDCA Channels for RS232 Interface:
#define RS232_RXCH		3
#define RS232_TXCH		4
// PDCA Channel for TWI / I²C
#define TWI_CHANNEL		5


// AMBE - Priorities
#define AMBE_EPR_PRIO		AVR32_INTC_INT1	// >= als TXRDY, damit vor TRXEND
#define AMBE_TRXSTART_PRIO	AVR32_INTC_INT3
#define AMBE_TRXEND_PRIO	AVR32_INTC_INT1

// Externe Interrupts EIC & NMI:
#define AMBE_EPR_INT		AVR32_EIC_INT3
//#define AMBE_EPR_PIN		AVR32_EIC_EXTINT_3_PIN
//#define AMBE_EPR_FUNCTION	AVR32_EIC_EXTINT_3_FUNCTION

// TWI Interface
#define TWI_INTPRIO		AVR32_INTC_INT0

// GMSK Timer + Ints
#define DVRX_TIMER_CH		0
#define DVTX_TIMER_CH		1
#define IDLE_TIMER_CH		1		// statt Transmit

#define DV_MOD_INTPRIO		AVR32_INTC_INT1
#define DV_DEMOD_INTPRIO	AVR32_INTC_INT2

#define MASTERCLOCK		60000000L

#define OSC0STARTUPVALUE	4

// use SSC TF-Workarround?
#define SSC_TFDIRECTION		1		// 1 = Use TF as Input (connected with RF)

//#define SSC_RCMR_INI		0x00000402	// RK-Input, Start falling RF
//#define SSC_RFMR_INI		0x0000008F	// RF-Input, 16Bit Len, MSB First
#define SSC_TCMR_INI		0x00000422	// Start=3 (High TF Level), with CKI, TK Pin is Src
#define SSC_TFMR_INI		0x0000008F	// 16 Bits, MSB first

#define SSC_RCMR_CODEC		0x00000402	// RK-Input, Start falling RF
#define SSC_RFMR_CODEC		0x0000008F	// RF: Input, 16Bit Len, MSB First
#define SSC_RCMR_AMBEACT	0x00000402	// RK-Input, Start falling RF
#define SSC_RFMR_AMBEACT	0x0000008F	// RF: Input, 16Bit Len, MSB First
// AMBE passive framed mode:
// use a min. gap of 25 SSC-Clock-Cylces to prevent timing errors
#define SSC_xCMR_AMBEPAS	0x13000402	// RK-Input, Start falling RF, Period 40Clks
#define SSC_xFMR_AMBEPAS	0x0020008F	// RF: Pos.Pulse, 16Bit Len, MSB First, 24 Data

// DAC Standard Values for GMSK / D-Star
#define DAC_MIDDLE		0		// Signed Offset to real Bit-Middle


#ifdef DVRPTR
  // GPIO for DV-RPTR Target
#endif

//#ifdef DVATRX ...

// GPIO Section (Defines)
// (target depended)
// Output-GPIO-Pins (PortA)
#define TRX_SEL_PIN		AVR32_PIN_PA03		// HiZ/act. LOW OUT
#define PLL_STROBE_PIN		AVR32_PIN_PA04		// High-Active (latch pll data)
#define MIC_PTT_PIN		AVR32_PIN_PA05		// IN + Pullup (EINT1)
#define MIC_SC_PIN		AVR32_PIN_PA06		// IN + Pullup (EINT0)
#define ADC_TEMPPA_PIN		AVR32_PIN_PA07		// Non-PIO
#define ADC_TEMPTRX_PIN		AVR32_PIN_PA08		// Non-PIO
#define PLL_DATA_PIN		AVR32_PIN_PA09		// High-Active / shared with TWI
#define PLL_CLOCK_PIN		AVR32_PIN_PA10		// High-Active / shared with TWI
#define WATCHDOG_PIN		AVR32_PIN_PA11		// toggle external WDI (ST706T)
#define DACLD_PIN		AVR32_PIN_PA12		// OUT low active
#define PFI_NMI_PIN		AVR32_PIN_PA13		// NMI/Input PowerFail
#define DACEN_PIN		AVR32_PIN_PA16		// OUT low active
#define CPLDRKTKDIR_PIN		AVR32_PIN_PA17
#define MIC_UP_PIN		AVR32_PIN_PA20		// IN + Pullup (alternative RXD2)
#define MIC_DN_PIN		AVR32_PIN_PA21		// IN + Pullup (alternative TXD2)
#define CPLDTFDIR_PIN		AVR32_PIN_PA22
#define RUNCFG_PIN		AVR32_PIN_PA24
#define KBINT_PIN		AVR32_PIN_PA25		// Input / EINT Key-Pressed
#define USBID_PIN		AVR32_PIN_PA26
#define USBVBOF_PIN		AVR32_PIN_PA27
#define PTT_OUT_PIN		AVR32_PIN_PA28		// OUT
#define MAIN_POWER_PIN		AVR32_PIN_PA29		// OUT


#define MIC_PTTKEY_INT		AVR32_EIC_INT0
#define MIC_SCKEY_INT		AVR32_EIC_INT1
#define KEYB_INT		AVR32_EIC_INT5


// Priorities
#define MICKEYINT_PRIO		AVR32_INTC_INT0
#define KEYBINT_PRIO		AVR32_INTC_INT0


// *** Definition of Port A ***
#define GPIO0_PULLUP		(1<<MIC_PTT_PIN)|(1<<MIC_SC_PIN)|(1<<MIC_UP_PIN)|	\
				(1<<MIC_DN_PIN)|(1<<KBINT_PIN)
// define GPIO Outputs
#define GPIO0_ODER		(1<<TRX_SEL_PIN)|(1<<PLL_STROBE_PIN)|(1<<PLL_DATA_PIN)|	\
	                        (1<<PLL_CLOCK_PIN)|(1<<WATCHDOG_PIN)|(1<<DACLD_PIN)|	\
	                        (1<<CPLDRKTKDIR_PIN)|(1<<CPLDTFDIR_PIN)|(1<<RUNCFG_PIN)| \
                                (1<<MAIN_POWER_PIN)|(1<<PTT_OUT_PIN)|(1<<USBVBOF_PIN)

#define GPIO0_OVRINIT		(1<<MAIN_POWER_PIN)|(1<<RUNCFG_PIN)|(1<<DACLD_PIN)

#define GPIO0_PMR0		0
#define GPIO0_PMR1		0
#define GPIO0_DISABLE_MASK	(1<<AVR32_TWI_SDA_0_0_PIN)|(1<<AVR32_TWI_SCL_0_0_PIN)|	\
	                        (1<<AVR32_SPI_MOSI_0_0_PIN)|(1<<AVR32_SPI_SCK_0_0_PIN)|	\
	                        (1<<AVR32_SPI_NPCS_0_0_PIN)|	\
	                        (1<<HFIN_PIN)|(1<<RSSI_PIN)


// GPIO Macros
#define dev_power_off()		(AVR32_GPIO.port[0].ovrc = 1 << MAIN_POWER_PIN)

#define enable_extdemod()	gpio0_clr(TRX_SEL_PIN)
#define disable_extdemod()	gpio0_set(TRX_SEL_PIN)
#define is_internalmod()	gpio0_readovr(TRX_SEL_PIN)

#define enable_ptt()		gpio0_set(PTT_OUT_PIN)
#define disable_ptt()		gpio0_clr(PTT_OUT_PIN)
#define is_pttactive()		gpio0_readovr(PTT_OUT_PIN)

#define pll_set_strobe()	gpio0_set(PLL_STROBE_PIN)
#define pll_clr_strobe()	gpio0_clr(PLL_STROBE_PIN)
#define pll_set_data()		gpio0_set(PLL_DATA_PIN)
#define pll_clr_data()		gpio0_clr(PLL_DATA_PIN)
#define pll_set_clock()		gpio0_set(PLL_CLOCK_PIN)
#define pll_clr_clock()		gpio0_clr(PLL_CLOCK_PIN)

// Spannungen und Widerstände / Teiler Definitionen

#define V_Ref_32	320	// Voltx100
#define V_Ref_5		490	// Voltx100

#define R_SQL_IN	27
#define	R_SQL_GND	47	// kOhms
#define V_SQL_MAX	((V_Ref_32*(R_SQL_IN+R_SQL_GND)+R_SQL_GND/2)/R_SQL_GND)

#define PreAmp_R2_Reihe		330
#define PreAmp_R2_Poti		50000
#define PreAmp_R1		2700

#define PreAmp_R2_Step		(PreAmp_R2_Poti/255)

// Spannungen in hunderstel Volt (x100) - ca. Werte
#define calc_demod_gain(x)	((x*PreAmp_R2_Step+PreAmp_R2_Reihe)*100/PreAmp_R1)

#define calc_modulation_vss(x)	((x*V_Ref_5)>>8)
#define calc_xtal_vss(x)	((x*V_Ref_5)>>8)

//#endif


// common pin defs (both devices):

#define WATCHDOG_PIN		AVR32_PIN_PA11

#define CPLDRKTKDIR_PIN		AVR32_PIN_PA17
#define CPLDTFDIR_PIN		AVR32_PIN_PA22
#define RUNCFG_PIN		AVR32_PIN_PA24

#define USBID_PIN		AVR32_PIN_PA26
#define USBVBOF_PIN		AVR32_PIN_PA27

#define HFIN_PIN		AVR32_ADC_AD_7_PIN
#define RSSI_PIN		AVR32_ADC_AD_6_PIN

// Output-GPIO-Pins (PortB)
#define CPLDCFG0_PIN		(AVR32_PIN_PB01-32)
#define CPLDCFG1_PIN		(AVR32_PIN_PB00-32)


// RS232 connected on Port B
#define RS232RXD_PIN		(AVR32_USART1_RXD_0_1_PIN-32)
#define RS232TXD_PIN		(AVR32_USART1_TXD_0_1_PIN-32)
// RS232: Function "2" RxD/TxD & "0" CTS/RTS
#define RS232CTS_PIN		(AVR32_USART1_CTS_0_0_PIN-32)
#define RS232RTS_PIN		(AVR32_USART1_RTS_0_0_PIN-32)
#define RS232OFF_PIN		AVR32_PIN_PA12


#define twi_release_pins()	(AVR32_GPIO.port[0].gpers = (1<<AVR32_TWI_SDA_0_0_PIN)|(1<<AVR32_TWI_SCL_0_0_PIN))
#define twi_connect_pins()	(AVR32_GPIO.port[0].gperc = (1<<AVR32_TWI_SDA_0_0_PIN)|(1<<AVR32_TWI_SCL_0_0_PIN))


// *** Definition of Port B ***

#define GPIO1_PULLUP		0
#define GPIO1_ODER		(1<<CPLDCFG0_PIN)|(1<<CPLDCFG1_PIN)
#define GPIO1_OVRINIT		0
#define GPIO1_PMR0		0
#define GPIO1_PMR1		(1<<RS232RXD_PIN)|(1<<RS232TXD_PIN)

#define GPIO1_DISABLE_MASK	(1<<(RS232RXD_PIN&0x1F))|(1<<(RS232TXD_PIN&0x1F))|	\
                                (1<<(RS232CTS_PIN&0x1F))|(1<<(RS232RTS_PIN&0x1F))|	\
                                (1<<(AVR32_SSC_RX_CLOCK_0_PIN&0x1F))| \
                                (1<<(AVR32_SSC_RX_DATA_0_PIN&0x1F))| \
                                (1<<(AVR32_SSC_RX_FRAME_SYNC_0_PIN&0x1F))| \
                                (1<<(AVR32_SSC_TX_CLOCK_0_PIN&0x1F))| \
                                (1<<(AVR32_SSC_TX_DATA_0_PIN&0x1F))| \
                                (1<<(AVR32_SSC_TX_FRAME_SYNC_0_PIN&0x1F))


// End GPIO Section

#define PBAMASK_DEFAULT		0x1FFF
#define PBAMASK_NEEDED		0x1B7F	// Disable Clock for USART0, PWM

#define PBBMASK_NEEDED		0x0005
#define PBBMASK_USBB		0x0002	// Bit1

// Peripheral Definitions:
#define BFIO			AVR32_USART2
#define BFIRQ			AVR32_USART2_IRQ
#define BFPIDRX			AVR32_PDCA_PID_USART2_RX
#define BFPIDTX			AVR32_PDCA_PID_USART2_TX

#define MICIO			AVR32_USART2
#define MICIRQ			AVR32_USART2_IRQ


#define RS232			AVR32_USART1
#define RS232IRQ		AVR32_USART1_IRQ
#define RS232PIDRX		AVR32_PDCA_PID_USART1_RX
#define RS232PIDTX		AVR32_PDCA_PID_USART1_TX

// Debug-Pins
#define DBG_PIN0		RED_LED
#define DBG_PIN1		GREEN_LED


#endif // HW_DEFS_H_
