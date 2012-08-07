/*
 * ambe_func.h
 *
 * Header-File for complex communication functions of AMBE-2020 IC.
 *
 *  Created on: 20.03.2009
 *      Author: DO1FJN, Jan Alte
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


#ifndef AMBE_FUNC_H_
#define AMBE_FUNC_H_


#define AMBE_HEADER	0x13EC
#define AMBE_FRAMESIZE	24		// in words

#define AMBE_JITTER	8		// allowed jitter in Bit-Times (Modulator)

#define	AMBE_RATE0	0x1030		// 2400bit/s Voice + 1200bits FEC Block Code
#define AMBE_RATE1	0x4000
#define AMBE_RATE2 	0x0000
#define AMBE_RATE3	0x0000
#define AMBE_RATE4	0x0048

#define AMBE_USEDDATA	9		// NoOfBytes in Data-Field, rate-depended
#define AMBE_MAXVOL	168		// max. used Volume Byte

#define AMBE_CALLTONE0	144		// 350+440Hz
#define AMBE_CALLTONE1	145		// 440+480Hz
#define AMBE_CALLTONE2	146		// 480+650Hz
#define	AMBE_ACKTONE	28		// 875Hz
#define	AMBE_ERRTONE	42		// ~1320Hz (n x 31.25Hz)


typedef struct {
    unsigned short Header;
    unsigned short Ctrl1;
    unsigned short Rate[5];
    unsigned short Unused[3];
    unsigned short DtmfCtrl;
    unsigned short Ctrl2;
	     char  Data[24];
} tAMBEinframe;

typedef struct {
    unsigned short Header;
    unsigned short Ctrl1;
    unsigned short Rate[5];
    unsigned short BER;
    unsigned short SoftDecDis;
    unsigned short CurrentBitErrors;
    unsigned short DtmfCtrl;
    unsigned short Ctrl2;
             char  Data[24];
} tAMBEoutframe;

typedef int (*tbits_left_fkt)(void);


typedef enum {
  AMBEnoinit, AMBEoff, AMBEpowerdown, AMBEbooting,	// no operational states
  AMBEgo2sleep, AMBEsleep, AMBEencoding, AMBEdecoding 	// op-states
} tambestate;


void init_ambe_ssc(void);

void	ambe_setup(void);		// Sets Buffer-Parameter and Vars once after reset
void	ambe_init(void);
void	ambe_powerdown(void);		// Kontrolliertes Herunterfahren des AMBE
void	ambe_encode(void);
void	ambe_decode(void);
void	ambe_standby(void);
void	ambe_stop(void);

tambestate ambe_getstate(void);


tambestate ambe_getnewstate(void);	// nur neue Zustände, sonst "noinit"


int	ambe_packetcount(void);		// gibt Paketzähler zurück

void	ambe_set_vollin(unsigned char volume);	// direct value for AMBE volume byte

void	ambe_set_dtmf(unsigned char tone, int len_frames);
void	ambe_stop_dtmf(void);

void	ambe_double_tone(unsigned char tone, int len_frames, int len_pause, unsigned char tone2nd);

// ambe_set_timeout() setzt dem Decoder ein Zeitlimit in (x mal 20ms)
// läuft die Zeit ab, ohne das neue Pakete (putvoice) eintreffen,
// schaltet der AMBE ab.
void	ambe_set_timeout(unsigned int time2shutdown);


unsigned short ambe_getber(void);	// gibt Bit Error Rate zurück

int	ambe_have_new_encoded(void);	// returns true, if new encoded data available
void	ambe_getvoice(unsigned char *dest);
void	ambe_putvoice(unsigned char *voicedat);

void	ambe_getsilence(unsigned char *dest);


#endif
