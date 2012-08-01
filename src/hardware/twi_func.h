/*
 * twi_func.h
 *
 * I²C functions. This module is not completed jet! A complete version must
 * implement eeprom_read()/eeprom_write() functions (handle 24C512 eeprom).
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
 */

#ifndef TWI_FUNC_H_
#define TWI_FUNC_H_

#define TWICLOCK	400000L

typedef enum {
  TWIok, TWIbusy, TWInak, TWIerror
} tTWIresult;


typedef void (*twi_handler)(tTWIresult, unsigned int);


void	twi_init(void);
void	twi_exit(void);

char	twi_busy(void);

void	twi_pause(void);
void	twi_continue(void);


tTWIresult twi_write(unsigned char adr, const char *data, unsigned int len, twi_handler RetFunc);
tTWIresult twi_read(unsigned char adr, char *dest, unsigned int len, twi_handler RetFunc);
void twi_blocking_write(unsigned char adr, const char *data, unsigned int len, twi_handler RetFunc);

tTWIresult ee_write(unsigned int adr, const char *data, unsigned int len, twi_handler RetFunc);
tTWIresult ee_read(unsigned int adr, char *dest, unsigned int len, twi_handler RetFunc);

// Register R/W function: Handling a 1-4 byte register / addressing with a 2nd byte
tTWIresult reg_write(unsigned char adr, unsigned char reg, unsigned int value, char reg_size, twi_handler RetFunc);
tTWIresult reg_read(unsigned char adr,  unsigned char reg, char *dest, char reg_size, twi_handler RetFunc);
// same like reg_write(), waits until the TWI job buffer have one position left
void reg_blocking_write(unsigned char adr, unsigned char reg, unsigned int value, char reg_size, twi_handler RetFunc);


#endif
