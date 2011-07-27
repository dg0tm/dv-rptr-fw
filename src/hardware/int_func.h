/*
 * int_func.h
 *
 * handles external int sources. Function prototye for an idle timer.
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


#ifndef INT_FUNC_H_
#define INT_FUNC_H_

#include "intc.h"

typedef enum {
  EICm_FALLING_EDGE, EICm_RAISING_EDGE,
  EICm_LOW_LEVEL, EICm_HIGH_LEVEL
} teictrigmode;


void	eic_enableint(unsigned int intnum, __int_handler handler, unsigned int int_lev, teictrigmode mode);
void	eic_disableint(unsigned int intnum);
void	eic_reenableint(unsigned int intnum);

void	eic_clrline(unsigned int intnum);

void	eic_changemode(unsigned int intnum, teictrigmode mode);
void	eic_setfilter(unsigned int intnum, int filtered);

int	eic_is_raising_edge(unsigned int intnum);


#define CALC_CLOCKS_FROM(x)     (((MASTERCLOCK/128)*x)/1000)

// IDLE Timer: Keep µC alive, if no other ints handled. Keep Trigger external WD
typedef void (*tidleproc)(void);        // void-void Functions generell

void	idle_timer_start(void);
void	idle_timer_stop(void);
void    idle_timer_custom(tidleproc handler, unsigned short clocks);

#endif // INT_FUNC_H_
