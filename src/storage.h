/*
 * storage.h
 *
 *  Created on: 25.04.2012
 *      Author: Jan Alte, DO1FJN
 */

#ifndef STORAGE_H_
#define STORAGE_H_

#include "compiler.h"


bool format_internal_eeprom(void);

bool load_internal_eeprom(void);


bool have_configs(void);	// Exists a CFG0 block? Yes->true

/* get_loaded_config()
 * get a Cx config block address (already loaded from EEProm) and
 * checks the CRC
 * return NULL, if configbuffer is invalid or not present
 */
char	*get_loaded_config(unsigned char nr);

char	*get_cfg0_buffer(void);		// Pointer to RAM copy of CFG0

bool	save_cfg0_eeprom(unsigned int used_length);

#endif // STORAGE_H_
