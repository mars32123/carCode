/*
 * DataStore.h
 *
 *  Created on: 2021年2月3日
 *      Author: 周文奇
 */

#ifndef CODE_DATASTORE_H_
#define CODE_DATASTORE_H_
#include "zf_eeprom.h"
void data_store(void);
void data_load(void);
extern uint32 sector;

#endif /* CODE_DATASTORE_H_ */
