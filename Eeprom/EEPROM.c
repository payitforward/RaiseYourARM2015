/*
 * EEPROM.c
 *
 *  Created on: Aug 6, 2013
 *      Author: Admin
 */
#include "../include.h"
#include "driverlib/eeprom.h"

typedef enum
{
	EEPROM_MSG_INVALID_ADDRESS = 0,
	EEPROM_MSG_INVALID_NUMOFWORDS,
	EEPROM_MSG_OK
} EEPROM_MSG;

static EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);
static EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);

static EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
{
	uint32_t Address;
	if (ui32WordAddress > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_ADDRESS);
	}
	if ((ui32WordAddress + NumOfWords) > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_NUMOFWORDS);
	}
	Address = ui32WordAddress << 2;
	ROM_EEPROMProgram(pui32_Data, Address, NumOfWords << 2);

	return EEPROM_MSG_OK;
}

static EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
{
	uint32_t Address;
	if (ui32WordAddress > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_ADDRESS);
	}
	if ((ui32WordAddress + NumOfWords) > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_NUMOFWORDS);
	}
	Address = ui32WordAddress << 2;
	ROM_EEPROMRead(pui32_Data, Address, NumOfWords << 2);

	return EEPROM_MSG_OK;
}

