/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	EEprom.h
 * @brief	EEPROM control
 */

#include "../include.h"
#include "driverlib/eeprom.h"

typedef enum
{
	EEPROM_MSG_INVALID_ADDRESS = 0,
	EEPROM_MSG_INVALID_NUMOFWORDS,
	EEPROM_MSG_OK
} EEPROM_MSG;

//* Private function prototype ----------------------------------------------*/
static EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);
static EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);

/**
 * @brief Write eeprom
 * @param pui32_Data pointer to data array
 * @param ui32WordAddress write address
 * @param NumOfWords number of words
 * @return 	EEPROM_MSG_INVALID_ADDRESS
 * 			EEPROM_MSG_INVALID_NUMOFWORDS
 */
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

/**
 * @brief read eeprom
 * @param pui32_Data pointer to data array
 * @param ui32WordAddress read address
 * @param NumOfWords number of words
 * @return 	EEPROM_MSG_INVALID_ADDRESS
 * 			EEPROM_MSG_INVALID_NUMOFWORDS
 */
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

