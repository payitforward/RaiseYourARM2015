/*
 * Bluetooth.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef BLUETOOTH_BLUETOOTH_H_
#define BLUETOOTH_BLUETOOTH_H_


extern void bluetooth_init(uint32_t baudrate);
extern void bluetooth_send(const uint8_t *pui8Buffer, uint32_t ui32Count);
extern uint16_t bluetooth_recv(uint8_t* rxBuf, uint16_t numToRead, bool block);
extern void bluetooth_print(const char * restrict format, ...);

#endif /* BLUETOOTH_BLUETOOTH_H_ */
