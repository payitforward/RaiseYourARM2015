/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	bluetooth.c
 * @brief
 */

#include "../include.h"
#include "Bluetooth.h"

/** Private Defines **/
#define MAX_RX_BUF              1500


/** Private Variables **/
static volatile uint8_t rxBuffer[MAX_RX_BUF];
static volatile uint16_t rxHead;
static uint16_t rxTail;

static uint16_t ReadRxBuffer(uint8_t* rxBuf, uint16_t numToRead);
static void Bluetooth_RxHandler(void);

void bluetooth_init(uint32_t baudrate)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), baudrate,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    UARTIntRegister(UART0_BASE, &Bluetooth_RxHandler);
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

static void Bluetooth_RxHandler(void)
{
    ROM_UARTIntClear(UART0_BASE, ROM_UARTIntStatus(UART0_BASE, true));

    while (ROM_UARTCharsAvail(UART0_BASE))
    {
        if (rxHead + 1 < MAX_RX_BUF)
        {
          if((rxHead + 1) != rxTail){
               rxBuffer[rxHead++] = ROM_UARTCharGet(UART0_BASE);
          }
        }
        else
        {
          if(0 != rxTail){
            rxBuffer[rxHead] = ROM_UARTCharGet(UART0_BASE);
            rxHead = 0;
          }
        }
    }
}

uint16_t bluetooth_recv(uint8_t* rxBuf, uint16_t numToRead, bool block)
{
     if (block)
     {
          // Blocking mode, so don't return until we read all the bytes requested
          uint16_t bytesRead;
          // Keep getting data if we have a number of bytes to fetch
          while (numToRead)
          {
               bytesRead = ReadRxBuffer(rxBuf, numToRead);
               if (bytesRead)
               {
                    rxBuf += bytesRead;
                    numToRead -= bytesRead;
               }
          }
          return bytesRead;
     }
     else
     {
          // Non-blocking mode, just read what is available in buffer
          return ReadRxBuffer(rxBuf, numToRead);
     }
}

static uint16_t ReadRxBuffer(uint8_t* rxBuf, uint16_t numToRead)
{
	uint16_t bytesRead = 0;

	while(rxTail != rxHead && bytesRead < numToRead)
	{
		if (rxTail  + 1 < MAX_RX_BUF)
		{
			*rxBuf++ = rxBuffer[rxTail++];
			bytesRead++;
		}
		else
		{
			*rxBuf++ = rxBuffer[rxTail];
			rxTail = 0;
			bytesRead++;
		}
	}

	return bytesRead;
}

void bluetooth_send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

void bluetooth_print(const char * restrict format, ...)
{
	volatile char Tx_Buf[512];
    va_list arg;

    memset((void *)Tx_Buf, 0, sizeof(Tx_Buf));
    //
    // Start the varargs processing.
    //
    va_start(arg, format);

    //
    // Call vsnprintf to perform the conversion.  Use a large number for the
    // buffer size.
    //
    uvsnprintf((char *)Tx_Buf, 0xffff, format, arg);

    //
    // End the varargs processing.
    //
    va_end(arg);

    bluetooth_send((const uint8_t *)Tx_Buf, strlen((const char *)Tx_Buf));
}
