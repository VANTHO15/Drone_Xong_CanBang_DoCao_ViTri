/*
 * MySerial.h
 *
 *  Created on: Jul 15, 2021
 *      Author: Admin
 */

#ifndef _MY_SERIAL_H_
#define _MY_SERIAL_H_



#include "stm32f4xx_hal.h"
#include "main.h"
/* change the size of the buffer */
#define UART_BUFFER_SIZE 256

typedef struct {
	unsigned char buffer[UART_BUFFER_SIZE];
	volatile unsigned int head;
	volatile unsigned int tail;
}RingBuffer_t;

class MySerial
{
public:
	UART_HandleTypeDef *uart ;


	MySerial(UART_HandleTypeDef *huart);

	void SelectUART(UART_HandleTypeDef *huart);
	/* reads the data in the rx_buffer and increment the tail count in rx_buffer */
	int Read(void);

	/* writes the data to the tx_buffer and increment the head count in tx_buffer */
	void Write(int c);

	/* function to send the string to the uart */
	void SendString(const char *s);
	void SendChar(char s);
	void SendBrustByte( uint8_t *s, uint8_t numberOfBytes);
	/* Print a number with any base
	 * base can be 10, 8 etc*/
	void PrintBase(long n, uint8_t base);

	/* Initialize the ring buffer */
	void Init(void);

	/* checks if the data is available to read in the rx_buffer */
	int IsDataAvailable(void);

	/* get the position of the given string within the incoming data.
	 * It returns the position, where the string ends */
	uint16_t GetPos(char *string);

	/* the ISR for the uart. put it in the IRQ handler */
	void UartIsr();

	/* once you hit 'enter' (\r\n), it copies the entire string to the buffer*/
	void GetString(char *buffer);

	/* keep waiting until the given string has arrived and stores the data in the buffertostore
	 * on success return 1
	 * or else returns 0
	 * @ usage :  while (!(wait_until("\r\n", buffer)));
	 */
	int WaitUntil(char *string, char *buffertostore);

private:
	RingBuffer_t rx_buffer;
	RingBuffer_t tx_buffer;

	RingBuffer_t *_rx_buffer;
	RingBuffer_t *_tx_buffer;

	void StoreChar(unsigned char c, RingBuffer_t *buffer);
};



#endif /* UARTRINGBUFFER_H_ */
