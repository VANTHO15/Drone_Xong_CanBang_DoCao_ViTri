/*
 * MySerial.c
 *
 *  Created on: Jul 15, 2021
 *      Author: Admin
 */
#include <string.h>
#include "MySerial.h"

MySerial::MySerial(UART_HandleTypeDef *huart)
{
	uart = huart;
	rx_buffer = { { 0 }, 0, 0};
	tx_buffer = { { 0 }, 0, 0};
}
void MySerial::Init(void)
{
	_rx_buffer = &rx_buffer;
	_tx_buffer = &tx_buffer;

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(uart, UART_IT_ERR);

	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(uart, UART_IT_RXNE);
}

void MySerial::StoreChar(unsigned char c, RingBuffer_t *buffer)
{
	unsigned int i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if(i != buffer->tail) {
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}
int MySerial::Read(void)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if(_rx_buffer->head == _rx_buffer->tail)
	{
		return -1;
	}
	else
	{
		unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
		_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
		return c;
	}
}

void MySerial::Write(int c)
{
	if (c>0)
	{
		unsigned int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE;

		// If the output buffer is full, there's nothing for it other than to
		// wait for the interrupt handler to empty it a bit
		// ???: return 0 here instead?
		while (i == _tx_buffer->tail);

		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)c;
		_tx_buffer->head = i;

		__HAL_UART_ENABLE_IT(uart, UART_IT_TXE); // Enable UART transmission interrupt
//				HAL_UART_Transmit(uart, _tx_buffer->buffer, sizeof(_tx_buffer->buffer), 1000);

	}
}
int MySerial::IsDataAvailable(void)
{
	return (uint16_t)(UART_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % UART_BUFFER_SIZE;
}

uint16_t MySerial::GetPos (char *string)
{
	static uint8_t so_far;
	uint16_t counter;
	int len = strlen (string);
	if (_rx_buffer->tail>_rx_buffer->head)
	{
		if (this->Read() == string[so_far])
		{
			counter=UART_BUFFER_SIZE-1;
			so_far++;
		}
		else so_far=0;
	}
	unsigned int start = _rx_buffer->tail;
	unsigned int end = _rx_buffer->head;
	for (unsigned int i=start; i<end; i++)
	{
		if (this->Read() == string[so_far])
		{
			counter=i;
			so_far++;
		}
		else so_far =0;
	}

	if (so_far == len)
	{
		so_far =0;
		return counter;
	}
	else return -1;
}

void MySerial::SendString(const char *s)
{
	while(*s) this->Write(*s++);
}

void MySerial::SendChar( char s){
	this->Write(s);
}
void MySerial::SendBrustByte( uint8_t *s, uint8_t numberOfBytes)
{
//	for (int i = 0; i < numberOfBytes; i++){
//		this->Write(s[i]);
//	}
	//while(*s) this->Write(*s++);

	HAL_UART_Transmit(this->uart, s, numberOfBytes, 20);
}
void MySerial::PrintBase(long n, uint8_t base)
{
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *s = &buf[sizeof(buf) - 1];

	*s = '\0';

	// prevent crash if called with base == 1
	if (base < 2) base = 10;

	do {
		unsigned long m = n;
		n /= base;
		char c = m - base * n;
		*--s = c < 10 ? c + '0' : c + 'A' - 10;
	} while(n);

	while(*s) this->Write(*s++);
}

void MySerial::GetString(char *buffer)
{
	int index=0;

	while (_rx_buffer->tail>_rx_buffer->head)
	{
		if ((_rx_buffer->buffer[_rx_buffer->head-1] == '\n')||((_rx_buffer->head == 0) && (_rx_buffer->buffer[UART_BUFFER_SIZE-1] == '\n')))
		{
			buffer[index] = this->Read();
			index++;
		}
	}
	unsigned int start = _rx_buffer->tail;
	unsigned int end = (_rx_buffer->head);
	if ((_rx_buffer->buffer[end-1] == '\n'))
	{

		for (unsigned int i=start; i<end; i++)
		{
			buffer[index] = this->Read();
			index++;
		}
	}
}

int MySerial::WaitUntil(char *string, char*buffertostore)
{
	while (!(IsDataAvailable()));
	int index=0;

	while (_rx_buffer->tail>_rx_buffer->head)
	{
		if ((_rx_buffer->buffer[_rx_buffer->head-1] == '\n')||((_rx_buffer->head == 0) && (_rx_buffer->buffer[UART_BUFFER_SIZE-1] == '\n')))
		{
			buffertostore[index] = this->Read();
			index++;
		}
	}

	unsigned int start = _rx_buffer->tail;
	unsigned int end = (_rx_buffer->head);
	if ((_rx_buffer->buffer[end-1] == '\n'))
	{
		for (unsigned int i=start; i<end; i++)
		{
			buffertostore[index] = this->Read();
			index++;
		}
		return 1;
	}
	return 0;
}


void MySerial::UartIsr ()
{
	uint32_t isrflags   = READ_REG(this->uart->Instance->SR);
	uint32_t cr1its     = READ_REG(this->uart->Instance->CR1);

	/* if DR is not empty and the Rx Int is enabled */
	if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
	{
		/******************
		 *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
		 *          error) and IDLE (Idle line detected) flags are cleared by software
		 *          sequence: a read operation to USART_SR register followed by a read
		 *          operation to USART_DR register.
		 * @note   RXNE flag can be also cleared by a read to the USART_DR register.
		 * @note   TC flag can be also cleared by software sequence: a read operation to
		 *          USART_SR register followed by a write operation to USART_DR register.
		 * @note   TXE flag is cleared only by a write to the USART_DR register.

		 *********************/
		this->uart->Instance->SR;                       /* Read status register */
		unsigned char c = this->uart->Instance->DR;     /* Read data register */
		this->StoreChar(c, _rx_buffer);  // store data in buffer
		return;
	}

	/*If interrupt is caused due to Transmit Data Register Empty */
	if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
	{
		if(tx_buffer.head == tx_buffer.tail)
		{
			// Buffer empty, so disable interrupts
			__HAL_UART_DISABLE_IT(this->uart, UART_IT_TXE);

		}

		else
		{
			// There is more data in the output buffer. Send the next byte
			unsigned char c = tx_buffer.buffer[tx_buffer.tail];
			tx_buffer.tail = (tx_buffer.tail + 1) % UART_BUFFER_SIZE;

			/******************
			 *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
			 *          error) and IDLE (Idle line detected) flags are cleared by software
			 *          sequence: a read operation to USART_SR register followed by a read
			 *          operation to USART_DR register.
			 * @note   RXNE flag can be also cleared by a read to the USART_DR register.
			 * @note   TC flag can be also cleared by software sequence: a read operation to
			 *          USART_SR register followed by a write operation to USART_DR register.
			 * @note   TXE flag is cleared only by a write to the USART_DR register.

			 *********************/

			this->uart->Instance->SR;
			this->uart->Instance->DR = c;

		}
		return;
	}
}
