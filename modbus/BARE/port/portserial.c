/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
static UART_HandleTypeDef huart;
static GPIO_TypeDef * DE_Port;
static uint16_t DE_Pin;

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if (xRxEnable) { // если переключаемся в режим приема
        // переключаем драйвер в режим приема (см. принципы работы rs485)
        HAL_GPIO_WritePin(DE_Port, DE_Pin, GPIO_PIN_RESET);
        __HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE); // разрешаем прерывание, реагирует если буфер приема не пуст
    } else {
        __HAL_UART_DISABLE_IT(&huart, UART_IT_RXNE); // запрещаем прерывание
    }
    if (xTxEnable) { // если переключаемся в режим передачи
        // переключаем драйвер в режим передачи (см. принципы работы rs485)
        HAL_GPIO_WritePin(DE_Port, DE_Pin, GPIO_PIN_SET);
        __HAL_UART_ENABLE_IT(&huart, UART_IT_TXE); // разрешаем прерывание, реагирует если буфер передачи пуст
    } else {
        __HAL_UART_DISABLE_IT(&huart, UART_IT_TXE); // запрещаем прерывание
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
		huart.Init.Mode = UART_MODE_TX_RX; // работаем на прием и передачу
	    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE; // без контроля потока (у нас же rs485)
	    // сэмплинг, не могу нормально объяснить, но это нужно для защиты от шумов
	    huart.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
	    huart.Init.OverSampling = UART_OVERSAMPLING_16;
	    // один стоп-бит
	    huart.Init.StopBits = UART_STOPBITS_1;
	    // без доп-фич
	    huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	    // портозависимая инициализация: UART, ножка контроля направления приемопередачи
	    switch (ucPORT) {
	    case 0:
	        huart.Instance = USART1;
	        DE_Port = GPIOA;
	        DE_Pin = GPIO_PIN_12;
	            break;
	    case 1:
	        huart.Instance = USART2;
	        DE_Port = GPIOA;
	        DE_Pin = GPIO_PIN_1;
	            break;
	    case 2:
	        huart.Instance = USART3;
	        DE_Port = GPIOB;
	        DE_Pin = GPIO_PIN_14;
	            break;
	    default:
	        // вызовем ошибку, если выбрали не тот номер порта, хотя это будет ошибкой портирования, но хоть что-то
	        return FALSE;
	    }

	    // скорость
	    huart.Init.BaudRate = ulBaudRate;

	    // размер слова
	    switch (ucDataBits) {
	    case 8:
	        huart.Init.WordLength = UART_WORDLENGTH_8B;
	        break;
	    case 9:
	        huart.Init.WordLength = UART_WORDLENGTH_9B;
	        break;
	    default:
	        // вызовем ошибку, если выбрали не тот номер порта, хотя это будет ошибкой портирования, но хоть что-то
	        return FALSE;
	    }

	    // настраиваем бит четности
	    switch (eParity) {
	    case MB_PAR_NONE:
	        huart.Init.Parity = UART_PARITY_NONE;
	        break;
	    case MB_PAR_EVEN:
	        huart.Init.Parity = UART_PARITY_EVEN;
	        break;
	    case MB_PAR_ODD:
	        huart.Init.Parity = UART_PARITY_ODD;
	        break;
	    default:
	        // вызовем ошибку, если выбрали не тот номер порта, хотя это будет ошибкой портирования, но хоть что-то
	        return FALSE;
	    }

	    // Инициализируем порт, как rs485
	    return HAL_OK == HAL_RS485Ex_Init(&huart, UART_DE_POLARITY_HIGH, 0, 0) ? TRUE : FALSE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	huart.Instance->TDR = ucByte;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = huart.Instance->RDR;
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */

BOOL UART_IRQ_Handler(USART_TypeDef * usart) {
	if (usart == huart.Instance) {
		if((__HAL_UART_GET_IT(&huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart, UART_IT_RXNE) != RESET)) {
			pxMBFrameCBByteReceived();
			__HAL_UART_SEND_REQ(&huart, UART_RXDATA_FLUSH_REQUEST);
			return TRUE;
		}
		if((__HAL_UART_GET_IT(&huart, UART_IT_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(&huart, UART_IT_TXE) != RESET)) {
			pxMBFrameCBTransmitterEmpty();
			return TRUE;
		}
	}
	return FALSE;
}

static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
