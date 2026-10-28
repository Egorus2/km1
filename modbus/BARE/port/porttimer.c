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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

#include "stm32g4xx_hal_tim.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
void prvvTIMERExpiredISR( void );
static TIM_HandleTypeDef htim;
static uint16_t timeout = 0;
uint16_t downcounter = 0;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	htim.Instance = TIM4; // указываем, что будем работать с 6 таймером
	    htim.Init.CounterMode = TIM_COUNTERMODE_UP; // тип работы таймера (от 0 и вверх)
	    /* инициализируем делитель частоты таймера, нам же не нужен таймер на 32МГц
	       HAL_RCC_GetPCLK1Freq() - возвращает частоту PCLK1, в нашем случае будет 32000000
	       делим на 1000000, чтобы получить делитель для работы таймера с частотой 1МГц
	       а -1 - это самый сок: дело в том, что 0 - это делитель на 1, 1 - делитель на 2 и т.д.*/
	    htim.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) - 1;
	    // период 50 мкс, с -1, думаю, все понятно
	    htim.Init.Period = 50 - 1;

	    // запишем значение таймаута, оно понадобится для перезапуска таймера
	    timeout = usTim1Timerout50us;

	    // все, вызываем инициализацию таймера, все остальное сделает HAL
	    return HAL_OK == HAL_TIM_Base_Init(&htim) ? TRUE : FALSE;
}


inline void
vMBPortTimersEnable(  )
{
    downcounter = timeout;
    HAL_TIM_Base_Start_IT(&htim);
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
}

inline void
vMBPortTimersDisable(  )
{
	 HAL_TIM_Base_Stop_IT(&htim);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}
