/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "port.h"
#include <device.h>
#include "parser.h"
extern UART_HandleTypeDef huart1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern long int optical_count;
extern int32_t encoder_position;  // текущая позиция энкодера
extern volatile int32_t last_position;     // позиция на предыдущем измерении
extern volatile float encoder_rpm;      // скорость вращения
extern volatile uint32_t last_tick;
extern int32_t motor_encoder_position;  // текущая позиция энкодера
extern volatile int32_t motor_last_position;     // позиция на предыдущем измерении
extern volatile float motor_encoder_rpm;      // скорость вращения
extern volatile uint32_t motor_last_tick;
extern bool new_data;
extern uint16_t downcounter;
volatile uint8_t encoder_last_state = 0;
volatile uint8_t motor_encoder_last_state = 0;
extern bool limit_flag_1;
extern bool limit_flag_2;
bool limit_flag = 0;
extern uint8_t rx_byte;
extern volatile int stop_count;
volatile GPIO_PinState limit_prev_13 = GPIO_PIN_RESET;
volatile GPIO_PinState limit_prev_14 = GPIO_PIN_RESET;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENC_A_ITR0_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENC_B_ITR1_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MOTOR_ENC_A_ITR11_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(OPT_LINEAR_Z_ITR5_Pin);
  HAL_GPIO_EXTI_IRQHandler(OPT_LINEAR_B_ITR6_Pin);
  HAL_GPIO_EXTI_IRQHandler(OPT_LINEAR_A_ITR7_Pin);
  HAL_GPIO_EXTI_IRQHandler(MOTOR_ENC_B_ITR9_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  GPIO_PinState first_optical_signal = HAL_GPIO_ReadPin(OPT_LINEAR_A_ITR7_GPIO_Port, OPT_LINEAR_A_ITR7_Pin);
  GPIO_PinState second_optical_signal = HAL_GPIO_ReadPin(OPT_LINEAR_B_ITR6_GPIO_Port, OPT_LINEAR_B_ITR6_Pin);
  GPIO_PinState zero_optical_signal = HAL_GPIO_ReadPin(OPT_LINEAR_Z_ITR5_GPIO_Port, OPT_LINEAR_Z_ITR5_Pin);



    if (second_optical_signal == GPIO_PIN_SET)
    {
      optical_count++;
    }
    else
    {
      optical_count--;
    }


    if (first_optical_signal == GPIO_PIN_SET)
    {
      optical_count--;
    }
    else
    {
      optical_count++;
    }

  if (zero_optical_signal == GPIO_PIN_SET)
  {
    optical_count = 0;
  }
  new_data = true;


  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  Encoder_UpdateRPM();

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	/* TIM Update event */
	if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) !=RESET) {
		__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
		if (!--downcounter)
			prvvTIMERExpiredISR();
	}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  //HAL_GPIO_EXTI_IRQHandler(MOTOR_ENC_A_ITR11_old_Pin);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(ENC_Z_ITR12_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt / UART5 wake-up interrupt through EXTI line 35.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        parser_process_byte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // заново включаем прием
    }
}

uint32_t tiks = 0 ;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	    uint8_t enc_a = HAL_GPIO_ReadPin(ENC_A_ITR0_GPIO_Port, ENC_A_ITR0_Pin);
	    uint8_t enc_b = HAL_GPIO_ReadPin(ENC_B_ITR1_GPIO_Port, ENC_B_ITR1_Pin);
	    uint8_t encoder_state = (enc_a << 1) | enc_b;

	    uint8_t motor_enc_a = HAL_GPIO_ReadPin(MOTOR_ENC_A_ITR11_GPIO_Port, MOTOR_ENC_A_ITR11_Pin);
	    uint8_t motor_enc_b = HAL_GPIO_ReadPin(MOTOR_ENC_B_ITR9_GPIO_Port, MOTOR_ENC_B_ITR9_Pin);
	    uint8_t motor_encoder_state = (motor_enc_a << 1) | motor_enc_b;

	    if (GPIO_Pin == ENC_A_ITR0_Pin || GPIO_Pin == ENC_B_ITR1_Pin)
	    {
	        // таблица направлений (Gray code)
	        if ((encoder_last_state == 0b00 && encoder_state == 0b01) ||
	            (encoder_last_state == 0b01 && encoder_state == 0b11) ||
	            (encoder_last_state == 0b11 && encoder_state == 0b10) ||
	            (encoder_last_state == 0b10 && encoder_state == 0b00))
	        {
	            encoder_position++;
	        }
	        else if ((encoder_last_state == 0b00 && encoder_state == 0b10) ||
	                 (encoder_last_state == 0b10 && encoder_state == 0b11) ||
	                 (encoder_last_state == 0b11 && encoder_state == 0b01) ||
	                 (encoder_last_state == 0b01 && encoder_state == 0b00))
	        {
	            encoder_position--;
	        }
	        encoder_last_state = encoder_state;
	    }


	    if (GPIO_Pin == MOTOR_ENC_A_ITR11_Pin || GPIO_Pin == MOTOR_ENC_B_ITR9_Pin)
	    {
	    	if ((motor_encoder_last_state == 0b00 && motor_encoder_state == 0b10) ||
	    	    (motor_encoder_last_state == 0b10 && motor_encoder_state == 0b11) ||
	    	    (motor_encoder_last_state == 0b11 && motor_encoder_state == 0b01) ||
	    	    (motor_encoder_last_state == 0b01 && motor_encoder_state == 0b00))
	    	{
	    	    motor_encoder_position++;
	    	}
	    	// counter-clockwise (reverse)
	    	else if ((motor_encoder_last_state == 0b00 && motor_encoder_state == 0b01) ||
	    	         (motor_encoder_last_state == 0b01 && motor_encoder_state == 0b11) ||
	    	         (motor_encoder_last_state == 0b11 && motor_encoder_state == 0b10) ||
	    	         (motor_encoder_last_state == 0b10 && motor_encoder_state == 0b00))
	    	{
	    	    motor_encoder_position--;
	    	}
	    	motor_encoder_last_state = motor_encoder_state;
	    }

    if(GPIO_Pin == ENC_Z_ITR12_Pin) // Z
    {
        encoder_position = 0; // сброс позиции на индекс
    }
	if ((GPIO_Pin == GPIO_PIN_14) || (GPIO_Pin == GPIO_PIN_13))
	{
	uint8_t curr14 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);
	uint8_t curr13 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET);
	if (curr13)
		{

		    GPIO_PinState curr = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);

		    //if (curr == GPIO_PIN_RESET && limit_prev_13 == GPIO_PIN_SET )
		    //{
		        limit_flag_1 = 1;
		        //Motor_Stop();
		        limit_flag = 1;
		    //}

		    limit_prev_13 = curr;



	        // Сбрасываем таймер
	        //__HAL_TIM_SET_COUNTER(&htim2, 0);

	        // Запускаем таймер на антидребезг
	        //HAL_TIM_Base_Start_IT(&htim2);


		}
		else if(curr14)
		{

				GPIO_PinState curr2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		        limit_flag_2 = 1;
		        limit_flag   = 1;
		        limit_prev_14 = curr2;


	        // Выключаем EXTI, чтобы дребезг не дергал ISR
			//HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	        // Сбрасываем таймер
	        //__HAL_TIM_SET_COUNTER(&htim2, 0);

	        // Запускаем таймер на антидребезг
	       // HAL_TIM_Base_Start_IT(&htim2);


			//char msg1 = 'B'; // символ для отправки
			//HAL_UART_Transmit(&huart1, (uint8_t*)&msg1, 1, HAL_MAX_DELAY);

		}
	}
	new_data = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2)
    {
    	//Motor_Stop();
        //HAL_TIM_Base_Stop_IT(&htim2);          // Остановить таймер
        //__HAL_TIM_SET_COUNTER(&htim2, 0);      // Сбросить счётчик
    	limit_flag = 0;
		//Motor_Stop();
		//stop_count ++;

    }
    if(htim->Instance == TIM5)
    {
            Sensor_Poll();
    }
}

/* USER CODE END 1 */
