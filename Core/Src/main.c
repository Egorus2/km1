/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <device.h>
#include <stdbool.h>
#include <string.h>
#include "mb.h"
#include "mbport.h"
#include "hx711.h"
#include "parser.h"
//#include
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_START_BYTE 0xFE
#define UART_STOP_BYTE 0xFF
#define UART_TX_BUF_LEN 3 + sizeof(uint32_t) + sizeof(long int)

#define SLAVE_ID    1
#define MODBUS_FUNC_WRITE_REG  0x06
#define SLAVE_ADDR  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
hx711_t hx711_handle;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern volatile long int optical_count;
extern volatile bool new_data;
extern volatile int32_t encoder_position;  // текущая позиция энкодера
extern volatile int32_t last_position;     // позиция на предыдущем измерении
extern volatile float encoder_rpm;      // скорость вращения
extern volatile uint32_t last_tick;
extern volatile int32_t motor_encoder_position;  // текущая позиция энкодера
extern volatile int32_t motor_last_position;     // позиция на предыдущем измерении
extern volatile float motor_encoder_rpm;      // скорость вращения
extern volatile uint32_t motor_last_tick;


//Основные переменные характеризующие состояние стенда
extern volatile int opt_lim_1;// пределы шкалы оптической линейки
extern volatile int opt_lim_2;
extern volatile bool limit_flag_1;//флаги срабатывания концевиков
extern volatile bool limit_flag_2;

extern float tenso_data;//текущая нагрузка
extern float set_tenso_data;//заданная требуемая нагрузка

extern float velocity;//текущая скорость
extern float set_velocity;//заданная тремуемая скорость

extern float position;//текущая позиция
extern float set_default_position;//заданное нулевое положение

extern float angle; //текущий угол
extern float set_angle; //заданное значение угла

int state = 0;
int prev_state = 0;
int step = 1;
int amplitude;
int regime;

enum states
{
	Initialization = 0,
	Calibration = 10,
	Working = 20,
	Stop = 30,
	Emergency_stop = 40
};






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 uint8_t data []= {1,1,1};
 send_uart_packet(0x00, data, 3, &huart1);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  eMBInit(MB_RTU, 0x01, 0, 38400, MB_PAR_EVEN);
  eMBEnable();
  HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_SET);
/*
  eMBInit(MB_RTU, 0x01, 0, 38400, MB_PAR_EVEN);
  eMBEnable();
  modbus_send_simple(1, 0x0191, 20);    // Ток
  HAL_Delay(100);
  modbus_send_simple(1, 0x01E1, 200);   // Скорость
  HAL_Delay(100);
  modbus_send_simple(1, 0x1801, 0x4001); // Вперед
  HAL_Delay(3000);
  modbus_send_simple(1, 0x1801, 0x4002); // Назад
  HAL_Delay(3000);
  modbus_send_simple(1, 0x1801, 0x0400); // Стоп
*/
  hx711_init(&hx711_handle, HX711_CLK_GPIO_Port, HX711_CLK_Pin, HX711_DOUT_GPIO_Port, HX711_DOUT_Pin);
  hx711_coef_set(&hx711_handle, 354.5); // read afer calibration
  hx711_tare(&hx711_handle, 10);


  uint8_t tx_buffer [UART_TX_BUF_LEN] = {0};
  tx_buffer[0] = UART_START_BYTE;
  tx_buffer[UART_TX_BUF_LEN - 2] = UART_STOP_BYTE;
  tx_buffer[UART_TX_BUF_LEN - 1] = (uint8_t)'\n';




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_Delay(500);




  while (1)
  {
    /* USER CODE END WHILE */
	//uint8_t * data = new(10);
    /* USER CODE BEGIN 3 */

	eMBPoll();


	switch(state)
	{
		case(Initialization):
		{
			//Добавить ввод данных(Режим работы, скорость, усилие)
			//Добавить вывод всех данных
			//set_tenso_data = ;
			//set_default_position = ;
			//set_velocity = ;
			//set_angle = ;
			//amplitude = ;
			//regime =;
			Encoder_Init();
			state = Calibration;
			prev_state = Initialization;
			break;
		}
		case(Calibration):
		{
			//двигаемся в сторону 1 концевика, когда дошли(limit_flag = 1), останавливаемся, записываем данные с линейки в opt_lim_1, так же в другую сторону.
			while(limit_flag_1 = 0)
			{
				//Motor_Move(set_velocity, step, 1);
			}
			opt_lim_1 = optical_count;
			//Motor_Move(set_velocity, step, -1);
			while(limit_flag_2 = 0)
			{
				//Motor_Move(set_velocity, step, -1);
			}
			opt_lim_2 = optical_count;
			//Motor_Move(set_velocity, step, 1);

			//если нужна нормировка шкалы [0, ...]
			/*
			 * opt_lim_1 = 0;
			 * opt_lim_2 = opt_lim_2 - opt_lim_1;
			 * position = encoder_position - opt_lim_1;
			 */
			while(position > set_default_position)
			{
				//Motor_Move(set_velocity, step, 1);
				position = optical_count;

			}

			state = Working;
			prev_state = Calibration;
			break;
		}
		case(Working):
		{
			while(limit_flag_1 || limit_flag_2 != 1)
			{
				if (regime = 1)//сжатие до обеспечения заданной нагрузки
				{
				//сжатие до обеспечения заданной нагрузки
					//из состояния initialization имеем переменную set_tenso_data заданную через ПО
					tenso_data = hx711_value(&hx711_handle);
					while(tenso_data < set_tenso_data)
					{
						//Motor_Move(set_velocity, step, 1);
						tenso_data = hx711_value(&hx711_handle);
					}
				}
				if (regime = 2)//колебания с заданой скоростью и амплитудой
				{
					position = optical_count;

					while(true)
					{
						//direction = 1;
						//Motor_Move(set_velocity, step, direction);
						position = optical_count;
						if ((set_default_position - position)*(set_default_position - position) > amplitude*amplitude)
						{
							//direction = -direction;
						}
					}
				}

			}
			state = Stop;
			prev_state = Working;
			break;
		}
		case(Stop):
		{

			break;
		}
		case(Emergency_stop):
		{

			break;
		}
	}



	  if(new_data)
	  {


	      long int optical_count_buf = optical_count;
	      char buffer3[64];
	      int len3 = snprintf(buffer3, sizeof(buffer3), "Opt linear: %ld\n", optical_count_buf);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer3, len3, HAL_MAX_DELAY);


	      //memcpy(tx_buffer + 1 + sizeof(uint32_t), &optical_count_buf, sizeof(long int));
	      //HAL_UART_Transmit(&huart1, tx_buffer, UART_TX_BUF_LEN, HAL_MAX_DELAY);


		  //HAL_Delay(1000);
		  //uint32_t tenso_data = (uint32_t) hx711_value(&hx711_handle);
		  //memcpy(tx_buffer + 1, &tenso_data, sizeof(uint32_t));
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tx_buffer, sizeof(tx_buffer));
		  //HAL_Delay(1000);


		  Encoder_UpdateRPM();
		  char buffer1[64];
		  char buffer2[64];
		  int len = snprintf(buffer1, sizeof(buffer1), "Pos: %ld, RPM: %.2f\r\n", encoder_position, encoder_rpm);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer1, len, HAL_MAX_DELAY);
		  int len1 = snprintf(buffer2, sizeof(buffer2), "Motor_Pos: %ld, Motor_RPM: %.2f\r\n", motor_encoder_position, motor_encoder_rpm);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer2, len1, HAL_MAX_DELAY);
		  HAL_Delay(100);
		  new_data = false;
	  }


  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
