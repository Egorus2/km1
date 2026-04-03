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
extern volatile hx711_t hx711_handle;
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
int32_t mode1_start = 0;

//Основные переменные характеризующие состояние стенда
extern volatile int opt_lim_1;// пределы шкалы оптической линейки
extern volatile int opt_lim_2;
volatile float encoder_lim_1 = 0;
volatile float encoder_lim_2 = 0;

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

int t = 0;
int t0 = 0;
int v0 = 10;
int v1 = 50;
int time1 = 1;

float zero_position = 0.0;
volatile int state = 0;
int prev_state = 0;
int step = 1;
int amplitude;
int angle_amplitude;
int regime;
int direction;
int count = 0;
int pull_number;
int set_current = 60;
volatile int stop_count = 0;
int opt_flag_1 = 0;
int opt_flag_2 = 0;
int z1 = 0;
uint8_t rx_byte = 0;
int control = 0;
int dir_ctrl = 1;
int start_tick;
int ramp = 0;
float k_p = 0.2;
int x1 = 0;
float angle_velocity;
float min_angle = 0;
float max_angle = 0;
float angle_velocity_ref;
int acc_coef;

#pragma pack(push, 1)

typedef struct {
    float min_angle;
    float max_angle;
    float freq;
    int count;
    float moment;
} mode1_config_t;

typedef struct {
    float speed;
    float start_angle;
    float end_angle;
    uint32_t hold_ms;
} mode2_config_t;

typedef struct {
    float torque;
    uint32_t duration;
    uint8_t direction;
} torque_t;

typedef struct {
    int ts_ms;
    float load;
    float pos;
    float angle1;
    float angle2;
    float temp;
    uint16_t flags;
} stream_data_t;

typedef struct {
    float p;
    float i;
    float d;
} pid_t;


#pragma pack(pop)


mode1_config_t g_mode1_cfg;
mode2_config_t g_mode2_cfg;
torque_t       g_torque_cmd;
pid_t          g_pid_cfg;
stream_data_t  g_stream_data;


enum states
{
	INITIALIZATION = 0x00,
	CALIBRATION = 0x01,
	MODE1_CONFIG = 0x10,
	MODE1_START = 0x11,
	MODE1_STOP = 0x12,
	MODE2_CONFIG = 0x20,
	MODE2_START = 0x21,
	MODE2_HOLD = 0x22,
	MODE2_RETURN = 0x23,
	MODE2_STOP = 0x24,
	MODE3_CONFIG = 0x30,
	MODE3_START = 0x31,
	MODE3_STOP = 0x32,
	ACK = 0x33,
	STREAM_DATA = 0x40
};

enum calibration_states
{
	RIGHT_RUN_FAST_CALIB = 0,
	RIGHT_RUN_SLOW_CALIB,
	LEFT_RUN_FAST_CALIB,
	LEFT_RUN_SLOW_CALIB,
	RIGHT_LOGIC,
	LEFT_LOGIC,
	END
};

uint8_t calib_state = 0;
uint8_t calib_state_prev = 0;
uint8_t non_limit_switch_motion = 0;
uint8_t motion_ctrl = 1;
uint8_t lside = 0;

mode1_config_t cfg;

void Mode1_Handle(const mode1_config_t* cfg)
{
    g_mode1_cfg = *cfg;
}

void Mode2_Handle(const mode2_config_t* cfg)
{
    g_mode2_cfg = *cfg;
}

void Torque_Handle(const torque_t* t)
{
    g_torque_cmd = *t;
}

void PID_Handle(const pid_t* pid)
{
    g_pid_cfg = *pid;
}

void Stream_Handle(const stream_data_t* data)
{
    g_stream_data = *data;
}




void packet_callback(uint8_t msg_type, uint8_t *payload, uint8_t size)
{
    switch(msg_type)
    {
        case 0x10: // MODE1_CONFIG
        {
        		regime = MODE1_CONFIG;
                memcpy(&cfg, payload, sizeof(cfg));
                mode1_start = 1;
                Mode1_Handle(&cfg);

            break;
        }

        case 0x20: // MODE2_CONFIG
        {
            if (size >= sizeof(mode2_config_t))
            {
                mode2_config_t cfg;
                memcpy(&cfg, payload, sizeof(cfg));
                Mode2_Handle(&cfg);
                uint8_t a = 0;
                uint8_t b = 0;
                uint8_t c = 0;
                uint8_t d = 0;
                set_velocity = g_mode2_cfg.speed;
                set_angle = g_mode2_cfg.start_angle;
                set_tenso_data = g_mode2_cfg.end_angle;
                d = g_mode2_cfg.hold_ms;// неправильное значение

            }
            break;
        }

        case 0x32: // TORQUE
        {
            if (size >= sizeof(torque_t))
            {
                torque_t cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                Torque_Handle(&cmd);
                uint8_t a = 0;
                uint8_t b = 0;
                uint8_t c = 0;
                a = g_torque_cmd.torque;
                b = g_torque_cmd.duration;//неправильное значение
                c = g_torque_cmd.direction;

            }
            break;
        }

        case 0x06: // PID CONFIG
        {
            if (size >= sizeof(pid_t))
            {
                pid_t pid;
                memcpy(&pid, payload, sizeof(pid));
                PID_Handle(&pid);

                uint8_t a = 0;
                uint8_t b = 0;
                uint8_t c = 0;
                a = g_pid_cfg.p;
                b = g_pid_cfg.i;
                c = g_pid_cfg.d;



            }
            break;
        }

        case 0x30: // STREAM
        {
            if (size >= sizeof(stream_data_t))//не заходит в if
            {
                stream_data_t data;
                memcpy(&data, payload, sizeof(data));
                Stream_Handle(&data);
                uint8_t a = 0;
                uint8_t b = 0;
                uint8_t c = 0;
                uint8_t d = 0;
                uint8_t e = 0;
                uint8_t f = 0;
                uint8_t g = 0;
                a = g_stream_data.ts_ms;
                b = g_stream_data.load;
                c = g_stream_data.pos;
                d = g_stream_data.angle1;
                e = g_stream_data.angle2;
                f = g_stream_data.temp;
                g = g_stream_data.flags;
                //HAL_Delay(10);
            }
            break;
        }

        default:
            break;
    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 //uint8_t data []= {1,1,1};
 //send_uart_packet(0x00, data, 3, &huart1);
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
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  //MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  parser_set_callback(packet_callback);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_SET);


  hx711_init(&hx711_handle, HX711_CLK_GPIO_Port, HX711_CLK_Pin, HX711_DOUT_GPIO_Port, HX711_DOUT_Pin);
  hx711_coef_set(&hx711_handle, 354.5); // read afer calibration
  hx711_tare(&hx711_handle, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim1);

  state = INITIALIZATION;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  g_stream_data.angle1 = angle;
	  g_stream_data.pos = (float)optical_count;
	  g_stream_data.load = (float)hx711_value(&hx711_handle);
	  g_stream_data.angle2 = (float)encoder_rpm;
	  g_stream_data.flags = 0;
	  g_stream_data.temp = 5;
	  g_stream_data.ts_ms = 6;
	  send_uart_packet(0x40, &g_stream_data, sizeof(g_stream_data), &huart1);
	  HAL_Delay(100);


	switch(state)
	{
		case(INITIALIZATION):
		{
			//Добавить ввод данных(Режим работы, скорость, усилие)
			//Добавить вывод всех данных
			//set_tenso_data = 0;
			//set_default_position = 0;
			//angle_velocity = 1;
			//set_angle = 0;
			//amplitude = 30000;
			//pull_number = 5;
			set_velocity = 50;
			regime = MODE1_CONFIG;
			Encoder_Init();
			Motor_Stop();
			state = MODE3_CONFIG;
			//state = CALIBRATION;
			prev_state = INITIALIZATION;
			break;
		}
		case(CALIBRATION):
		{
			int len;
			char buffer1[64];


			if (x1 < 1000000)
			{
				x1++;
			}
			else
			{
				int len2 = snprintf(buffer1, sizeof(buffer1), "Pos: %.2f, RPM: %.3f \n", angle, encoder_rpm);
				//len = snprintf(buffer1, sizeof(buffer1), "Pos: %ld\n", optical_count);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer1, len2, HAL_MAX_DELAY);
				x1 = 0;
			}
			switch(calib_state)
			{
			case(RIGHT_RUN_FAST_CALIB):
			{
				if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity, 1);
					motion_ctrl = 0;
				}
				if(limit_flag_1)
				{
					calib_state = RIGHT_LOGIC;
					calib_state_prev = RIGHT_RUN_FAST_CALIB;
					motion_ctrl = 1;
				}
				if(non_limit_switch_motion)
				{
					HAL_Delay(3000);
					//TODO: ПЕРЕДЕЛАТЬ НА ОПРОС ТЙМЕРА ПО ТИКАМ ДЛЯ ТОГО ЧТО БЫ НЕ БЛОКАТЬ РАБОТУ
					calib_state = LEFT_LOGIC;
					calib_state_prev = RIGHT_RUN_FAST_CALIB;
					motion_ctrl = 1;
					non_limit_switch_motion = 0;
				}
				break;
			}

			case(LEFT_RUN_FAST_CALIB):
			{
				if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity, -1);
					motion_ctrl = 0;
				}
				if(non_limit_switch_motion)
				{
					HAL_Delay(3000);
					//TODO: ПЕРЕДЕЛАТЬ НА ОПРОС ТЙМЕРА ПО ТИКАМ ДЛЯ ТОГО ЧТО БЫ НЕ БЛОКАТЬ РАБОТУ
					calib_state = RIGHT_LOGIC;
					calib_state_prev = LEFT_RUN_FAST_CALIB;
					motion_ctrl = 1;
					non_limit_switch_motion = 0;
				}
				if(limit_flag_2)
				{

					calib_state = LEFT_LOGIC;
					calib_state_prev = LEFT_RUN_FAST_CALIB;
					motion_ctrl = 1;
				}
				break;
			}

			case(RIGHT_RUN_SLOW_CALIB):
			{
				if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity/5, 1);
					motion_ctrl = 0;
				}
				if(limit_flag_1)
				{
					opt_lim_1 = optical_count;
					encoder_lim_1 = angle;
					calib_state = RIGHT_LOGIC;
					calib_state_prev = RIGHT_RUN_SLOW_CALIB;
					motion_ctrl = 1;
				}
				break;
			}

			case(LEFT_RUN_SLOW_CALIB):
			{
				if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity/5, -1);
					motion_ctrl = 0;
				}
				if(limit_flag_2)
				{
					opt_lim_2 = optical_count;
					encoder_lim_2 = angle;
					calib_state = LEFT_LOGIC;
					calib_state_prev = LEFT_RUN_SLOW_CALIB;
					motion_ctrl = 1;
				}
				break;
			}
			case (RIGHT_LOGIC):
			{
				limit_flag_1 = 0;
				limit_flag_2 = 0;
				if(calib_state_prev == RIGHT_RUN_FAST_CALIB)
				{
					calib_state = LEFT_RUN_FAST_CALIB;
					non_limit_switch_motion = 1;

				}
				else if(calib_state_prev == LEFT_RUN_FAST_CALIB)
				{
					calib_state = RIGHT_RUN_SLOW_CALIB;

				}
				else if(calib_state_prev == RIGHT_RUN_SLOW_CALIB)
				{
					calib_state = LEFT_RUN_FAST_CALIB;

				}
			break;
			}

			case(LEFT_LOGIC):
			{
				motion_ctrl = 1;
				limit_flag_1 = 0;
				limit_flag_2 = 0;
				if(calib_state_prev == LEFT_RUN_FAST_CALIB)
				{
					calib_state = RIGHT_RUN_FAST_CALIB;
					non_limit_switch_motion = 1;
				}
				else if(calib_state_prev == RIGHT_RUN_FAST_CALIB)
				{
					calib_state = LEFT_RUN_SLOW_CALIB;
				}
				else if(calib_state_prev == LEFT_RUN_SLOW_CALIB)
				{
					calib_state = END;
				}
				break;
			}

			case(END):
			{
				//state = ACK;
				//prev_state = CALIBRATION;
				//Motor_Move(set_current, set_velocity, 1);
				//HAL_Delay(5000);
				//if (optical_count < set_default_position)
				//{
				char buffer4[64];
				int len4 = snprintf(buffer4, sizeof(buffer4), "lim1: %.2f, lim2: %.2f \n", encoder_lim_1, encoder_lim_2);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer4, len4, HAL_MAX_DELAY);
					Motor_Stop();
					state = ACK;
				//}
				break;
			}
			}
			//двигаемся в сторону 1 концевика, когда дошли(limit_flag = 1), останавливаемся, записываем данные с линейки в opt_lim_1, так же в другую сторону.
			/*if(stop_count == 0)
			{
				Motor_Move(set_current, set_velocity, 1);
				stop_count++;
			}
			if(stop_count == 2)
			{
				Motor_Move(set_current, set_velocity, -1);
				z1 = 0;
				while(z1 < 1000000000/set_velocity)
				{
					__NOP();
					z1++;
				}
				Motor_Stop();
				Motor_Move(set_current, set_velocity/5, 1);
				stop_count++;
			}
			if(stop_count == 4)
			{
				if(opt_flag_1 == 0)
				{
					opt_lim_1 = optical_count;
					opt_flag_1++;
				}
				z1 = 0;

				Motor_Move(set_current, set_velocity, -1);
				stop_count++;

			}
			if(stop_count == 6)
			{
				Motor_Move(set_current, set_velocity, 1);
				z1 = 0;
				while(z1 < 1000000000/set_velocity)
				{
					__NOP();
					z1++;
				}
				Motor_Stop();
				Motor_Move(set_current, set_velocity/5, -1);
				stop_count++;
			}
			if(stop_count == 8)
			{
				if(opt_flag_2 == 0)
				{
					opt_lim_2 = optical_count;
					opt_flag_2++;
				}
				z1 = 0;
				while(z1 < 1000000000/set_velocity)
				{
					__NOP();
					z1++;
				}
				Motor_Move(set_current, set_velocity, 1);
			}
			if((position > 0) && (opt_flag_1 == 1) && (opt_flag_2 == 1))
			{
			    char buffer3[64];
			    int len3 = snprintf(buffer3, sizeof(buffer3), "Opt linear: %ld, %ld\n", opt_lim_1, opt_lim_2);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer3, len3, HAL_MAX_DELAY);
				Motor_Stop();
				state = ACK;
				prev_state = CALIBRATION;

			}*/


			break;
		}

		case(MODE1_CONFIG):
		{
			//min_angle = cfg.min_angle;
			//max_angle = cfg.max_angle;
			//pull_number = cfg.count;
			//angle_velocity = (cfg.max_angle - cfg.min_angle) * (2 *cfg.freq)  * 60;
			//direction = 1;
			//count = 0;
			//motion_ctrl = 1;








			min_angle = 90;
			max_angle = 150;
			set_default_position = optical_count;
			direction = 1;
			motion_ctrl = 1;
			angle_velocity_ref = 20;
			acc_coef = 5;
			pull_number = 10;

			state = MODE1_START;
			prev_state = MODE1_CONFIG;
			start_tick = HAL_GetTick();
			break;
		}
		case(MODE1_START):
		{




			if(angle < min_angle + acc_coef*(max_angle-min_angle)/100)
			{
				angle_velocity = angle_velocity_ref * (angle - min_angle) / (acc_coef*(max_angle-min_angle)/100);
			}
			if((angle > min_angle + acc_coef*(max_angle-min_angle)/100) && (angle < max_angle - acc_coef*(max_angle-min_angle)/100))
			{
				angle_velocity = angle_velocity_ref;
			}
			if(angle > max_angle - acc_coef*(max_angle-min_angle)/100)
			{
				angle_velocity = angle_velocity_ref * (max_angle - angle) / (acc_coef*(max_angle-min_angle)/100);
			}


			Motor_Move(set_current, angle_velocity + k_p * (angle_velocity - encoder_rpm) , direction);

			if((angle < max_angle) && (angle > min_angle))
			{
				motion_ctrl = 1;
			}

			if (angle >= max_angle)
			{
				if(motion_ctrl == 1)
				{
					direction = -1;
					count++;
					motion_ctrl = 0;
				}
			}

			if (angle <= min_angle)
			{
				if(motion_ctrl == 1)
				{
					direction = 1;
					count++;
					motion_ctrl = 0;
				}
			}
			/*
			if (optical_count * optical_count < amplitude * amplitude)
			{
				dir_ctrl = 1;
			}

			if((optical_count * optical_count > amplitude * amplitude) && (dir_ctrl == 1))
			{
				direction = -1 * direction;
				count++;
				motion_ctrl = 1;
				dir_ctrl = 0;
				start_tick = HAL_GetTick();
			}
			*/
			if(count >= pull_number * 2)
			{
					state = MODE1_STOP;
					prev_state = MODE1_START;
			}
			if (x1 < 100)
			{
				x1++;
			}
			else
			{
				char buffer3[64];
				int len3 = snprintf(buffer3, sizeof(buffer3), "Pos: %.2f, RPM: %.3f \n", angle, encoder_rpm);
				//len = snprintf(buffer1, sizeof(buffer1), "Pos: %ld\n", optical_count);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer3, len3, HAL_MAX_DELAY);
				x1 = 0;
			}
			break;
		}
		case(MODE1_STOP):
		{
			Motor_Stop();
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);

			break;
		}
		case(MODE2_CONFIG):
		{
			state = MODE2_START;
			prev_state = MODE2_CONFIG;
			break;
		}
		case(MODE2_START):
		{
			Motor_Move(set_current, set_velocity, 1);
			tenso_data = hx711_value(&hx711_handle);
			if (tenso_data > set_tenso_data)
			{
				Motor_Stop();
				state = MODE2_STOP;
				prev_state = MODE2_START;
			}
			break;
		}
		case(MODE2_HOLD):
		{
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);
			z1 = 0;
			while(z1 < 100000000)//выставить необходимую задержку
			{
				__NOP();
				z1++;
			}
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_SET);

			state = MODE2_STOP;//подумать куда
			prev_state = MODE2_STOP;
			break;
		}
		case(MODE2_RETURN):
		{
			Motor_Move(set_current, set_velocity, -1);//возвращаемся в нулевое положение
			if(optical_count > set_default_position)
			{
				Motor_Stop();
				state = MODE1_STOP;
				prev_state = MODE1_START;
			}
		}
		case(MODE2_STOP):
		{

		}

		case(MODE3_CONFIG):
		{
			t = 0;
			t0 = 0;
			v0 = 10;
			v1 = 1500;
			time1 = 1;
			t0 = HAL_GetTick();

			state = MODE3_START;
			prev_state = MODE3_CONFIG;
			break;

		}
		case(MODE3_START):
		{
			t = HAL_GetTick();
			if ((t - t0)/1000 < time1)
			{
				Motor_Move(set_current, (float)v0 + (float)(t - t0)/1000 * (float)(v1 - v0)/time1, 1);
			}
			else
			{
				state = MODE3_STOP;
				prev_state = MODE3_START;
				Motor_Stop();
			}

			break;
		}

		case(MODE3_STOP):
		{
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);
			break;
		}
		case(ACK):
		{
			Motor_Stop();
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);
			if(mode1_start == 1)
			{
				state = MODE1_CONFIG;
				prev_state = ACK;
			}
			break;
		}
		case(STREAM_DATA):
		{

			break;
		}
	}



	  if(new_data)
	  {





	      //long int optical_count_buf = optical_count;
	      //char buffer3[64];
	      //int len3 = snprintf(buffer3, sizeof(buffer3), "Opt linear: %ld\n", optical_count_buf);
		  //HAL_UART_Transmit(&huart1, (uint8_t*)buffer3, len3, HAL_MAX_DELAY);


	      //memcpy(tx_buffer + 1 + sizeof(uint32_t), &optical_count_buf, sizeof(long int));
	      //HAL_UART_Transmit(&huart1, tx_buffer, UART_TX_BUF_LEN, HAL_MAX_DELAY);


		  //HAL_Delay(1000);
		  //uint32_t tenso_data = (uint32_t) hx711_value(&hx711_handle);
		  //memcpy(tx_buffer + 1, &tenso_data, sizeof(uint32_t));
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t *) tx_buffer, sizeof(tx_buffer));
		  //HAL_Delay(1000);
/*
		  Encoder_UpdateRPM();
		  char buffer12[64];
		  char buffer22[64];
		  int len2 = snprintf(buffer12, sizeof(buffer12), "Pos: %ld, RPM: %.2f\r\n", encoder_position, encoder_rpm);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer12, len2, HAL_MAX_DELAY);
		  int len12 = snprintf(buffer22, sizeof(buffer22), "Motor_Pos: %ld, Motor_RPM: %.2f\r\n", motor_encoder_position, motor_encoder_rpm);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer22, len12, HAL_MAX_DELAY);
		  HAL_Delay(100);
		  new_data = false;*/

		  //Encoder_UpdateRPM();
		  /*char buffer1[64];
		  char buffer2[64];
		  int len = snprintf(buffer1, sizeof(buffer1), "Pos: %ld, RPM: %.2f\r\n", encoder_position, encoder_rpm);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer1, len, HAL_MAX_DELAY);
		  int len1 = snprintf(buffer2, sizeof(buffer2), "Motor_Pos: %ld, Motor_RPM: %.2f\r\n", motor_encoder_position, motor_encoder_rpm);
		  HAL_UART_Transmit(&huart1, (uint8_t*)buffer2, len1, HAL_MAX_DELAY);
		  //HAL_Delay(100);
		  new_data = false;*/
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
