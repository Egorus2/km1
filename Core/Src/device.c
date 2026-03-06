#include <device.h>
#include <stdbool.h>
#include <string.h>
#include "mb.h"
#include "mbport.h"
#include "hx711.h"

#define ENCODER_PPR 4000
#define TIMER_PERIOD_MS 100
#define RPM_WINDOW 5

volatile hx711_t hx711_handle;
volatile long int optical_count;
volatile bool new_data = 0;
volatile int32_t encoder_position = 0;  // текущая позиция энкодера
volatile int32_t last_position = 0;     // позиция на предыдущем измерении
volatile float encoder_rpm = 0.0f;      // скорость вращения
volatile uint32_t last_tick = 0;
volatile int32_t motor_encoder_position = 0;  // текущая позиция энкодера
volatile int32_t motor_last_position = 0;     // позиция на предыдущем измерении
volatile float motor_encoder_rpm = 0.0f;      // скорость вращения
volatile uint32_t motor_last_tick = 0;


float angle_buffer[RPM_WINDOW];
int buffer_index = 0;
bool buffer_filled = false;
float sum_ticks;
float sum_time;

//Основные переменные характеризующие состояние стенда
volatile int opt_lim_1;// пределы шкалы оптической линейки
volatile int opt_lim_2;
volatile bool limit_flag_1 = 0;//флаги срабатывания концевиков
volatile bool limit_flag_2 = 0;

float tenso_data;//текущая нагрузка
float set_tenso_data;//заданная требуемая нагрузка

float velocity;//текущая скорость
float set_velocity;//заданная тремуемая скорость

float position;//текущая позиция
float set_default_position;//заданное нулевое положение

float angle; //текущий угол
float set_angle; //заданное значение угла

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

void Encoder_Init(void)
{
    encoder_position = 0;
    last_position = 0;
    encoder_rpm = 0.0f;
    last_tick = HAL_GetTick();
    motor_encoder_position = 0;
    motor_last_position = 0;
    motor_encoder_rpm = 0.0f;
    motor_last_tick = HAL_GetTick();
}

void Encoder_UpdateRPM(void)
{

    //float delta_pos = 360*((float)encoder_position - (float)last_position)/4000;
    float delta_time_ms = 100;

    angle = 360*(float)encoder_position/4000;

    //encoder_rpm = 60 * 1000 * (angle - last_position) / (delta_time_ms) / 360;

    angle_buffer[buffer_index] = angle;
    buffer_index++;

    if (buffer_index >= RPM_WINDOW) {
        buffer_index = 0;
        buffer_filled = true;
    }

    if (buffer_filled) {
        float angle_old = angle_buffer[buffer_index];

        float delta_angle = angle - angle_old;
        float total_time = delta_time_ms * RPM_WINDOW;

        encoder_rpm = 60.0f * 1000.0f * delta_angle / total_time / 360.0f;
    }



    uint32_t motor_current_tick = HAL_GetTick();
    int32_t motor_delta_pos = motor_encoder_position - motor_last_position;

    // RPM = (Δимпульсы * 60000) / (PPR * Δt(ms))
    motor_encoder_rpm = (motor_delta_pos * 60000.0f) / (ENCODER_PPR * (motor_current_tick - motor_last_tick));

    motor_last_position = motor_encoder_position;
    motor_last_tick = motor_current_tick;
}

int32_t Encoder_GetPosition(void)
{
    return encoder_position;
}

int32_t Encoder_GetRPM(void)
{
    return encoder_rpm;
}

int32_t Motor_Encoder_GetPosition(void)
{
    return motor_encoder_position;
}

int32_t Motor_Encoder_GetRPM(void)
{
    return motor_encoder_rpm;
}

/* Функция расчета CRC16 для Modbus */
uint16_t CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t j = 0; j < length; j++) {
        crc ^= data[j];
        for (uint16_t i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}



void modbus_send_simple(uint8_t slave_id, uint16_t reg_addr, uint16_t value)
{
    uint8_t tx_data[8];
    uint16_t crc;

    tx_data[0] = slave_id;
    tx_data[1] = 0x06;
    tx_data[2] = (reg_addr >> 8);
    tx_data[3] = (reg_addr & 0xFF);
    tx_data[4] = (value >> 8);
    tx_data[5] = (value & 0xFF);

    crc = CRC16(tx_data, 6);
    tx_data[6] = (crc & 0xFF);
    tx_data[7] = (crc >> 8);

    HAL_UART_Transmit_IT(&huart3, tx_data, 8);
    HAL_Delay(10);
}

void Motor_Move(int current, int velocity, int direction)
{
	  modbus_send_simple(1, 0x0191, current);
	  //modbus_send_simple(1, 0x2001, current);
	  modbus_send_simple(1, 0x6200, 0x0002);
	  modbus_send_simple(1, 0x6203, velocity*direction);
	  modbus_send_simple(1, 0x6002, 0x0010);
}
void Motor_Stop()
{
	modbus_send_simple(1, 0x6002, 0x0040);
}

void Sensor_Poll()
{

	Encoder_UpdateRPM();
	tenso_data = hx711_value(&hx711_handle);
	position = optical_count;
	angle = encoder_position;
	velocity = motor_encoder_rpm;
}
