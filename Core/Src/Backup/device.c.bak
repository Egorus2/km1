#include <device.h>
#include <stdbool.h>
#include <string.h>
#include "mb.h"
#include "mbport.h"

#define ENCODER_PPR 1000
#define TIMER_PERIOD_MS 100

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
    uint32_t current_tick = HAL_GetTick();
    int32_t delta_pos = encoder_position - last_position;

    // RPM = (Δимпульсы * 60000) / (PPR * Δt(ms))
    encoder_rpm = (delta_pos * 60000.0f) / (ENCODER_PPR * (current_tick - last_tick));

    last_position = encoder_position;
    last_tick = current_tick;

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

    HAL_UART_Transmit(&huart3, tx_data, 8, 1000);
    HAL_Delay(10);
}

void Motor_Move(int current, int velocity, int distance, int direction)
{
	modbus_send_simple(1, 0x0191, current);    // Ток
	modbus_send_simple(1, 0x01E1, velocity);   // Скорость
	modbus_send_simple(1, 0x0202, distance & 0xFFFF); // Расстояние
	modbus_send_simple(1, 0x0203, (distance >> 16) & 0xFFFF); // Расстояние
	HAL_Delay(100);
	if (direction == 1)
	{
		modbus_send_simple(1, 0x1801, 0x4001); // Вперед
	}
	if (direction == -1)
	{
		modbus_send_simple(1, 0x1801, 0x4002); // Вперед
	}
	HAL_Delay(3000);

}
