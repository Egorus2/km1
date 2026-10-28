#include <device.h>
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "mb.h"
#include "mbport.h"
#include "hx711.h"

#define ENCODER_PPR 4000
#define TIMER_PERIOD_MS 100
#define RPM_WINDOW 5
#define TENSO_AVG_WINDOW 5
#define TENSO_DEFAULT_ZERO_RAW 8476210.0f
#define TENSO_FIT_A -1.12968903e-9f
#define TENSO_FIT_B 3.92968595e-4f
#define TENSO_FIT_C 0.1156123f

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
volatile bool encoder_calibrated = false;
extern volatile float encoder_lim_1;
extern volatile int state;

static int32_t rpm_position_buffer[RPM_WINDOW];
static uint32_t rpm_tick_buffer[RPM_WINDOW];
static uint8_t rpm_buffer_index;
static uint8_t rpm_sample_count;
static int32_t motor_command_velocity;
static HAL_StatusTypeDef motor_last_tx_status = HAL_OK;

//Основные переменные характеризующие состояние стенда
volatile int opt_lim_1;// пределы шкалы оптической линейки
volatile int opt_lim_2;
volatile bool limit_flag_1 = 0;//флаги срабатывания концевиков
volatile bool limit_flag_2 = 0;

float tenso_data;//текущая нагрузка
float set_tenso_data;//заданная требуемая нагрузка
static float tenso_avg_buffer[TENSO_AVG_WINDOW];
static uint8_t tenso_avg_index = 0;
static uint8_t tenso_avg_count = 0;
static float tenso_avg_sum = 0.0f;
static float tenso_zero_raw = TENSO_DEFAULT_ZERO_RAW;
static float tenso_zero_kg_offset = TENSO_FIT_C;

float velocity;//текущая скорость
float set_velocity;//заданная тремуемая скорость

float position;//текущая позиция
float set_default_position;//заданное нулевое положение

float angle; //текущий угол
float set_angle; //заданное значение угла

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void Encoder_ResetRPMFilter(void)
{
    uint32_t now = HAL_GetTick();
    int32_t current_position = encoder_position;
    rpm_position_buffer[0] = current_position;
    rpm_tick_buffer[0] = now;
    rpm_buffer_index = 1;
    rpm_sample_count = 1;
    encoder_rpm = 0.0f;
    last_position = current_position;
    last_tick = now;
}

void Encoder_Init(void)
{
    encoder_position = 0;
    last_position = 0;
    encoder_rpm = 0.0f;
    encoder_calibrated = false;
    Encoder_ResetRPMFilter();
    last_tick = HAL_GetTick();
    motor_encoder_position = 0;
    motor_last_position = 0;
    motor_encoder_rpm = 0.0f;
    motor_last_tick = HAL_GetTick();
}

void Tenso_ResetAverage(void)
{
    memset(tenso_avg_buffer, 0, sizeof(tenso_avg_buffer));
    tenso_avg_index = 0;
    tenso_avg_count = 0;
    tenso_avg_sum = 0.0f;
    tenso_data = 0.0f;
}

float Tenso_UpdateAverage(void)
{
    float sample = (float)hx711_value(&hx711_handle);

    if (tenso_avg_count < TENSO_AVG_WINDOW)
    {
        tenso_avg_count++;
    }
    else
    {
        tenso_avg_sum -= tenso_avg_buffer[tenso_avg_index];
    }

    tenso_avg_buffer[tenso_avg_index] = sample;
    tenso_avg_sum += sample;

    tenso_avg_index++;
    if (tenso_avg_index >= TENSO_AVG_WINDOW)
    {
        tenso_avg_index = 0;
    }

    tenso_data = tenso_avg_sum / (float)tenso_avg_count;
    return tenso_data;
}

static float Tenso_CalcKgFromRaw(float raw_tenso_data)
{
	float dx = raw_tenso_data - TENSO_DEFAULT_ZERO_RAW;

	return TENSO_FIT_A * dx * dx + TENSO_FIT_B * dx + TENSO_FIT_C;
}

void Tenso_CalibrateZero(void)
{
	if (tenso_avg_count == 0)
	{
		Tenso_UpdateAverage();
	}

	tenso_zero_raw = tenso_data;
	tenso_zero_kg_offset = Tenso_CalcKgFromRaw(tenso_zero_raw);
	Tenso_ResetAverage();
}

float Tenso_in_kg(float raw_tenso_data)
{

	return Tenso_CalcKgFromRaw(raw_tenso_data) - tenso_zero_kg_offset;
}

void Encoder_UpdateRPM(void)
{
    uint32_t now = HAL_GetTick();
    int32_t current_position = encoder_position;
    float raw_angle = -360.0f * (float)current_position / ENCODER_PPR;
    angle = encoder_calibrated ? raw_angle - encoder_lim_1 + 180.0f : raw_angle;

    /* Read the oldest sample before overwriting it. Use its actual timestamp:
     * the old implementation divided four sample intervals by five. */
    uint8_t oldest = (rpm_sample_count < RPM_WINDOW) ? 0 : rpm_buffer_index;
    uint32_t elapsed_ms = now - rpm_tick_buffer[oldest];
    if (rpm_sample_count > 0 && elapsed_ms > 0U)
    {
        int64_t delta = (int64_t)current_position - rpm_position_buffer[oldest];
        encoder_rpm = -(float)delta * 60000.0f / ((float)ENCODER_PPR * elapsed_ms);
    }
    if (now != last_tick || rpm_sample_count == 0)
    {
        rpm_position_buffer[rpm_buffer_index] = current_position;
        rpm_tick_buffer[rpm_buffer_index] = now;
        rpm_buffer_index = (rpm_buffer_index + 1U) % RPM_WINDOW;
        if (rpm_sample_count < RPM_WINDOW)
        {
            rpm_sample_count++;
        }
    }
    last_position = current_position;
    last_tick = now;

    int32_t current_motor_position = motor_encoder_position;
    uint32_t motor_elapsed_ms = now - motor_last_tick;
    if (motor_elapsed_ms > 0U)
    {
        int64_t delta = (int64_t)current_motor_position - motor_last_position;
        motor_encoder_rpm = (float)delta * 60000.0f / ((float)ENCODER_PPR * motor_elapsed_ms);
        motor_last_position = current_motor_position;
        motor_last_tick = now;
    }
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

    motor_last_tx_status = HAL_UART_Transmit_IT(&huart2, tx_data, 8);
    HAL_Delay(10);
}

void Motor_Move(int current, int velocity, int direction)
{
      motor_command_velocity = velocity * direction;
	  HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_SET);
	  HAL_Delay(20);
	  modbus_send_simple(1, 0x0191, current);
	  //modbus_send_simple(1, 0x2001, current);
	  modbus_send_simple(1, 0x6200, 0x0002);
	  modbus_send_simple(1, 0x6203, velocity*direction);
	  modbus_send_simple(1, 0x6002, 0x0010);
}
void Motor_Stop()
{
    motor_command_velocity = 0;
	modbus_send_simple(1, 0x6002, 0x0040);
}

int32_t Motor_GetCommandVelocity(void)
{
    return motor_command_velocity;
}

HAL_StatusTypeDef Motor_GetLastTxStatus(void)
{
    return motor_last_tx_status;
}

void Sensor_Poll()
{

	Encoder_UpdateRPM();
	tenso_data = Tenso_UpdateAverage();
	position = optical_count;
	velocity = motor_encoder_rpm;
}
