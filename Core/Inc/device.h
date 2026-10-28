#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "stm32g4xx_hal.h"

void Encoder_Init(void);
void Encoder_UpdateRPM(void);
void Encoder_ResetRPMFilter(void);
int32_t Encoder_GetPosition(void);
int32_t Encoder_GetRPM(void);
int32_t Motor_Encoder_GetPosition(void);
int32_t Motor_Encoder_GetRPM(void);

uint16_t CRC16(uint8_t *data, uint16_t length);
void Motor_Move(int current, int velocity, int direction);
void Motor_Stop();
int32_t Motor_GetCommandVelocity(void);
HAL_StatusTypeDef Motor_GetLastTxStatus(void);
void Tenso_ResetAverage(void);
void Tenso_CalibrateZero(void);
float Tenso_in_kg(float raw_tenso_data);
float Tenso_UpdateAverage(void);
void Sensor_Poll();

#endif
