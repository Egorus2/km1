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
#include <math.h>
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
#define STREAM_PERIOD_MS 100U
#define MODE1_SEEK_MIN_SPEED 50
#define MODE1_SEEK_MIN_OFFSET_DEG 1.0f
#define MODE1_SEEK_MIN_TOL_DEG 0.5f
#define MODE1_FREQ_RPM_SCALE 1.0f
#define MODE1_PID_DEFAULT_P 30.0f
#define MODE1_PID_DEFAULT_I 1.0f
#define MODE1_PID_DEFAULT_D 5.0f// 0.0f
#define MODE1_PID_BASE_SPEED 50.0f
#define MODE1_MOTOR_COMMAND_SCALE 1.10f
#define MODE1_MOTOR_SPEED_MIN 1.0f
#define MODE1_MOTOR_SPEED_MAX 1500.0f
#define MODE1_PID_INTEGRAL_LIMIT 100.0f
/* Each ramp covers 20% of the angular stroke. */
#define MODE1_RAMP_FRACTION 0.20f
#define MODE1_CREEP_RPM_RATIO 0.05f
#define MODE2_ANGLE_TOL_DEG 0.5f
#define MODE2_DEFAULT_SPEED 50.0f
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
extern volatile bool encoder_calibrated;
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
float mode1_pid_integral = 0.0f;
float mode1_pid_prev_measurement = 0.0f;
uint32_t mode1_pid_last_tick = 0;
float mode1_profile_rpm = 0.0f;
uint32_t mode1_profile_last_tick = 0;

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
    uint32_t cycles;
    /* Optional diagnostics appended after the compatible 30-byte prefix. */
    float target_deg_s;
    float motor_rpm;
    int32_t motor_command;
    uint16_t control_flags; /* bit0: Mode1 tracking, bit1: speed limit,
                              bit2: UART busy, bit3: UART error/timeout */
} stream_data_t;

typedef struct {
    float p;
    float i;
    float d;
} pid_t;


#pragma pack(pop)

_Static_assert(sizeof(stream_data_t) == 44U, "Telemetry layout must match the desktop decoder");


mode1_config_t g_mode1_cfg;
mode2_config_t g_mode2_cfg;
torque_t       g_torque_cmd;
pid_t          g_pid_cfg = {MODE1_PID_DEFAULT_P, MODE1_PID_DEFAULT_I, MODE1_PID_DEFAULT_D};
stream_data_t  g_stream_data;
volatile int requested_mode_after_calib = 0;
uint32_t last_stream_tick = 0;


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

enum mode2_phases
{
	MODE2_PHASE_IDLE = 0,
	MODE2_PHASE_SEEK_START_THEN_END,
	MODE2_PHASE_MOVE_TO_END,
	MODE2_PHASE_MOVE_TO_START
};

uint8_t calib_state = 0;
uint8_t calib_state_prev = 0;
uint8_t non_limit_switch_motion = 0;
uint8_t motion_ctrl = 1;
uint8_t lside = 0;
uint8_t mode1_seek_min = 0;
uint8_t mode2_configured = 0;
uint8_t mode2_phase = MODE2_PHASE_IDLE;
uint8_t mode2_next_target_is_end = 0;
uint8_t mode2_first_run = 0;
volatile uint8_t mode2_stop_requested = 0;
volatile uint8_t tenso_zero_requested = 0;
volatile uint8_t emergency_stop_requested = 0;
volatile uint8_t emergency_stop_reason = STOP_REASON_NONE;
volatile int8_t motor_motion_direction = 0;
float mode2_target_angle = 0.0f;

mode1_config_t cfg;

static float Mode1_AbsFloat(float value);
static void Mode1_ResetPid(void);
static int Mode1_GetPidMotorSpeed(float target_encoder_rpm);
static void Mode1_ResetProfile(void);
static float Mode1_GetProfileSpeed(void);
static float ClampAngleToCalibratedRange(float target);
static void EmergencyStopNow(uint8_t reason);
static uint8_t SafeMotorMove(int current, int velocity, int direction);
static void Mode2_ResetToStartState(void);
static void Mode2_StopAndResetToStart(void);
static void Mode2_BeginNextMove(void);

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
    if ((g_pid_cfg.p == 0.0f) && (g_pid_cfg.i == 0.0f) && (g_pid_cfg.d == 0.0f))
    {
        g_pid_cfg.p = MODE1_PID_DEFAULT_P;
        g_pid_cfg.i = MODE1_PID_DEFAULT_I;
        g_pid_cfg.d = MODE1_PID_DEFAULT_D;
    }
    Mode1_ResetPid();
}

void Stream_Handle(const stream_data_t* data)
{
    g_stream_data = *data;
}

static float Mode1_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Mode1_ResetPid(void)
{
    mode1_pid_integral = 0.0f;
    mode1_pid_prev_measurement = Mode1_AbsFloat(encoder_rpm);
    mode1_pid_last_tick = HAL_GetTick();
}

/* The square-root braking envelope gives constant angular deceleration.
 * Time-limited acceleration also works when starting inside the stroke. */
static void Mode1_ResetProfile(void)
{
    mode1_profile_rpm = 0.0f;
    mode1_profile_last_tick = HAL_GetTick();
}

static float Mode1_GetProfileSpeed(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - mode1_profile_last_tick) / 1000.0f;
    mode1_profile_last_tick = now;
    if (dt > 0.2f)
    {
        dt = 0.2f;
    }

    float ramp_degrees = (max_angle - min_angle) * MODE1_RAMP_FRACTION;
    if (ramp_degrees <= 0.0f || angle_velocity_ref <= 0.0f)
    {
        mode1_profile_rpm = 0.0f;
        return 0.0f;
    }

    /* RPM * 6 = deg/s. v^2 = 2*a*s. */
    float acceleration_rpm_s = 3.0f * angle_velocity_ref * angle_velocity_ref / ramp_degrees;
    float remaining_degrees = (direction > 0) ? max_angle - angle : angle - min_angle;
    if (remaining_degrees < 0.0f)
    {
        remaining_degrees = 0.0f;
    }
    float braking_rpm = sqrtf(acceleration_rpm_s * remaining_degrees / 3.0f);
    /* A small final approach avoids stopping short due to encoder quantization. */
    float creep_rpm = angle_velocity_ref * MODE1_CREEP_RPM_RATIO;
    if (braking_rpm < creep_rpm)
    {
        braking_rpm = creep_rpm;
    }
    if (braking_rpm > angle_velocity_ref)
    {
        braking_rpm = angle_velocity_ref;
    }

    mode1_profile_rpm += acceleration_rpm_s * dt;
    if (mode1_profile_rpm > braking_rpm)
    {
        mode1_profile_rpm = braking_rpm;
    }
    return mode1_profile_rpm;
}

static int Mode1_GetPidMotorSpeed(float target_encoder_rpm)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - mode1_pid_last_tick) / 1000.0f;
    if (dt < 0.001f)
    {
        dt = 0.001f;
    }
    else if (dt > 0.2f)
    {
        dt = 0.2f;
    }
    mode1_pid_last_tick = now;

    if (target_encoder_rpm < 0.0f)
    {
        target_encoder_rpm = 0.0f;
    }

    float measured_encoder_rpm = Mode1_AbsFloat(encoder_rpm);
    float error = target_encoder_rpm - measured_encoder_rpm;

    float previous_integral = mode1_pid_integral;
    mode1_pid_integral += error * dt;
    if (mode1_pid_integral > MODE1_PID_INTEGRAL_LIMIT)
    {
        mode1_pid_integral = MODE1_PID_INTEGRAL_LIMIT;
    }
    else if (mode1_pid_integral < -MODE1_PID_INTEGRAL_LIMIT)
    {
        mode1_pid_integral = -MODE1_PID_INTEGRAL_LIMIT;
    }

    /* Differentiate the measurement to avoid a kick as the profile changes. */
    float derivative = -(measured_encoder_rpm - mode1_pid_prev_measurement) / dt;
    mode1_pid_prev_measurement = measured_encoder_rpm;

    float target_scale = 1.0f;
    if (angle_velocity_ref > 0.001f)
    {
        target_scale = target_encoder_rpm / angle_velocity_ref;
    }
    if (target_scale < 0.0f)
    {
        target_scale = 0.0f;
    }
    else if (target_scale > 1.0f)
    {
        target_scale = 1.0f;
    }

    float motor_speed = MODE1_PID_BASE_SPEED * target_scale
                      + g_pid_cfg.p * error
                      + g_pid_cfg.i * mode1_pid_integral
                      + g_pid_cfg.d * derivative;

    motor_speed *= MODE1_MOTOR_COMMAND_SCALE;

    if (motor_speed < MODE1_MOTOR_SPEED_MIN)
    {
        motor_speed = MODE1_MOTOR_SPEED_MIN;
        if (error < 0.0f)
        {
            mode1_pid_integral = previous_integral;
        }
    }
    else if (motor_speed > MODE1_MOTOR_SPEED_MAX)
    {
        motor_speed = MODE1_MOTOR_SPEED_MAX;
        if (error > 0.0f)
        {
            mode1_pid_integral = previous_integral;
        }
    }

    return (int)motor_speed;
}

static float ClampAngleToCalibratedRange(float target)
{
    if (!encoder_calibrated)
    {
        return target;
    }

    float mech_min = 180.0f;
    float mech_max = encoder_lim_2 - encoder_lim_1 + 180.0f;
    if (mech_min > mech_max)
    {
        float tmp = mech_min;
        mech_min = mech_max;
        mech_max = tmp;
    }
    if (target < mech_min)
    {
        return mech_min;
    }
    if (target > mech_max)
    {
        return mech_max;
    }
    return target;
}

static void Mode2_ResetToStartState(void)
{
    mode2_phase = MODE2_PHASE_IDLE;
    mode2_next_target_is_end = 0;
    mode2_first_run = 0;
    state = ACK;
    prev_state = MODE2_STOP;
}

static void Mode2_StopAndResetToStart(void)
{
    Motor_Stop();
    Mode2_ResetToStartState();
}

static void Mode2_BeginNextMove(void)
{
    if (!mode2_configured || !encoder_calibrated)
    {
        mode2_stop_requested = 1;
        Mode2_ResetToStartState();
        return;
    }

    set_velocity = g_mode2_cfg.speed;
    if (set_velocity <= 0.0f)
    {
        set_velocity = MODE2_DEFAULT_SPEED;
        g_mode2_cfg.speed = MODE2_DEFAULT_SPEED;
    }

    g_mode2_cfg.start_angle = ClampAngleToCalibratedRange(g_mode2_cfg.start_angle);
    g_mode2_cfg.end_angle = ClampAngleToCalibratedRange(g_mode2_cfg.end_angle);

    if (mode2_first_run)
    {
        mode2_phase = MODE2_PHASE_SEEK_START_THEN_END;
        mode2_target_angle = g_mode2_cfg.start_angle;
    }
    else if (mode2_next_target_is_end)
    {
        mode2_phase = MODE2_PHASE_MOVE_TO_END;
        mode2_target_angle = g_mode2_cfg.end_angle;
    }
    else
    {
        mode2_phase = MODE2_PHASE_MOVE_TO_START;
        mode2_target_angle = g_mode2_cfg.start_angle;
    }

    state = MODE2_START;
    prev_state = ACK;
}

void UpdateLimitFlagsFromPins(void)
{
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        limit_flag_1 = 1;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
    {
        limit_flag_2 = 1;
    }
}

static void EmergencyStopNow(uint8_t reason)
{
    emergency_stop_requested = 0;
    emergency_stop_reason = reason;

    mode1_seek_min = 0;
    Mode1_ResetPid();
    motor_motion_direction = 0;
    mode2_stop_requested = 0;
    mode2_phase = MODE2_PHASE_IDLE;
    mode2_next_target_is_end = 0;
    mode2_first_run = 0;

    Motor_Stop();
    HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);

    prev_state = state;
    state = ACK;
}

static uint8_t SafeMotorMove(int current, int velocity, int direction)
{
    int8_t requested_direction = 0;
    uint8_t limit1_active = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET);
    uint8_t limit2_active = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);

    if (limit1_active)
    {
        limit_flag_1 = 1;
    }
    if (limit2_active)
    {
        limit_flag_2 = 1;
    }

    if (direction > 0)
    {
        requested_direction = 1;
    }
    else if (direction < 0)
    {
        requested_direction = -1;
    }

    if ((requested_direction > 0) && limit1_active)
    {
        EmergencyStopNow(STOP_REASON_LIMIT_1);
        return 0;
    }
    if ((requested_direction < 0) && limit2_active)
    {
        EmergencyStopNow(STOP_REASON_LIMIT_2);
        return 0;
    }

    motor_motion_direction = requested_direction;
    Motor_Move(current, velocity, direction);
    return 1;
}

void StartCalibrationForMode(int target_mode)
{
    requested_mode_after_calib = target_mode;
    regime = target_mode;
    mode1_start = 0;
    mode1_seek_min = 0;
    motor_motion_direction = 0;
    encoder_calibrated = false;
    Encoder_ResetRPMFilter();
    calib_state = RIGHT_RUN_FAST_CALIB;
    calib_state_prev = RIGHT_RUN_FAST_CALIB;
    non_limit_switch_motion = 0;
    motion_ctrl = 1;
    limit_flag_1 = 0;
    limit_flag_2 = 0;
    prev_state = ACK;
    UpdateLimitFlagsFromPins();
    if (limit_flag_1 && !limit_flag_2)
    {
        calib_state = LEFT_RUN_FAST_CALIB;
        calib_state_prev = RIGHT_RUN_FAST_CALIB;
        non_limit_switch_motion = 1;
    }
    else if (limit_flag_2 && !limit_flag_1)
    {
        calib_state = RIGHT_RUN_FAST_CALIB;
        calib_state_prev = LEFT_RUN_FAST_CALIB;
        non_limit_switch_motion = 1;
    }
    state = CALIBRATION;
}




void packet_callback(uint8_t msg_type, uint8_t *payload, uint8_t size)
{
    switch(msg_type)
    {
        case 0x01: // CALIBRATION
        {
            StartCalibrationForMode(ACK);
            break;
        }

        case 0x10: // MODE1_CONFIG
        {
            if (size >= sizeof(mode1_config_t))
            {
                memcpy(&cfg, payload, sizeof(cfg));
                Mode1_Handle(&cfg);
                requested_mode_after_calib = 0;
                state = ACK;
                prev_state = MODE1_CONFIG;
            }

            break;
        }

        case 0x20: // MODE2_CONFIG
        {
            if (size >= sizeof(mode2_config_t))
            {
                mode2_config_t cfg;
                memcpy(&cfg, payload, sizeof(cfg));
                Mode2_Handle(&cfg);
                if (g_mode2_cfg.speed <= 0.0f)
                {
                    g_mode2_cfg.speed = MODE2_DEFAULT_SPEED;
                }
                set_velocity = g_mode2_cfg.speed;
                set_angle = g_mode2_cfg.start_angle;
                set_tenso_data = g_mode2_cfg.end_angle;
                mode2_configured = 1;
                mode2_phase = MODE2_PHASE_IDLE;
                mode2_next_target_is_end = 0;
                mode2_first_run = 1;
                requested_mode_after_calib = 0;
                state = ACK;
                prev_state = MODE2_CONFIG;
            }
            break;
        }

        case 0x11: // MODE1_START
        {
            if (encoder_calibrated)
            {
                state = MODE1_CONFIG;
            }
            else
            {
                Motor_Stop();
                state = ACK;
            }
            prev_state = ACK;
            break;
        }

        case 0x12: // MODE1_STOP
        {
            requested_mode_after_calib = 0;
            mode1_start = 0;
            mode1_seek_min = 0;
            Mode1_ResetPid();
            state = ACK;
            prev_state = MODE1_STOP;
            break;
        }

        case 0x21: // MODE2_START
        {
            Mode2_BeginNextMove();
            break;
        }

        case 0x22: // MODE2_STOP from UI
        case 0x24: // MODE2_STOP in firmware enum
        {
            requested_mode_after_calib = 0;
            mode2_stop_requested = 1;
            Mode2_ResetToStartState();
            break;
        }

        case 0x07: // TENSO_ZERO
        {
            tenso_zero_requested = 1;
            break;
        }

        case 0x32: // TORQUE
        {
            if (size >= sizeof(torque_t))
            {
                torque_t cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                Torque_Handle(&cmd);
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  parser_set_callback(packet_callback);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_SET);


  hx711_init(&hx711_handle, HX711_CLK_GPIO_Port, HX711_CLK_Pin, HX711_DOUT_GPIO_Port, HX711_DOUT_Pin);
  hx711_coef_set(&hx711_handle, 354.5); // read afer calibration
  hx711_tare(&hx711_handle, 10);
  Tenso_ResetAverage();

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

	  /* механическая проверка энкодера

	  int32_t ticks = Encoder_GetPosition();
	  uint8_t enc_a = HAL_GPIO_ReadPin(ENC_A_ITR0_GPIO_Port, ENC_A_ITR0_Pin);
	  uint8_t enc_b = HAL_GPIO_ReadPin(ENC_B_ITR1_GPIO_Port, ENC_B_ITR1_Pin);
	  float angle_deg = -360.0f * (float)ticks / 4000.0f;
	  char angle_buffer[96];
	  int angle_len = snprintf(angle_buffer, sizeof(angle_buffer),
	                           "Ticks:%ld A:%u B:%u Angle:%.3f\r\n",
	                           ticks, enc_a, enc_b, angle_deg);
	  HAL_UART_Transmit(&huart1, (uint8_t*)angle_buffer, angle_len, HAL_MAX_DELAY);
	  HAL_Delay(1000);

	  */

	  if (emergency_stop_requested)
	  {
		  EmergencyStopNow(emergency_stop_reason);
	  }

	  if (mode2_stop_requested)
	  {
		  mode2_stop_requested = 0;
		  Mode2_StopAndResetToStart();
	  }

	  if (tenso_zero_requested)
	  {
		  tenso_zero_requested = 0;
		  Tenso_CalibrateZero();
	  }

	  uint32_t now = HAL_GetTick();
	  if ((now - last_stream_tick) >= STREAM_PERIOD_MS)
	  {
		  last_stream_tick = now;
		  tenso_data = Tenso_UpdateAverage();
		  g_stream_data.ts_ms = (int)now;
		  g_stream_data.load = Tenso_in_kg(tenso_data);
		  g_stream_data.pos = (float)optical_count;
		  g_stream_data.angle1 = angle;
		  g_stream_data.angle2 = encoder_rpm;
		  g_stream_data.temp = 0.0f;
		  g_stream_data.flags = ((uint16_t)(calib_state) << 8) | ((uint16_t)state & 0x00FF);
		  g_stream_data.cycles = (uint32_t)(count / 2);
          uint8_t tracking = (state == MODE1_START && !mode1_seek_min);
          int32_t motor_command = Motor_GetCommandVelocity();
          HAL_StatusTypeDef tx_status = Motor_GetLastTxStatus();
          g_stream_data.target_deg_s = tracking ? direction * mode1_profile_rpm * 6.0f : 0.0f;
          g_stream_data.motor_rpm = motor_encoder_rpm;
          g_stream_data.motor_command = motor_command;
          g_stream_data.control_flags = tracking ? 1U : 0U;
          if (tracking && Mode1_AbsFloat((float)motor_command) >= MODE1_MOTOR_SPEED_MAX)
          {
              g_stream_data.control_flags |= 2U;
          }
          if (tx_status == HAL_BUSY)
          {
              g_stream_data.control_flags |= 4U;
          }
          else if (tx_status != HAL_OK)
          {
              g_stream_data.control_flags |= 8U;
          }
		  send_uart_packet(STREAM_DATA, (uint8_t*)&g_stream_data, sizeof(g_stream_data), &huart1);
	  }

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

			set_velocity = 200;
			regime = ACK;
			requested_mode_after_calib = 0;
			mode1_start = 0;
			mode1_seek_min = 0;
			Mode1_ResetPid();
			Encoder_Init();
			Motor_Stop();
			// state = MODE3_CONFIG;
			state = ACK;
			prev_state = INITIALIZATION;
			break;
		}
		case(CALIBRATION):
		{
			UpdateLimitFlagsFromPins();
			if (x1 < 1000000)
			{
				x1++;
			}
			else
			{
				// UI UART uses framed packets only.
				x1 = 0;
			}
			switch(calib_state)
			{
			case(RIGHT_RUN_FAST_CALIB):
			{
				if(limit_flag_1)
				{	Motor_Stop(); // добавил
					calib_state = RIGHT_LOGIC;
					calib_state_prev = RIGHT_RUN_FAST_CALIB;
					motion_ctrl = 1;
				}
				else if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity, 1); // 1 или -1 -- направление движения
					motion_ctrl = 0;
				}
				if(non_limit_switch_motion)
				{
					if(motion_ctrl)
					{
						Motor_Move(set_current, set_velocity, 1);
						motion_ctrl = 0;
					}
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
				if(non_limit_switch_motion)
				{
					if(motion_ctrl)
					{
						Motor_Move(set_current, set_velocity, -1);
						motion_ctrl = 0;
					}
					HAL_Delay(3000);
					//TODO: ПЕРЕДЕЛАТЬ НА ОПРОС ТЙМЕРА ПО ТИКАМ ДЛЯ ТОГО ЧТО БЫ НЕ БЛОКАТЬ РАБОТУ
					calib_state = RIGHT_LOGIC;
					calib_state_prev = LEFT_RUN_FAST_CALIB;
					motion_ctrl = 1;
					non_limit_switch_motion = 0;
				}
				else if(limit_flag_2)
				{
					Motor_Stop(); // добавил
					calib_state = LEFT_LOGIC;
					calib_state_prev = LEFT_RUN_FAST_CALIB;
					motion_ctrl = 1;
				}
				else if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity, -1);
					motion_ctrl = 0;
				}
				break;
			}

			case(RIGHT_RUN_SLOW_CALIB):
			{
				if(limit_flag_1)
				{
					Motor_Stop(); // добавил
					opt_lim_1 = optical_count;
					encoder_lim_1 = angle;
					calib_state = RIGHT_LOGIC;
					calib_state_prev = RIGHT_RUN_SLOW_CALIB;
					motion_ctrl = 1;
				}
				else if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity/5, 1);
					motion_ctrl = 0;
				}
				break;
			}

			case(LEFT_RUN_SLOW_CALIB):
			{
				if(limit_flag_2)
				{
					Motor_Stop(); // добавил
					opt_lim_2 = optical_count;
					encoder_lim_2 = angle;
					calib_state = LEFT_LOGIC;
					calib_state_prev = LEFT_RUN_SLOW_CALIB;
					motion_ctrl = 1;
				}
				else if(motion_ctrl)
				{
					Motor_Move(set_current, set_velocity/5, -1);
					motion_ctrl = 0;
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
				Motor_Stop();
				encoder_calibrated = true;
				Encoder_ResetRPMFilter();
				prev_state = CALIBRATION;
				if (requested_mode_after_calib == MODE1_CONFIG)
				{
					state = MODE1_CONFIG;
				}
				else if (requested_mode_after_calib == MODE2_CONFIG)
				{
					state = MODE2_CONFIG;
				}
				else
				{
					state = ACK;
				}
				requested_mode_after_calib = 0;
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




			min_angle = g_mode1_cfg.min_angle;
			max_angle = g_mode1_cfg.max_angle;
			if (encoder_calibrated)
			{
				float mech_min = 180.0f;
				float mech_max = encoder_lim_2 - encoder_lim_1 + 180.0f;
				if (mech_min > mech_max)
				{
					float tmp = mech_min;
					mech_min = mech_max;
					mech_max = tmp;
				}
				if (min_angle < mech_min)
				{
					min_angle = mech_min;
				}
				if (max_angle > mech_max)
				{
					max_angle = mech_max;
				}
			}
			if (max_angle <= min_angle)
			{
				Motor_Stop();
				mode1_seek_min = 0;
				state = ACK;
				prev_state = MODE1_CONFIG;
				break;
			}
			set_default_position = optical_count;
			direction = 1;
			motion_ctrl = 0;
			angle_velocity_ref = 20.0f * (1.0f + 2.0f * MODE1_RAMP_FRACTION);
			if (g_mode1_cfg.freq > 0.0f)
			{
				angle_velocity_ref = (max_angle - min_angle) * (2.0f * g_mode1_cfg.freq) / 6.0f * MODE1_FREQ_RPM_SCALE
                                   * (1.0f + 2.0f * MODE1_RAMP_FRACTION);
				if (angle_velocity_ref < 0.1f)
				{
					angle_velocity_ref = 0.1f;
				}
				else if (angle_velocity_ref > 100.0f)
				{
					angle_velocity_ref = 100.0f;
				}
			}
			Mode1_ResetProfile();
			pull_number = g_mode1_cfg.count;
			if (pull_number <= 0)
			{
				pull_number = 1;
			}
			count = 0;
			mode1_seek_min = 1;
			Mode1_ResetPid();

			state = MODE1_START;
			prev_state = MODE1_CONFIG;
			start_tick = HAL_GetTick();
			break;
		}
		case(MODE1_START):
		{
			if (mode1_seek_min)
			{
				float mode1_min_target = min_angle + MODE1_SEEK_MIN_OFFSET_DEG;
				if (mode1_min_target > max_angle)
				{
					mode1_min_target = min_angle;
				}
				if (angle < mode1_min_target - MODE1_SEEK_MIN_TOL_DEG)
				{
					if (!SafeMotorMove(set_current, MODE1_SEEK_MIN_SPEED, 1))
					{
						break;
					}
				}
				else if (angle > mode1_min_target + MODE1_SEEK_MIN_TOL_DEG)
				{
					if (!SafeMotorMove(set_current, MODE1_SEEK_MIN_SPEED, -1))
					{
						break;
					}
				}
				else
				{
					Motor_Stop();
					mode1_seek_min = 0;
					direction = 1;
					motion_ctrl = 1;
					count = 0;
					Encoder_ResetRPMFilter();
					Mode1_ResetPid();
					Mode1_ResetProfile();
				}
				break;
			}
			if((angle < max_angle) && (angle > min_angle))
			{
				motion_ctrl = 1;
			}

			if (angle >= max_angle)
			{
				if(motion_ctrl == 1)
				{
					Motor_Stop();
					direction = -1;
					count++;
					motion_ctrl = 0;
					Mode1_ResetPid();
					Mode1_ResetProfile();
					if (count >= pull_number * 2)
					{
						state = MODE1_STOP;
						prev_state = MODE1_START;
					}
					break;
				}
			}

			if (angle <= min_angle)
			{
				if(motion_ctrl == 1)
				{
					Motor_Stop();
					direction = 1;
					count++;
					motion_ctrl = 0;
					Mode1_ResetPid();
					Mode1_ResetProfile();
					if (count >= pull_number * 2)
					{
						state = MODE1_STOP;
						prev_state = MODE1_START;
					}
					break;
				}
			}

			angle_velocity = Mode1_GetProfileSpeed();

			if (!SafeMotorMove(set_current, Mode1_GetPidMotorSpeed(angle_velocity), direction))
			{
				break;
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
				// UI UART uses framed packets only.
				x1 = 0;
			}
			break;
		}
		case(MODE1_STOP):
		{
			mode1_seek_min = 0;
			Mode1_ResetPid();
			Motor_Stop();
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);

			break;
		}
		case(MODE2_CONFIG):
		{
			// MODE2_CONFIG only stores UI parameters. Movement starts on MODE2_START.
			state = ACK;
			prev_state = MODE2_CONFIG;
			break;
		}
		case(MODE2_START):
		{
			if (!mode2_configured || !encoder_calibrated || (mode2_phase == MODE2_PHASE_IDLE))
			{
				Mode2_StopAndResetToStart();
				break;
			}

			if (set_velocity <= 0.0f)
			{
				set_velocity = MODE2_DEFAULT_SPEED;
			}

			float mode2_error = mode2_target_angle - angle;
			if (Mode1_AbsFloat(mode2_error) <= MODE2_ANGLE_TOL_DEG)
			{
				Motor_Stop();
				if (mode2_phase == MODE2_PHASE_SEEK_START_THEN_END)
				{
					mode2_phase = MODE2_PHASE_MOVE_TO_END;
					mode2_target_angle = g_mode2_cfg.end_angle;
					mode2_error = mode2_target_angle - angle;
					if (Mode1_AbsFloat(mode2_error) > MODE2_ANGLE_TOL_DEG)
					{
						int mode2_direction = (mode2_error > 0.0f) ? 1 : -1;
						if (!SafeMotorMove(set_current, set_velocity, mode2_direction))
						{
							break;
						}
						break;
					}
				}

				mode2_next_target_is_end = (mode2_phase == MODE2_PHASE_MOVE_TO_START) ? 1 : 0;
				mode2_first_run = 0;
				mode2_phase = MODE2_PHASE_IDLE;
				state = MODE2_STOP;
				prev_state = MODE2_START;
				break;
			}

			int mode2_direction = (mode2_error > 0.0f) ? 1 : -1;
			SafeMotorMove(set_current, set_velocity, mode2_direction);
			break;
		}
		case(MODE2_HOLD):
		{
			// Old hold-by-time Mode2 logic is disabled for angle-to-angle mode.
			Motor_Stop();
			state = MODE2_STOP;
			prev_state = MODE2_HOLD;
			break;

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
			// Old optical-count return logic is disabled for angle-to-angle mode.
			Motor_Stop();
			state = MODE2_STOP;
			prev_state = MODE2_RETURN;
			break;

			SafeMotorMove(set_current, set_velocity, -1);//возвращаемся в нулевое положение
			if(optical_count > set_default_position)
			{
				Motor_Stop();
				state = MODE2_STOP;
				prev_state = MODE2_RETURN;
			}
			break;
		}
		case(MODE2_STOP):
		{
			Motor_Stop();
			HAL_GPIO_WritePin(MOTOR_BREAK_GPIO_Port, MOTOR_BREAK_Pin, GPIO_PIN_RESET);
			break;
		}

		case(MODE3_CONFIG):
		{
			t = 0;
			t0 = 0;
			v0 = 10;
			// v1 = 1500;
			v1 = 500;
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
				if (!SafeMotorMove(set_current, (float)v0 + (float)(t - t0)/1000 * (float)(v1 - v0)/time1, 1))
				{
					break;
				}
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
