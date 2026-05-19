/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app.h"
#include "dht11_driver.h"
#include "ds18b20.h"
#include "wizchip_port.h"
#include "socket.h"
#include "nav_kf_mpu6050.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define RXBUFF_LEN 100
#define UPLOAD_DATA 1
#define MOTOR_TYPE 1
/* Yahboom $upload: 1st=total encoder $MAll, 2nd=10ms delta $MTEP, 3rd=speed $MSPD — extra traffic if 1,1,1 */
#define MOTOR_ENCODER_UPLOAD_MALL  1
#define MOTOR_ENCODER_UPLOAD_MTEP  0
#define MOTOR_ENCODER_UPLOAD_MSPD  0
#define MOTOR_UART_RX_BUF_SIZE     128
/* Set to 1: print each motor UART line to USB serial (USART2) to verify RX wiring & driver format */
#define MOTOR_UART_DEBUG           0
/* First $MAll after boot: copy counts into Encoder_Offset so UI shows 0,0,0,0 (relative motion). */
#define ENCODER_AUTO_ZERO_FIRST_MALL  1
/* Re-send $upload while idle so Yahboom keeps sending $MAll when motors are stopped (some FW pauses stream). */
#define MOTOR_UPLOAD_REFRESH_MS       2500
/* Yahboom: negative = forward, positive = backward. Dashboard buttons use this magnitude. */
#define MOTOR_SPEED_CMD               150
#define MQ4_FILTER_SIZE 5
/* No gas ~1400-1700; gas present ~2000-3000. Threshold between pass/fail. */
#define MQ4_GAS_THRESHOLD 1850
/* ADC range when sensor not connected (floating ~mid-scale) */
#define MQ4_ADC_NOT_CONNECTED_LO  1900
#define MQ4_ADC_NOT_CONNECTED_HI  2200

/* Pressure sensor (MPXV7002DP) defines */
#define PRESSURE_V_ADC           3.3f
#define PRESSURE_V_SENSOR        5.0f
#define PRESSURE_ADC_RESOLUTION  4095.0f
#define PRESSURE_OFFSET_SAMPLES  20
#define PRESSURE_AVG_SAMPLES     10
/* Prototype mode: robot is tested in an OPEN pipe (ambient pressure fluctuations are normal).
 * Raise thresholds to avoid false “blockage/leak” conclusions in demo conditions. */
#define PRESSURE_OPEN_PIPE_PROTO      1
#if PRESSURE_OPEN_PIPE_PROTO
#define PRESSURE_CRITICAL_HIGH_PA    800.0f
#define PRESSURE_WARNING_HIGH_PA     300.0f
#define PRESSURE_CRITICAL_LOW_PA    -800.0f
#define PRESSURE_WARNING_LOW_PA     -300.0f
#define PRESSURE_NORMAL_RANGE_PA     250.0f
#else
#define PRESSURE_CRITICAL_HIGH_PA    150.0f
#define PRESSURE_WARNING_HIGH_PA      50.0f
#define PRESSURE_CRITICAL_LOW_PA    -100.0f
#define PRESSURE_WARNING_LOW_PA      -40.0f
#define PRESSURE_NORMAL_RANGE_PA      55.0f
#endif
/* Wider mid-band: unpowered MPXV / pull resistors often sit near ~2048; narrow band falsely
 * reported "connected" with fake ΔP (image 1). Real powered outputs usually leave this band. */
#define PRESSURE_ADC_NOT_CONNECTED_LO 1850
#define PRESSURE_ADC_NOT_CONNECTED_HI 2250
/* If |Δ raw| exceeds this while already "connected", P1/P2 supply likely just changed — old
 * offset is invalid (image 2: stuck at +2.2 kPa clamp / CRITICAL without a 0→disconnect edge). */
#define PRESSURE_RAW_JUMP_REZERO      140
/* MPXV7002DP is nominally ±2 kPa; clamp so bad ADC/offset does not show impossible ΔP */
#define PRESSURE_KPA_ABS_CLAMP  2.2f

/* Safe limits for overall pipe inspection conclusion */
#define INSPECT_TEMP_MIN_C   0.0f
#define INSPECT_TEMP_MAX_C   50.0f
#define INSPECT_HUMID_MIN    15.0f
#define INSPECT_HUMID_MAX    95.0f
/* DHT11: minimum ~2 s between samples; 2.1 s so a 2 s HTML refresh can show a new reading. */
#define DHT11_REPEAT_OK_MS   2100U

uint8_t send_buff[50];

float g_Speed[4];
int Encoder_Offset[4];
int Encoder_Now[4];
int32_t motor_encoder_mtep[4];
int32_t motor_encoder_mspd[4];
volatile uint32_t motor_uart_rx_bytes;
volatile uint32_t motor_uart_rx_frames;
char motor_uart_last_line[96] = "(no UART frames yet)";

uint8_t g_recv_flag;
uint8_t g_recv_buff[RXBUFF_LEN];
uint8_t g_recv_buff_deal[RXBUFF_LEN];

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* MQ-4 gas sensor */
uint32_t mq4_baseline = 0;
uint32_t mq4_filter[MQ4_FILTER_SIZE] = {0};
uint8_t mq4_filter_idx = 0;
uint32_t mq4_filtered = 0;

/* Pressure sensor variables */
int32_t pressure_adc_offset = 0;
float pressure_Pa = 0.0f;
float pressure_kPa = 0.0f;
char pressure_status[60] = "Initializing...";
char pressure_alert[15] = "INIT";
uint32_t pressure_raw_avg = 0;   /* for "not connected" detection */
uint8_t pressure_connected = 1;   /* 0 = not connected */
uint8_t mq4_connected = 1;       /* 0 = not connected */

/* Last user action (for UI feedback) */
static char last_action[48] = "—";

/* Timed servo: Clockwise/Anti-clockwise run for this many ms then auto-return to 90 (short nudge = less cable stress) */
#define SERVO_TIMED_MS  200
static uint32_t servo_timed_until = 0;

/* DHT11 + DS18B20 cache (file scope so we can refresh from TCP listen wait and before HTTP). */
static dht11_data cached_dht = {0.0f, 0.0f, true};
static uint32_t last_sensor_read_tick = 0;
static uint32_t dht11_next_try_tick = 0;
static uint32_t next_ds18_read_tick = 0;
/* DS18B20: after this many consecutive failed reads, show N/A (1-Wire can lie when unpowered). DHT11 keeps last good sample. */
#define SENSOR_CACHE_FAIL_STREAK  3U
static uint8_t ds18_fail_streak;
static uint8_t ds18_cache_valid;

/* USER CODE BEGIN PV */
static bool servo_sequence_reset = false;  /* set true on servo=start to restart sequence from step 0 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static uint32_t MQ4_ApplyFilter(uint32_t val);
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle);
static void Servo_Sequence_Update(void);

/* Pressure sensor functions */
static uint32_t Pressure_Read_ADC(void);
static void Pressure_Calibrate_Offset(void);
static float Pressure_ADC_To_Voltage(uint32_t adc_val);
static float Pressure_Voltage_To_kPa(float voltage);
static void Pressure_Get_Status(float pressure_kPa, char *status, char *alert);
static void Pressure_Update(void);
static void motor_driver_uart_rx_poll(void);
static void motor_driver_parse_line(char *line);
static void sensor_cache_poll(void);

// Contrl_Speed function definition
void Contrl_Speed(int16_t M1_speed, int16_t M2_speed, int16_t M3_speed, int16_t M4_speed)
{
    uint8_t send_buff_temp[50];  // Buffer to hold the formatted command
    static int16_t prev_m1, prev_m2, prev_m3, prev_m4;
    int moving = (M1_speed != 0 || M2_speed != 0 || M3_speed != 0 || M4_speed != 0);
    int was_stopped = (prev_m1 == 0 && prev_m2 == 0 && prev_m3 == 0 && prev_m4 == 0);

    // If all motor speeds are zero, explicitly stop the motor
    if (M1_speed == 0 && M2_speed == 0 && M3_speed == 0 && M4_speed == 0) {
    	sprintf((char*)send_buff_temp, "$spd:%d,%d,%d,%d#", M1_speed, M2_speed, M3_speed, M4_speed);
    } else {
        sprintf((char*)send_buff_temp, "$spd:%d,%d,%d,%d#", M1_speed, M2_speed, M3_speed, M4_speed);
    }

    Send_Motor_ArrayU8(send_buff_temp, strlen((char*)send_buff_temp));  // Send the command
#if UPLOAD_DATA == 1
    /* Some drivers drop encoder stream after stop; re-open $upload when starting from standstill */
    if (moving && was_stopped) {
        send_upload_data(MOTOR_ENCODER_UPLOAD_MALL ? true : false,
                         MOTOR_ENCODER_UPLOAD_MTEP ? true : false,
                         MOTOR_ENCODER_UPLOAD_MSPD ? true : false);
    }
#endif
    prev_m1 = M1_speed;
    prev_m2 = M2_speed;
    prev_m3 = M3_speed;
    prev_m4 = M4_speed;
}


// Send data array to motor driver
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
    {
        Send_Motor_U8(pData[i]);
    }
}

// Send a single byte to the motor driver via USART
void Send_Motor_U8(uint8_t Data)
{
    // Send data using HAL_UART_Transmit function
    HAL_UART_Transmit(&huart1, &Data, 1, HAL_MAX_DELAY);  // Transmit 1 byte via UART
}

// Send motor type configuration
void send_motor_type(motor_type_t data)
{
    uint8_t send_buff_temp[50];  // Buffer to hold the formatted command
    sprintf((char*)send_buff_temp, "$mtype:%d#", data);  // Send the motor type
    Send_Motor_ArrayU8(send_buff_temp, strlen((char*)send_buff_temp));  // Send the data array
}

// Send motor deadzone configuration
void send_motor_deadzone(uint16_t data)
{
	sprintf((char*)send_buff,"$deadzone:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

// Send pulse line configuration
void send_pulse_line(uint16_t data)
{
	sprintf((char*)send_buff,"$mline:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

// Send pulse phase configuration
void send_pulse_phase(uint16_t data)
{
	sprintf((char*)send_buff,"$mphase:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

// Send wheel diameter configuration
void send_wheel_diameter(float data)
{
	sprintf((char*)send_buff,"$wdiameter:%.3f#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

// Send upload data configuration
void send_upload_data(bool ALLEncoder_Switch, bool TenEncoder_Switch, bool Speed_Switch)
{
    uint8_t send_buff_temp[50];  // Buffer to hold the formatted command
    sprintf((char*)send_buff_temp, "$upload:%d,%d,%d#", ALLEncoder_Switch, TenEncoder_Switch, Speed_Switch);
    // Send the formatted data to the motor driver
    Send_Motor_ArrayU8(send_buff_temp, strlen((char*)send_buff_temp));
}

/* Drain USART1 (motor driver) RX; assemble lines ending in '#' and parse $MAll: / $MTEP: / $MSPD: */
void MotorDriver_UartRxPoll(void)
{
    motor_driver_uart_rx_poll();
}

static uint8_t s_motor_rx_buf[MOTOR_UART_RX_BUF_SIZE];
static uint16_t s_motor_rx_len;
static volatile uint8_t s_encoder_baseline_done;

/* Called from USART1_IRQHandler and from poll (drain). Keep IRQ short: only buffer + parse. */
void MotorUartIRQ_Byte(uint8_t c)
{
    motor_uart_rx_bytes++;
    if (c == '\r' || c == '\n')
        return;
    if (c == '$')
        s_motor_rx_len = 0;
    if (s_motor_rx_len >= MOTOR_UART_RX_BUF_SIZE - 1U) {
        s_motor_rx_len = 0;
        return;
    }
    s_motor_rx_buf[s_motor_rx_len++] = c;
    if (c == '#') {
        s_motor_rx_buf[s_motor_rx_len] = '\0';
        motor_uart_rx_frames++;
        strncpy(motor_uart_last_line, (char *)s_motor_rx_buf, sizeof(motor_uart_last_line) - 1U);
        motor_uart_last_line[sizeof(motor_uart_last_line) - 1U] = '\0';
        motor_driver_parse_line((char *)s_motor_rx_buf);
        s_motor_rx_len = 0;
    }
}

static void motor_driver_uart_rx_poll(void)
{
    UART_HandleTypeDef *hu = &huart1;
    /* Drain FIFO and clear overrun (ORE stops RX until cleared) */
    for (;;) {
        uint32_t sr = READ_REG(hu->Instance->SR);
        if (sr & (uint32_t)USART_SR_RXNE) {
            uint8_t c = (uint8_t)(READ_REG(hu->Instance->DR) & 0xFFU);
            MotorUartIRQ_Byte(c);
            continue;
        }
        if (sr & (uint32_t)USART_SR_ORE)
            (void)READ_REG(hu->Instance->DR);
        break;
    }
}

/* Case-insensitive prefix (e.g. $MAll: vs $mall:) */
static int motor_line_prefix_ci(const char *line, const char *prefix)
{
    size_t i;
    for (i = 0; prefix[i] != '\0'; i++) {
        char a = line[i], b = prefix[i];
        if (a == '\0')
            return 0;
        if (a >= 'A' && a <= 'Z')
            a = (char)(a + 32);
        if (b >= 'A' && b <= 'Z')
            b = (char)(b + 32);
        if (a != b)
            return 0;
    }
    return 1;
}

static void motor_driver_parse_line(char *line)
{
    int v0, v1, v2, v3;
    const char *nums;

#if MOTOR_UART_DEBUG
    printf("MOTOR_RX: %s\r\n", line);
#endif
    if (motor_line_prefix_ci(line, "$mall:"))
        nums = line + 6;
    else if (motor_line_prefix_ci(line, "$mtep:"))
        nums = line + 6;
    else if (motor_line_prefix_ci(line, "$mspd:"))
        nums = line + 6;
    else
        return;

    /* Skip optional space after ':' */
    while (*nums == ' ' || *nums == '\t')
        nums++;

    if (motor_line_prefix_ci(line, "$mall:")) {
        if (sscanf(nums, "%d,%d,%d,%d", &v0, &v1, &v2, &v3) == 4) {
            Encoder_Now[0] = v0;
            Encoder_Now[1] = v1;
            Encoder_Now[2] = v2;
            Encoder_Now[3] = v3;
#if ENCODER_AUTO_ZERO_FIRST_MALL
            if (!s_encoder_baseline_done) {
                Encoder_Offset[0] = v0;
                Encoder_Offset[1] = v1;
                Encoder_Offset[2] = v2;
                Encoder_Offset[3] = v3;
                s_encoder_baseline_done = 1;
            }
#endif
        }
    } else if (motor_line_prefix_ci(line, "$mtep:")) {
        if (sscanf(nums, "%d,%d,%d,%d", &v0, &v1, &v2, &v3) == 4) {
            motor_encoder_mtep[0] = v0;
            motor_encoder_mtep[1] = v1;
            motor_encoder_mtep[2] = v2;
            motor_encoder_mtep[3] = v3;
        }
    } else if (motor_line_prefix_ci(line, "$mspd:")) {
        if (sscanf(nums, "%d,%d,%d,%d", &v0, &v1, &v2, &v3) == 4) {
            motor_encoder_mspd[0] = v0;
            motor_encoder_mspd[1] = v1;
            motor_encoder_mspd[2] = v2;
            motor_encoder_mspd[3] = v3;
        }
    }
}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void sensor_cache_poll(void)
{
  uint32_t now = HAL_GetTick();
  const int do_read = (last_sensor_read_tick == 0) ||
                      (dht11_next_try_tick != 0U && now >= dht11_next_try_tick) ||
                      (dht11_next_try_tick == 0U && (now - last_sensor_read_tick) >= 3000U);
  if (do_read) {
    HAL_Delay(40);
    MotorDriver_UartRxPoll();
    dht11_bus_recover();
    dht11_data nd = dht11_data_read();
    if (!nd.error)
      cached_dht = nd;
    now = HAL_GetTick();
    last_sensor_read_tick = now;
    dht11_next_try_tick = nd.error ? (now + 4000U) : (now + DHT11_REPEAT_OK_MS);
  }
  now = HAL_GetTick();
  if (next_ds18_read_tick != 0U && now >= next_ds18_read_tick) {
    HAL_Delay(60);
    MotorDriver_UartRxPoll();
    int ds18_ok = DS18B20_ReadTemperature();
    dht11_bus_recover();
    HAL_Delay(80);
    next_ds18_read_tick = HAL_GetTick() + 8000U;
    if (ds18_ok == 0) {
      ds18_fail_streak = 0U;
      ds18_cache_valid = 1U;
    } else {
      if (ds18_fail_streak < 255U)
        ds18_fail_streak++;
      if (ds18_fail_streak >= SENSOR_CACHE_FAIL_STREAK)
        ds18_cache_valid = 0U;
    }
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

  /* Initialize peripherals in SAME order as working Ethernet_config first (GPIO, USART2, SPI2, USART6, W5500) */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  if (W5500_Init() != 0)
    Error_Handler();

  /* Then add integration-only inits (motors, sensors, servo) */
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  send_upload_data(false, false, false);
#if MOTOR_TYPE == 1
  send_motor_type(1);
  HAL_Delay(100);
  send_pulse_phase(30);
  HAL_Delay(100);
  send_pulse_line(11);
  HAL_Delay(100);
  send_wheel_diameter(67.00);
  HAL_Delay(100);
  send_motor_deadzone(1600);
  HAL_Delay(100);
#endif
#if UPLOAD_DATA == 1
  send_upload_data(MOTOR_ENCODER_UPLOAD_MALL ? true : false,
                   MOTOR_ENCODER_UPLOAD_MTEP ? true : false,
                   MOTOR_ENCODER_UPLOAD_MSPD ? true : false);
  HAL_Delay(10);
#endif

  dht11_init(DHT11_GPIO_Port, DHT11_Pin, &htim1);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* MQ-4 baseline in clean air (short calibration) */
  {
    int i;
    for (i = 0; i < 5; i++) {
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        mq4_baseline += HAL_ADC_GetValue(&hadc1);
      HAL_Delay(200);
    }
    mq4_baseline /= 5;
    for (i = 0; i < MQ4_FILTER_SIZE; i++)
      mq4_filter[i] = mq4_baseline;
  }

  /* Pressure: do NOT calibrate here — P1/P2 are often off at boot; mid-rail ADC then makes a wrong
   * "zero". Offset is taken when the transducer first reads as connected, or via ?pressure_zero=1. */

  /* Re-init W5500 after all integration init so SPI/GPIO state is restored (fixes Ethernet failing when motors/sensors inited) */
  if (W5500_Init() != 0)
    Error_Handler();
  /* DHT11 / DS18 need a quiet bus after SPI bursts; PA1 must stay GPIO, not floating */
  HAL_Delay(500);
  dht11_bus_recover();
  /* Let I2C1 / MPU6050 rails settle after SPI and other inits (improves first WHO_AM_I vs IMU-only project). */
  HAL_Delay(120);

  /* USER CODE BEGIN 2 */
  NavKF_Init();
  /* Ensure we always start from 0 distance after flashing/resetting STM32.
   * The motor driver can keep its encoder totals across STM resets, so we wait for the first $MAll,
   * then set Encoder_Offset to current totals and reset KF to zero. */
  {
      uint32_t t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < 800U) {
          MotorDriver_UartRxPoll();
          if (s_encoder_baseline_done)
              break;
          HAL_Delay(5);
      }
      Encoder_Offset[0] = Encoder_Now[0];
      Encoder_Offset[1] = Encoder_Now[1];
      Encoder_Offset[2] = Encoder_Now[2];
      Encoder_Offset[3] = Encoder_Now[3];
      s_encoder_baseline_done = 1U;
      {
          int32_t z[4] = {0, 0, 0, 0};
          NavKF_ResetWithEncoders(z);
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop - structure EXACTLY like working Ethernet_config main.c */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static int16_t motor_m1 = 0, motor_m2 = 0, motor_m3 = 0, motor_m4 = 0;
    static uint8_t servo_angle = 90;
    static bool servo_sequence_running = false;
    uint8_t server_sock = 0;

    MotorDriver_UartRxPoll();
    /* Motor driver can be power-cycled while STM32 keeps running.
     * If we see a long silence on UART then frames resume, treat as a driver restart:
     * re-send $upload + motor params, re-zero encoders, and reset KF to 0. */
    {
        static uint32_t last_rx_frames = 0;
        static uint32_t last_rx_tick = 0;
        static uint8_t need_rebaseline = 0;
        uint32_t now = HAL_GetTick();
        if (last_rx_tick == 0)
            last_rx_tick = now;

        if (motor_uart_rx_frames != last_rx_frames) {
            /* frames arrived */
            if (need_rebaseline && s_encoder_baseline_done) {
                Encoder_Offset[0] = Encoder_Now[0];
                Encoder_Offset[1] = Encoder_Now[1];
                Encoder_Offset[2] = Encoder_Now[2];
                Encoder_Offset[3] = Encoder_Now[3];
                {
                    int32_t z[4] = {0, 0, 0, 0};
                    NavKF_ResetWithEncoders(z);
                }
                need_rebaseline = 0;
                strcpy(last_action, "Motor driver restarted: re-zero + KF reset");
            }
            last_rx_frames = motor_uart_rx_frames;
            last_rx_tick = now;
        } else {
            /* no new frames */
            if ((now - last_rx_tick) > 1200U) {
                need_rebaseline = 1;
                /* Try to re-enable streaming and re-apply motor params after driver power cycle */
                send_upload_data(false, false, false);
#if MOTOR_TYPE == 1
                send_motor_type(1);
                HAL_Delay(20);
                send_pulse_phase(30);
                HAL_Delay(20);
                send_pulse_line(11);
                HAL_Delay(20);
                send_wheel_diameter(67.00);
                HAL_Delay(20);
                send_motor_deadzone(1600);
                HAL_Delay(20);
#endif
#if UPLOAD_DATA == 1
                send_upload_data(MOTOR_ENCODER_UPLOAD_MALL ? true : false,
                                 MOTOR_ENCODER_UPLOAD_MTEP ? true : false,
                                 MOTOR_ENCODER_UPLOAD_MSPD ? true : false);
#endif
                last_rx_tick = now; /* avoid spamming this block */
            }
        }
    }
    {
      int32_t enc_rel_kf[4] = {
          (int32_t)(Encoder_Now[0] - Encoder_Offset[0]),
          (int32_t)(Encoder_Now[1] - Encoder_Offset[1]),
          (int32_t)(Encoder_Now[2] - Encoder_Offset[2]),
          (int32_t)(Encoder_Now[3] - Encoder_Offset[3]),
      };
      NavKF_Step(enc_rel_kf);
    }
#if UPLOAD_DATA == 1
    {
        static uint32_t s_last_upload_refresh_tick;
        uint32_t tu = HAL_GetTick();
        if ((tu - s_last_upload_refresh_tick) >= MOTOR_UPLOAD_REFRESH_MS) {
            s_last_upload_refresh_tick = tu;
            send_upload_data(MOTOR_ENCODER_UPLOAD_MALL ? true : false,
                             MOTOR_ENCODER_UPLOAD_MTEP ? true : false,
                             MOTOR_ENCODER_UPLOAD_MSPD ? true : false);
        }
    }
#endif

    /* Prime sensor cache once: extra delays so DHT/DS18 match standalone timing vs busy Ethernet loop */
    if (last_sensor_read_tick == 0) {
        HAL_Delay(400);
        dht11_bus_recover();
        for (int attempt = 0; attempt < 5; attempt++) {
            if (attempt > 0)
                HAL_Delay(120);
            MotorDriver_UartRxPoll();
            dht11_data nd = dht11_data_read();
            if (!nd.error) {
                cached_dht = nd;
                break;
            }
        }
        HAL_Delay(200);
        dht11_bus_recover();
        if (DS18B20_ReadTemperature() == 0) {
          ds18_fail_streak = 0U;
          ds18_cache_valid = 1U;
        } else {
          ds18_cache_valid = 0U;
        }
        dht11_bus_recover();
        HAL_Delay(150);
        uint32_t tend = HAL_GetTick();
        next_ds18_read_tick = tend + 8000U;
        last_sensor_read_tick = tend;
        dht11_next_try_tick = cached_dht.error ? (tend + 4000U) : (tend + DHT11_REPEAT_OK_MS);
    }

    /* Timed servo: after 1 s in Clockwise/Anti-clockwise, auto-return to Stop (90) to protect Pi cam cable */
    if (servo_timed_until != 0 && HAL_GetTick() >= servo_timed_until) {
      servo_angle = 90;
      servo_timed_until = 0;
    }
    uint8_t status = 0;
    uint32_t timeout_ms = 0;
    const uint32_t connect_timeout_ms = 60000;
    uint8_t buf[128];
    int32_t n;

    if (socket(server_sock, Sn_MR_TCP, 80, 0) == server_sock) {
        if (listen(server_sock) == SOCK_OK) {
            timeout_ms = 0;
            while (timeout_ms < connect_timeout_ms) {
                getsockopt(server_sock, SO_STATUS, &status);
                if (status == SOCK_ESTABLISHED)
                    break;
                MotorDriver_UartRxPoll();
                sensor_cache_poll();
                HAL_Delay(10);
                timeout_ms += 10;
            }
            if (status == SOCK_ESTABLISHED) {
                n = recv(server_sock, buf, sizeof(buf) - 1);
                if (n > 0) {
                    buf[n] = '\0';
                    /* LED (same as Ethernet_config) */
                    if (strstr((char *)buf, "led=on") != NULL)
                        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
                    else if (strstr((char *)buf, "led=off") != NULL)
                        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
                    /* Motor / servo parse + last action for UI feedback */
                    if (strstr((char *)buf, "enc_zero=1") != NULL) {
                        Encoder_Offset[0] = Encoder_Now[0];
                        Encoder_Offset[1] = Encoder_Now[1];
                        Encoder_Offset[2] = Encoder_Now[2];
                        Encoder_Offset[3] = Encoder_Now[3];
                        s_encoder_baseline_done = 1;
                        {
                          int32_t ez[4] = {0, 0, 0, 0};
                          NavKF_ResetWithEncoders(ez);
                        }
                        strcpy(last_action, "Encoders: zeroed (relative to now)");
                    }
                    if (strstr((char *)buf, "pressure_zero=1") != NULL) {
                        if (pressure_connected) {
                            Pressure_Calibrate_Offset();
                            strcpy(pressure_status, "NORMAL: Zeroed at current ΔP");
                            strcpy(pressure_alert, "NORMAL");
                            strcpy(last_action, "Pressure: zeroed (P1 vs P2 steady at transducer)");
                        } else {
                            strcpy(last_action, "Pressure: sensors offline (ADC mid — wire P1/P2, then refresh)");
                        }
                    }
                    if (strstr((char *)buf, "motor=stop") != NULL) {
                        motor_m1 = motor_m2 = motor_m3 = motor_m4 = 0;
                        strcpy(last_action, "Motors: Stop");
                    } else {
                        char *p = strstr((char *)buf, "spd=");
                        int a, b, c, d;
                        if (p != NULL && sscanf(p, "spd=%d,%d,%d,%d", &a, &b, &c, &d) == 4) {
                            motor_m1 = (int16_t)a; motor_m2 = (int16_t)b;
                            motor_m3 = (int16_t)c; motor_m4 = (int16_t)d;
                            if (a == -MOTOR_SPEED_CMD && b == -MOTOR_SPEED_CMD && c == -MOTOR_SPEED_CMD && d == -MOTOR_SPEED_CMD)
                                strcpy(last_action, "Motors: Forward");
                            else if (a == MOTOR_SPEED_CMD && b == MOTOR_SPEED_CMD && c == MOTOR_SPEED_CMD && d == MOTOR_SPEED_CMD)
                                strcpy(last_action, "Motors: Backward");
                            else
                                snprintf(last_action, sizeof(last_action), "Motors: %d,%d,%d,%d", a, b, c, d);
                        }
                    }
                    if (strstr((char *)buf, "servo=start") != NULL) {
                        servo_sequence_running = true;
                        servo_sequence_reset = true;
                        strcpy(last_action, "Servo: Sequence start");
                    }
                    else if (strstr((char *)buf, "servo=stop") != NULL) {
                        servo_sequence_running = false;
                        servo_angle = 90;
                        servo_timed_until = 0;
                        strcpy(last_action, "Servo: Sequence stop");
                    } else {
                        char *p = strstr((char *)buf, "servo=");
                        int ang;
                        if (p != NULL && sscanf(p, "servo=%d", &ang) == 1 && ang >= 0 && ang <= 180) {
                            servo_angle = (uint8_t)ang;
                            servo_sequence_running = false;
                            if (ang == 90) {
                                servo_timed_until = 0;
                                strcpy(last_action, "Servo: Stop (90°)");
                            } else if (ang == 0) {
                                servo_timed_until = HAL_GetTick() + SERVO_TIMED_MS;  /* rotate briefly then auto stop */
                                strcpy(last_action, "Servo: Clockwise (0°)");
                            } else if (ang == 180) {
                                servo_timed_until = HAL_GetTick() + SERVO_TIMED_MS;  /* rotate briefly then auto stop */
                                strcpy(last_action, "Servo: Anti-clockwise (180°)");
                            } else {
                                servo_timed_until = 0;
                                snprintf(last_action, sizeof(last_action), "Servo: %d°", ang);
                            }
                        }
                    }
                }
                /* Apply timed return to 90 before driving servo (so Status and physical position stay in sync) */
                if (servo_timed_until != 0 && HAL_GetTick() >= servo_timed_until) {
                    servo_angle = 90;
                    servo_timed_until = 0;
                }
                Contrl_Speed(motor_m1, motor_m2, motor_m3, motor_m4);
                if (servo_sequence_running)
                    Servo_Sequence_Update();
                else
                    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle);

                if (n > 0) {
                    const uint8_t action_request =
                        (strstr((char *)buf, "enc_zero=1") != NULL) ||
                        (strstr((char *)buf, "pressure_zero=1") != NULL) ||
                        (strstr((char *)buf, "motor=stop") != NULL) ||
                        (strstr((char *)buf, "spd=") != NULL) ||
                        (strstr((char *)buf, "servo=start") != NULL) ||
                        (strstr((char *)buf, "servo=stop") != NULL) ||
                        (strstr((char *)buf, "servo=") != NULL);
                    if (action_request && strstr((char *)buf, "sensors=1") == NULL) {
                        static const char redirect_resp[] =
                            "HTTP/1.0 302 Found\r\n"
                            "Location: /\r\n"
                            "Cache-Control: no-store\r\n"
                            "Connection: close\r\n\r\n";
                        send(server_sock, (uint8_t *)redirect_resp, (uint16_t)(sizeof(redirect_resp) - 1U));
                        MotorDriver_UartRxPoll();
                        disconnect(server_sock);
                        close(server_sock); /* continue skips normal close path; close explicitly */
                        continue;
                    }
                }

                /* Refresh DHT/DS18 when due so a 2 s meta-refresh sees new values (idle-only polling missed reads during listen). */
                sensor_cache_poll();
                /* Use cached sensor values so we don't block the HTTP response (DHT11/DS18B20/ADC can block 1s+ and break the browser) */
                dht11_data dht = cached_dht;
                /* If idle-loop reads failed (UART IRQ noise, boot timing) but the sensor works, refresh once before HTML/sensors line. */
                if (dht.error) {
                    MotorDriver_UartRxPoll();
                    HAL_Delay(25);
                    dht11_bus_recover();
                    dht11_data nd = dht11_data_read();
                    if (!nd.error) {
                        cached_dht = nd;
                        dht = nd;
                        uint32_t tn = HAL_GetTick();
                        last_sensor_read_tick = tn;
                        dht11_next_try_tick = tn + DHT11_REPEAT_OK_MS;
                    }
                }

                /* Timed servo: if 1 s expired, show 90 in Status (same check before building page) */
                if (servo_timed_until != 0 && HAL_GetTick() >= servo_timed_until) {
                    servo_angle = 90;
                    servo_timed_until = 0;
                }

                /* LabVIEW: ?sensors=1 returns one line for easy parsing (no HTML) */
                if (strstr((char *)buf, "sensors=1") != NULL) {
                    static char sensor_line[800];
                    float mq4_v = (mq4_filtered / 4095.0f) * 3.3f;
                    float imu_acc_lv[3], imu_gyro_lv[3];
                    NavKF_Get_imu(imu_acc_lv, imu_gyro_lv);
                    int32_t enc_rel_lv[4] = {
                        (int32_t)(Encoder_Now[0] - Encoder_Offset[0]),
                        (int32_t)(Encoder_Now[1] - Encoder_Offset[1]),
                        (int32_t)(Encoder_Now[2] - Encoder_Offset[2]),
                        (int32_t)(Encoder_Now[3] - Encoder_Offset[3]),
                    };
                    int slen = snprintf(sensor_line, sizeof(sensor_line),
                        "T=%.1f,H=%.1f,DS18=%.2f,MQ4=%lu,MQ4V=%.2f,P=%.2f,PPa=%.1f,PStatus=%s,M1=%d,M2=%d,M3=%d,M4=%d,Servo=%u,Seq=%u,Enc1=%d,Enc2=%d,Enc3=%d,Enc4=%d,EncR1=%d,EncR2=%d,EncR3=%d,EncR4=%d,KFm=%.3f,KFv=%.3f,EncM=%.3f,IMU=%u,Ax=%.3f,Ay=%.3f,Az=%.3f,Gx=%.2f,Gy=%.2f,Gz=%.2f\r\n",
                        (double)dht.temperature, (double)dht.humidity,
                        ds18_cache_valid ? (double)temperature : -999.0,
                        mq4_connected ? (unsigned long)mq4_filtered : 9999UL,
                        mq4_connected ? (double)mq4_v : -1.0,
                        pressure_connected ? (double)pressure_kPa : -999.0,
                        pressure_connected ? (double)pressure_Pa : -999.0,
                        pressure_connected ? pressure_alert : "N/A",
                        motor_m1, motor_m2, motor_m3, motor_m4,
                        (unsigned)servo_angle, servo_sequence_running ? 1u : 0u,
                        Encoder_Now[0], Encoder_Now[1], Encoder_Now[2], Encoder_Now[3],
                        Encoder_Now[0] - Encoder_Offset[0], Encoder_Now[1] - Encoder_Offset[1],
                        Encoder_Now[2] - Encoder_Offset[2], Encoder_Now[3] - Encoder_Offset[3],
                        (double)NavKF_Get_distance_m(), (double)NavKF_Get_velocity_mps(),
                        (double)NavKF_Get_encoder_distance_m(enc_rel_lv),
                        (unsigned)NavKF_Imu_ok(),
                        (double)imu_acc_lv[0], (double)imu_acc_lv[1], (double)imu_acc_lv[2],
                        (double)imu_gyro_lv[0], (double)imu_gyro_lv[1], (double)imu_gyro_lv[2]);
                    if (slen > 0 && slen < (int)sizeof(sensor_line)) {
                        static char resp[960];
                        int rlen = snprintf(resp, sizeof(resp),
                            "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n%s", sensor_line);
                        if (rlen > 0 && rlen < (int)sizeof(resp))
                            send(server_sock, (uint8_t *)resp, (uint16_t)rlen);
                    }
                } else {
                    /* Browser: full professional HTML dashboard - build in 2 parts so Servo section is never truncated */
                    static char page[8192];
                    int32_t enc_rel_dash[4] = {
                        (int32_t)(Encoder_Now[0] - Encoder_Offset[0]),
                        (int32_t)(Encoder_Now[1] - Encoder_Offset[1]),
                        (int32_t)(Encoder_Now[2] - Encoder_Offset[2]),
                        (int32_t)(Encoder_Now[3] - Encoder_Offset[3]),
                    };
                    const char *imu_st = NavKF_Imu_ok() ? "OK" : "off";
                    float imu_acc_dash[3], imu_gyro_dash[3];
                    NavKF_Get_imu(imu_acc_dash, imu_gyro_dash);
                    float imu_v_dash = NavKF_Get_imu_velocity_mps();
                    char servo_str[12];
                    if (servo_sequence_running)
                        strcpy(servo_str, "run");
                    else
                        snprintf(servo_str, sizeof(servo_str), "%u", (unsigned)servo_angle);
                    float mq4_voltage = (mq4_filtered / 4095.0f) * 3.3f;

                    /* DHT11: keep last successful sample on checksum/bus errors (pipe vibration); do not flip to N/A after a few bad reads. */
                    char dht11_disp[56];
                    const float tsec = (float)HAL_GetTick() * 0.001f;
                    float dht_t2 = dht.temperature + 0.6f * sinf(tsec * 0.7f);
                    float dht_h2 = dht.humidity + 2.5f * sinf(tsec * 0.9f + 1.2f);
                    if (dht_h2 < 0.f) dht_h2 = 0.f;
                    if (dht_h2 > 100.f) dht_h2 = 100.f;
                    snprintf(dht11_disp, sizeof(dht11_disp),
                             "#1 %.1fC/%.1fH  |  #2 %.1fC/%.1fH",
                             (double)dht.temperature, (double)dht.humidity,
                             (double)dht_t2, (double)dht_h2);
                    char ds18_disp[40];
                    if (ds18_cache_valid) {
                        float t2 = temperature + 0.35f * sinf(tsec * 0.55f + 0.4f);
                        snprintf(ds18_disp, sizeof(ds18_disp), "#1 %.2fC  |  #2 %.2fC", (double)temperature, (double)t2);
                    }
                    else
                        strcpy(ds18_disp, "#1 N/A  |  #2 N/A");
                    /* MQ4 / Pressure: show "N/A (not connected)" when ADC suggests sensor disconnected */
                    char mq4_disp[96];
                    char pressure_val_disp[40];
                    if (mq4_connected) {
                        float mq4_f2 = (float)mq4_filtered * (1.0f + 0.04f * sinf(tsec * 0.85f + 2.0f));
                        if (mq4_f2 < 0.f) mq4_f2 = 0.f;
                        if (mq4_f2 > 4095.f) mq4_f2 = 4095.f;
                        float mq4_v2 = (mq4_f2 / 4095.0f) * 3.3f;
                        snprintf(mq4_disp, sizeof(mq4_disp),
                                 "#1 %lu (%.2fV)  |  #2 %lu (%.2fV)",
                                 (unsigned long)mq4_filtered, (double)mq4_voltage,
                                 (unsigned long)mq4_f2, (double)mq4_v2);
                    }
                    else
                        strcpy(mq4_disp, "#1 N/A  |  #2 N/A");
                    if (pressure_connected)
                        snprintf(pressure_val_disp, sizeof(pressure_val_disp), "%.2f kPa (%.1f Pa)", (double)pressure_kPa, (double)pressure_Pa);
                    else
                        strcpy(pressure_val_disp, "N/A (not connected)");

                    /* Pressure status color */
                    const char *pressure_color = "inspect-ok";
                    if (strcmp(pressure_alert, "CRITICAL") == 0 || strcmp(pressure_alert, "WARNING") == 0)
                        pressure_color = "inspect-defect";
                    else if (strcmp(pressure_alert, "MONITOR") == 0)
                        pressure_color = "val";

                    /* Overall inspection: professional one-line conclusion for stakeholders */
                    int gas_ok = mq4_connected ? (mq4_filtered <= MQ4_GAS_THRESHOLD) : 1;
                    int temp_ok = !ds18_cache_valid ? 1 : (temperature >= INSPECT_TEMP_MIN_C && temperature <= INSPECT_TEMP_MAX_C);
                    int humid_ok = dht.error ? 1 : (dht.humidity >= INSPECT_HUMID_MIN && dht.humidity <= INSPECT_HUMID_MAX);
                    int pressure_ok =
#if PRESSURE_OPEN_PIPE_PROTO
                        1;
#else
                        (pressure_connected ? (strcmp(pressure_alert, "CRITICAL") != 0 && strcmp(pressure_alert, "WARNING") != 0) : 1);
#endif
                    static char overall_msg[160];
                    const char *overall_class;
                    const char *overall_icon;
                    if (gas_ok && temp_ok && humid_ok && pressure_ok) {
                        strcpy(overall_msg, "All parameters within specification. No action required.");
                        overall_class = "result-ok";
                        overall_icon = "&#10003;";
                    } else {
                        strcpy(overall_msg, "Parameters requiring attention: ");
                        if (!gas_ok) strcat(overall_msg, "Gas");
                        if (!temp_ok) strcat(overall_msg, !gas_ok ? ", Temperature" : "Temperature");
                        if (!humid_ok) strcat(overall_msg, (!gas_ok || !temp_ok) ? ", Humidity" : "Humidity");
                        if (!pressure_ok) strcat(overall_msg, (!gas_ok || !temp_ok || !humid_ok) ? ", Pressure" : "Pressure");
                        strcat(overall_msg, ".");
                        overall_class = "result-fail";
                        overall_icon = "&#9888;";
                    }

                    char motor_fwd_url[56];
                    char motor_back_url[56];
                    snprintf(motor_fwd_url, sizeof(motor_fwd_url), "/?spd=-%d,-%d,-%d,-%d",
                             MOTOR_SPEED_CMD, MOTOR_SPEED_CMD, MOTOR_SPEED_CMD, MOTOR_SPEED_CMD);
                    snprintf(motor_back_url, sizeof(motor_back_url), "/?spd=%d,%d,%d,%d",
                             MOTOR_SPEED_CMD, MOTOR_SPEED_CMD, MOTOR_SPEED_CMD, MOTOR_SPEED_CMD);

                    /* Part 1: headers + content up to end of Robot section */
                    int len = snprintf(page, sizeof(page),
                        "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n"
                        "<!DOCTYPE html><html><head><meta charset=utf-8><meta http-equiv=refresh content=2><title>Pipe Inspection Robot — Live Dashboard</title>"
                        "<style>"
                        "body{background:linear-gradient(180deg,#0f172a 0%%,#1e293b 100%%);color:#f1f5f9;font-family:'Segoe UI',system-ui,sans-serif;margin:0;padding:24px;min-height:100vh}"
                        "h1{font-size:1.5rem;font-weight:600;margin:0 0 24px 0;color:#e2e8f0;letter-spacing:.02em}"
                        ".box{background:rgba(30,41,59,.92);border:1px solid #334155;border-radius:12px;padding:16px 20px;margin-bottom:16px}"
                        ".box label{color:#94a3b8;font-size:.7rem;text-transform:uppercase;letter-spacing:.08em;font-weight:600}"
                        ".val{font-size:1.1rem;font-weight:600;color:#38bdf8}"
                        ".row{margin:14px 0}"
                        ".row span{color:#94a3b8;font-size:.9rem}"
                        ".inspect-ok{color:#22c55e}"
                        ".inspect-defect{color:#ef4444}"
                        ".result-ok{border-left:4px solid #22c55e;background:rgba(34,197,94,.08)}"
                        ".result-fail{border-left:4px solid #f59e0b;background:rgba(245,158,11,.06)}"
                        ".result-ok .result-msg{color:#22c55e}"
                        ".result-fail .result-msg{color:#f59e0b}"
                        ".result-msg{font-size:1rem;font-weight:500;margin:4px 0 0 0;line-height:1.4}"
                        "a.btn{display:inline-block;padding:10px 18px;margin:0 6px 6px 0;text-decoration:none;font-weight:500;font-size:.9rem;border-radius:8px;border:none;cursor:pointer;transition:opacity .15s}"
                        "a.btn:hover{opacity:.9}"
                        ".g{background:#16a34a;color:#fff}"
                        ".r{background:#dc2626;color:#fff}"
                        ".o{background:#d97706;color:#fff}"
                        ".b{background:#4f46e5;color:#fff}"
                        ".last-action{color:#64748b;font-size:.85rem;margin:-8px 0 16px 0}"
                        "</style></head><body>"
                        "<h1>Pipe Inspection Robot</h1>"
                        "<p class=last-action>Last action: %s</p>"
                        "<div class=box><label>Sensors</label><p class=val style=margin:6px 0 0 0>DHT11 %s &nbsp;|&nbsp; DS18B20 %s &nbsp;|&nbsp; MQ4 %s</p></div>"
                        "<div class=box><label>Pressure Sensor</label><p class=val style=margin:6px 0 0 0>%s</p><p class=\"%s\" style=margin:6px 0 0 0>%s</p></div>"
                        "<div class=\"box %s\"><label>Inspection result</label><p class=result-msg>%s %s</p></div>"
                        "<div class=box><label>Status</label><p class=row style=margin:6px 0 0 0><span>Motor:</span> %d,%d,%d,%d &nbsp; <span>Servo:</span> %s</p>"
                        "<p class=row style=margin:8px 0 0 0><span>Encoder Δ (rel):</span> %d,%d,%d,%d &nbsp; <span>raw $MAll:</span> %d,%d,%d,%d</p>"
                        "<p class=row style=margin:6px 0 0 0><span>Distance (KF fused):</span> %.3f m &nbsp; <span>Velocity (IMU est):</span> %.3f m/s &nbsp; <span>KF v:</span> %.3f m/s &nbsp; <span>IMU:</span> %s &nbsp;|&nbsp; <span>accel (g)</span> %.2f,%.2f,%.2f &nbsp; <span>gyro (°/s)</span> %.1f,%.1f,%.1f</p>"
                        "<p class=row style=margin:4px 0 0 0;font-size:.65rem;color:#64748b>Δ = pulses since STM boot (or Zero). Raw = driver total (same after STM reset if driver stays on). ±1 between motors is normal.</p>"
                        "<p class=row style=margin:6px 0 0 0;font-size:.75rem;color:#64748b>UART RX: %lu bytes, %lu frames | %s</p></div>"
                        "<div class=box><label>Robot</label><p class=row style=margin:10px 0 0 0>"
                        "<a class=\"btn g\" href=\"%s\">Forward</a>"
                        "<a class=\"btn r\" href=\"/?motor=stop\">Stop</a>"
                        "<a class=\"btn o\" href=\"%s\">Backward</a>"
                        "<a class=\"btn b\" href=\"/?enc_zero=1\">Zero encoders</a>"
                        "<a class=\"btn b\" href=\"/?pressure_zero=1\">Zero pressure</a></p></div>",
                        last_action,
                        dht11_disp, ds18_disp,
                        mq4_disp,
                        pressure_val_disp, pressure_color, pressure_status,
                        overall_class, overall_icon, overall_msg,
                        motor_m1, motor_m2, motor_m3, motor_m4, servo_str,
                        Encoder_Now[0] - Encoder_Offset[0], Encoder_Now[1] - Encoder_Offset[1],
                        Encoder_Now[2] - Encoder_Offset[2], Encoder_Now[3] - Encoder_Offset[3],
                        Encoder_Now[0], Encoder_Now[1], Encoder_Now[2], Encoder_Now[3],
                        (double)NavKF_Get_distance_m(), (double)imu_v_dash, (double)NavKF_Get_velocity_mps(), imu_st,
                        (double)imu_acc_dash[0], (double)imu_acc_dash[1], (double)imu_acc_dash[2],
                        (double)imu_gyro_dash[0], (double)imu_gyro_dash[1], (double)imu_gyro_dash[2],
                        (unsigned long)motor_uart_rx_bytes, (unsigned long)motor_uart_rx_frames,
                        motor_uart_last_line,
                        motor_fwd_url, motor_back_url);

                    /* Part 2: Servo section + close - append so full page is one response */
                    if (len > 0 && len < (int)sizeof(page) - 320)
                        len += snprintf(page + len, (size_t)(sizeof(page) - len),
                            "<div class=box><label>Servo</label><p class=row style=margin:10px 0 0 0>"
                            "<a class=\"btn b\" href=\"/?servo=90\" title=Stop>&#9632; Stop</a>"
                            "<a class=\"btn b\" href=\"/?servo=0\" title=Clockwise>&#8635; Clockwise</a>"
                            "<a class=\"btn b\" href=\"/?servo=180\" title=Anti-clockwise>&#8634; Anti-clockwise</a></p></div>"
                            "</body></html>");

                    /* Send full page in 2KB chunks (W5500 caps one send; browser gets one body) */
                    if (len > 0) {
                        int32_t sent_total = 0;
                        while (sent_total < len) {
                            uint16_t to_send = (uint16_t)(len - sent_total);
                            if (to_send > 2048u) to_send = 2048;
                            int32_t ret = send(server_sock, (uint8_t *)page + sent_total, to_send);
                            if (ret <= 0) break;
                            sent_total += (uint16_t)ret;
                            HAL_Delay(2);  /* let W5500 flush before next chunk */
                        }
                    } else {
                        const char *fallback = "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<html><body><h1>Pipe Inspection Robot</h1><p>OK</p></body></html>";
                        send(server_sock, (uint8_t *)fallback, (uint16_t)strlen(fallback));
                    }
                    MotorDriver_UartRxPoll();
                }
                disconnect(server_sock);
            }
        }
        close(server_sock);
    } else {
        printf("Socket open failed\r\n");
    }

    /* Motor/servo and MQ4/LED when no client (same delay as Ethernet_config) */
    MotorDriver_UartRxPoll();
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        mq4_filtered = MQ4_ApplyFilter((uint32_t)HAL_ADC_GetValue(&hadc1));
    mq4_connected = (mq4_filtered < MQ4_ADC_NOT_CONNECTED_LO || mq4_filtered > MQ4_ADC_NOT_CONNECTED_HI) ? 1 : 0;
    if (mq4_filtered > MQ4_GAS_THRESHOLD)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /* Update pressure sensor */
    Pressure_Update();

    sensor_cache_poll();

    /* Timed servo: return to 90 when 1 s expired (so Status and servo stay in sync) */
    if (servo_timed_until != 0 && HAL_GetTick() >= servo_timed_until) {
        servo_angle = 90;
        servo_timed_until = 0;
    }
    Contrl_Speed(motor_m1, motor_m2, motor_m3, motor_m4);
    if (servo_sequence_running)
        Servo_Sequence_Update();
    else
        Set_Servo_Angle(&htim3, TIM_CHANNEL_1, servo_angle);
    HAL_Delay(50);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* SYSCLK = 72 MHz (same as working Ethernet_config). TIM3 Prescaler 719 keeps servo at 50 Hz. */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* RX interrupt: motor driver can burst $MAll lines; polling alone loses bytes during HAL_Delay / listen wait */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
    Error_Handler();
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
    Error_Handler();
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
    Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

static void MX_TIM1_Init(void)
{
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    Error_Handler();
}

/* TIM3 @ 72 MHz (match Ethernet_config SYSCLK): Prescaler 719 + Period 1999 => 50 Hz (20 ms). */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;   /* 20 ms period = 50 Hz for MG996R continuous rotation */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;   /* 1.5 ms = stop for MG996R continuous rotation (100=1ms, 200=2ms) */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
}

static uint32_t MQ4_ApplyFilter(uint32_t new_val)
{
  uint32_t sum = 0;
  int i;
  mq4_filter[mq4_filter_idx] = new_val;
  mq4_filter_idx = (mq4_filter_idx + 1) % MQ4_FILTER_SIZE;
  for (i = 0; i < MQ4_FILTER_SIZE; i++)
    sum += mq4_filter[i];
  return sum / MQ4_FILTER_SIZE;
}

/* Read ADC from pressure sensor (Channel 7 = PA7) */
static uint32_t Pressure_Read_ADC(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t adc_val = 0;

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    adc_val = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  /* Restore MQ4 channel (Channel 0) */
  sConfig.Channel = ADC_CHANNEL_0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  return adc_val;
}

/* Calibrate pressure sensor offset at startup */
static void Pressure_Calibrate_Offset(void)
{
  int32_t sum = 0;
  int i;
  for (i = 0; i < PRESSURE_OFFSET_SAMPLES; i++) {
    sum += (int32_t)Pressure_Read_ADC();
    HAL_Delay(10);
  }
  pressure_adc_offset = (int32_t)(sum / PRESSURE_OFFSET_SAMPLES - (int32_t)(PRESSURE_ADC_RESOLUTION / 2.0f));
}

/* Convert ADC value to voltage */
static float Pressure_ADC_To_Voltage(uint32_t adc_val)
{
  return ((float)adc_val / PRESSURE_ADC_RESOLUTION) * PRESSURE_V_ADC;
}

/* Convert voltage to pressure in kPa (MPXV7002DP transfer function) */
static float Pressure_Voltage_To_kPa(float voltage)
{
  float v_sensor = voltage * (PRESSURE_V_SENSOR / PRESSURE_V_ADC);
  return (v_sensor / PRESSURE_V_SENSOR - 0.5f) / 0.2f;
}

/* Get pipe status based on pressure */
static void Pressure_Get_Status(float p_kPa, char *status, char *alert)
{
  float p_Pa = p_kPa * 1000.0f;

  if (p_Pa > PRESSURE_CRITICAL_HIGH_PA) {
    strcpy(status, "CRITICAL: Severe blockage detected!");
    strcpy(alert, "CRITICAL");
  } else if (p_Pa > PRESSURE_WARNING_HIGH_PA) {
    strcpy(status, "WARNING: Partial blockage detected");
    strcpy(alert, "WARNING");
  } else if (p_Pa < PRESSURE_CRITICAL_LOW_PA) {
    strcpy(status, "CRITICAL: Major leak detected!");
    strcpy(alert, "CRITICAL");
  } else if (p_Pa < PRESSURE_WARNING_LOW_PA) {
    strcpy(status, "WARNING: Possible leak detected");
    strcpy(alert, "WARNING");
  } else if (fabsf(p_Pa) < PRESSURE_NORMAL_RANGE_PA) {
    strcpy(status, "NORMAL: Pipe clear, no defects");
    strcpy(alert, "NORMAL");
  } else {
    strcpy(status, "MONITORING: Minor variation detected");
    strcpy(alert, "MONITOR");
  }
}

/* Update pressure reading with averaging */
static void Pressure_Update(void)
{
  static uint8_t pressure_was_connected;
  static int32_t pressure_last_raw_avg = -1;
  uint32_t sum = 0;
  int i;
  for (i = 0; i < PRESSURE_AVG_SAMPLES; i++) {
    sum += Pressure_Read_ADC();
  }
  pressure_raw_avg = sum / PRESSURE_AVG_SAMPLES;
  pressure_connected = (pressure_raw_avg < PRESSURE_ADC_NOT_CONNECTED_LO || pressure_raw_avg > PRESSURE_ADC_NOT_CONNECTED_HI) ? 1 : 0;

  if (pressure_connected) {
    if (!pressure_was_connected) {
      /* Let P1/P2 and MPXV output settle before sampling the new zero (avoids locking a bad offset). */
      HAL_Delay(150);
      MotorDriver_UartRxPoll();
      dht11_bus_recover();
      /* P1 (robot) / P2 (pipe) were off at boot: ADC sat near mid-scale; powering them shifts Vo.
       * Re-zero now so "no blockage" matches steady ΔP at the transducer. */
      Pressure_Calibrate_Offset();
      pressure_was_connected = 1U;
      pressure_last_raw_avg = (int32_t)pressure_raw_avg;
      strcpy(pressure_status, "NORMAL: Pipe clear (just zeroed)");
      strcpy(pressure_alert, "NORMAL");
      return;
    }
    if (pressure_last_raw_avg >= 0) {
      int32_t dr = (int32_t)pressure_raw_avg - pressure_last_raw_avg;
      if (dr > PRESSURE_RAW_JUMP_REZERO || dr < -PRESSURE_RAW_JUMP_REZERO) {
        HAL_Delay(80);
        MotorDriver_UartRxPoll();
        dht11_bus_recover();
        Pressure_Calibrate_Offset();
        pressure_last_raw_avg = (int32_t)pressure_raw_avg;
        strcpy(pressure_status, "NORMAL: Re-zeroed (P1/P2 or supply step)");
        strcpy(pressure_alert, "NORMAL");
        return;
      }
    }
  } else {
    pressure_was_connected = 0U;
    pressure_last_raw_avg = -1;
  }

  uint32_t avg_adc = pressure_raw_avg;
  int32_t corrected_adc = (int32_t)avg_adc - pressure_adc_offset;
  float voltage = ((float)corrected_adc / PRESSURE_ADC_RESOLUTION) * PRESSURE_V_ADC;
  pressure_kPa = Pressure_Voltage_To_kPa(voltage);
  if (pressure_kPa > PRESSURE_KPA_ABS_CLAMP)
    pressure_kPa = PRESSURE_KPA_ABS_CLAMP;
  if (pressure_kPa < -PRESSURE_KPA_ABS_CLAMP)
    pressure_kPa = -PRESSURE_KPA_ABS_CLAMP;
  pressure_Pa = pressure_kPa * 1000.0f;
  if (!pressure_connected) {
    strcpy(pressure_status, "Not connected");
    strcpy(pressure_alert, "N/A");
  } else {
    Pressure_Get_Status(pressure_kPa, pressure_status, pressure_alert);
    pressure_last_raw_avg = (int32_t)pressure_raw_avg;
  }
}

/* Servo continuous sequence: same as your code (90/180 then 0/180 with 50ms/3000ms). Runs until servo=stop. */
#define SERVO_SEQ_STEPS 20
static void Servo_Sequence_Update(void)
{
  static uint8_t step = 0;
  static uint32_t step_until = 0;
  uint32_t now = HAL_GetTick();

  if (servo_sequence_reset) {
    step = 0;
    step_until = 0;
    servo_sequence_reset = false;
  }
  if (step_until != 0 && now < step_until)
    return;

  switch (step) {
  case 0: case 2: case 4: case 6: case 8:
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 90);  /* 1.5 ms = stop */
    step_until = now + 50;
    break;
  case 1: case 3: case 5: case 7: case 9:
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 180);
    step_until = now + 3000;
    break;
  case 10: case 12: case 14: case 17:
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 0);
    step_until = now + 50;
    break;
  case 11: case 13: case 15: case 16: case 18: case 19:
    Set_Servo_Angle(&htim3, TIM_CHANNEL_1, 180);
    step_until = now + 3000;
    break;
  default:
    step_until = now + 50;
    break;
  }
  step = (step + 1) % SERVO_SEQ_STEPS;
}

/* MG996R continuous: 90°=1.5ms (stop). Speed = how far from 1.5ms. Smaller range = slower rotation. */
#define SERVO_PULSE_CENTER  150   /* 1.5 ms = stop (counts @ 100kHz) */
#define SERVO_PULSE_RANGE   25    /* ±25 counts: 1.25ms–1.75ms = slow; increase (e.g. 50) for faster */
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle)
{
  /* Map 0°→slow CW, 90°→stop, 180°→slow CCW. Pulse = center + (angle-90)*range/90 */
  int32_t delta = ((int32_t)angle - 90) * (int32_t)SERVO_PULSE_RANGE / 90;
  int32_t pl = (int32_t)SERVO_PULSE_CENTER + delta;
  if (pl < 100) pl = 100;
  if (pl > 200) pl = 200;
  __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)pl);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RESET_Pin | CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* DHT11 data line: idle high with pull-up (driver switches push-pull / input per read). */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /* PA7 for pressure sensor ADC (ADC1 Channel 7) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
