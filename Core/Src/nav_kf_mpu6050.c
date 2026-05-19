#include "nav_kf_mpu6050.h"
#include "main.h"
#include <math.h>
#include <string.h>

/*
 * Pipe-axis navigation: 2-state linear Kalman filter (s = distance m, v = speed m/s).
 * Encoders ($MAll, new frame via motor_uart_rx_frames) provide position and inter-frame speed.
 * MPU6050: used only for ZUPT — occasional measurement v≈0 when the IMU looks stationary. There is
 * no double-integration of accelerometer into distance (that would drift badly). Fused distance is
 * therefore encoder-dominated; the IMU trims velocity when the robot is genuinely still.
 */
extern I2C_HandleTypeDef hi2c1;
extern volatile uint32_t motor_uart_rx_frames;

#define MPU6050_PWR_MGMT_1   0x6BU
#define MPU6050_WHO_AM_I     0x75U
#define MPU6050_SMPLRT_DIV   0x19U
#define MPU6050_GYRO_CONFIG  0x1BU
#define MPU6050_ACCEL_CONFIG 0x1CU
#define MPU6050_ACCEL_XOUT_H 0x3BU
#define MPU6050_GYRO_XOUT_H  0x43U
#define WHO_AM_I_MPU6050     0x68U
#define WHO_AM_I_MPU6500     0x70U
/* Match IMU-only project reliability: timeout is max wait; successful transfers finish in <1 ms. */
#define MPU6050_I2C_TIMEOUT_MS 1000U

#define GRAVITY_MPS2       9.80665f
#define NAV_CALIB_SAMPLES  48U
#define NAV_CALIB_DELAY_MS 3U
#define NAV_ACCEL_DEADBAND_G  0.02f
#define ZUPT_ACC_SUM_G        0.22f
#define ZUPT_GYRO_AXIS_DPS    2.5f

/* Wheel travel per count: circumference / (quadrature counts per full wheel revolution).
 * send_pulse_line(11) is 11 lines; quadrature is typically x4 edges per line on the sensed shaft.
 * If the driver’s $MAll counts the MOTOR shaft (before gearbox), one wheel rev = (11*4)*ratio.
 * If $MAll counts the OUTPUT/wheel shaft only, set ratio to 1 (use 11*4 only).
 * Calibrate: zero encoders, roll exactly one wheel turn, read avg(|Δ four motors|) and set this
 * denominator to that integer. For this prototype, 48 gives realistic tape distance. */
#define NAV_KF_QUAD_COUNTS_PER_WHEEL_REV  (11.0f * 4.0f * 48.0f)
#define NAV_KF_WHEEL_DIAMETER_M           (0.067f)
#define NAV_KF_M_PER_AVG_COUNT \
  ((float)(3.14159265 * (double)NAV_KF_WHEEL_DIAMETER_M / (double)NAV_KF_QUAD_COUNTS_PER_WHEEL_REV))
/* Robot command uses negative = forward (e.g. -150) -> encoders count negative while moving forward.
 * NAV_KF_ENC_SIGN -1 maps that to +distance / +encoder delta on the dashboard. */
#define NAV_KF_ENC_SIGN         (-1.0f)
/* Still if |Δs_enc| < this many "average encoder ticks" in meters between two NavKF_Step calls.
 * A fixed 2.5 mm threshold was wrong once m_per became ~1e-4: motion <2.5 mm/step looked "still"
 * forever → velocity stuck at 0 even while driving. */
#define NAV_KF_ENC_STILL_AVG_TICKS  (0.85f)

static uint8_t g_mpu_ready;
static uint16_t g_mpu6050_i2c_addr8;
static uint8_t g_who_am_i;

static int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
static int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
static float Ax, Ay, Az, Gx, Gy, Gz;
static float g_cal_ax, g_cal_ay, g_cal_az;
static float g_cal_gx, g_cal_gy, g_cal_gz;

/* Linear KF: state [s, v] = distance (m), velocity (m/s) along pipe */
static float kf_s, kf_v;
static float kf_P00, kf_P01, kf_P10, kf_P11;
static uint32_t kf_tick_ms;
static float enc_s_prev;
static int32_t enc_rel_prev[4];
static uint8_t kf_bootstrapped;

/* Encoder fusion runs faster than Yahboom $MAll; velocity must use Δs between UART frames. */
static uint32_t nav_mall_fr_prev;
static float nav_s_enc_at_mall;
static uint32_t nav_tick_at_mall;
static uint8_t nav_have_mall_anchor;
/* Last encoder-derived speed (m/s) from a $MAll frame; dashboard fallback if KF v was ZUPT’d. */
static float nav_last_venc_mps;
/* Allow IMU ZUPT only when encoders also say “still”, else pipe vibration zeros v after every update. */
static uint8_t nav_zupt_ok = 1U;
/* IMU-only longitudinal velocity estimate (m/s), leaky-integrated for display (not fed into KF). */
static float nav_imu_v_mps;

static void nav_imu_integrate_velocity(float dt_s, float lax, float lay, float laz)
{
  if (dt_s < 0.002f || dt_s > 0.5f)
    dt_s = 0.05f;

  /* Pick the dominant calibrated axis automatically so board orientation mismatch
   * does not pin IMU velocity near zero on the dashboard. */
  float a_g = lax;
  if (fabsf(lay) > fabsf(a_g))
    a_g = lay;
  if (fabsf(laz) > fabsf(a_g))
    a_g = laz;

  /* Deadband small residual accel (noise around 0g after calibration). */
  if (a_g > -NAV_ACCEL_DEADBAND_G && a_g < NAV_ACCEL_DEADBAND_G)
    a_g = 0.f;

  const float a_mps2 = a_g * GRAVITY_MPS2;

  /* Leaky integration to keep bounded drift while still showing “IMU velocity trend”. */
  const float tau_s = 6.0f; /* larger = less decay, more responsive to sustained motion */
  const float leak = expf(-dt_s / tau_s);
  nav_imu_v_mps = nav_imu_v_mps * leak + a_mps2 * dt_s;

  /* Hard clamp: this is only for display. */
  if (nav_imu_v_mps > 3.0f) nav_imu_v_mps = 3.0f;
  if (nav_imu_v_mps < -3.0f) nav_imu_v_mps = -3.0f;
}

static uint8_t MPU6x00_WhoAmI_Ok(uint8_t id)
{
  return (id == WHO_AM_I_MPU6050 || id == WHO_AM_I_MPU6500) ? 1U : 0U;
}

static void I2C1_Recover(void)
{
  GPIO_InitTypeDef gpio = {0};
  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(2);
  for (uint32_t n = 0U; n < 9U; n++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(2);
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  (void)HAL_I2C_Init(&hi2c1);
}

static HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint16_t addr8, uint8_t *who)
{
  uint8_t b2[2];
  if (HAL_I2C_Mem_Read(&hi2c1, addr8, MPU6050_WHO_AM_I, 1U, who, 1U, MPU6050_I2C_TIMEOUT_MS) == HAL_OK)
    return HAL_OK;
  I2C1_Recover();
  if (HAL_I2C_Mem_Read(&hi2c1, addr8, MPU6050_WHO_AM_I, 1U, b2, 2U, MPU6050_I2C_TIMEOUT_MS) == HAL_OK) {
    *who = b2[0];
    return HAL_OK;
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef MPU6050_Init(void)
{
  static const uint16_t kAddr8[] = {0xD0U, 0xD2U};
  uint8_t check = 0;
  uint8_t data;
  uint8_t found = 0U;

  if (hi2c1.State != HAL_I2C_STATE_READY)
    I2C1_Recover();

  for (uint32_t i = 0U; i < (sizeof(kAddr8) / sizeof(kAddr8[0])); i++) {
    data = 0x00U;
    if (HAL_I2C_Mem_Write(&hi2c1, kAddr8[i], MPU6050_PWR_MGMT_1, 1U, &data, 1U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
      I2C1_Recover();
      continue;
    }
    HAL_Delay(25);
    if (MPU6050_ReadWhoAmI(kAddr8[i], &check) != HAL_OK) {
      I2C1_Recover();
      continue;
    }
    if (MPU6x00_WhoAmI_Ok(check)) {
      g_mpu6050_i2c_addr8 = kAddr8[i];
      g_who_am_i = check;
      found = 1U;
      break;
    }
    I2C1_Recover();
  }
  if (found == 0U)
    return HAL_ERROR;

  data = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, g_mpu6050_i2c_addr8, MPU6050_PWR_MGMT_1, 1U, &data, 1U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    I2C1_Recover();
    return HAL_ERROR;
  }
  data = 0x07U;
  if (HAL_I2C_Mem_Write(&hi2c1, g_mpu6050_i2c_addr8, MPU6050_SMPLRT_DIV, 1U, &data, 1U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    I2C1_Recover();
    return HAL_ERROR;
  }
  data = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, g_mpu6050_i2c_addr8, MPU6050_GYRO_CONFIG, 1U, &data, 1U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    I2C1_Recover();
    return HAL_ERROR;
  }
  data = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, g_mpu6050_i2c_addr8, MPU6050_ACCEL_CONFIG, 1U, &data, 1U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    I2C1_Recover();
    return HAL_ERROR;
  }
  return HAL_OK;
}

static void MPU6050_Read_Accel(void)
{
  uint8_t rec[6];
  if (HAL_I2C_Mem_Read(&hi2c1, g_mpu6050_i2c_addr8, MPU6050_ACCEL_XOUT_H, 1U, rec, 6U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    I2C1_Recover();
    return;
  }
  Accel_X_RAW = (int16_t)((rec[0] << 8) | rec[1]);
  Accel_Y_RAW = (int16_t)((rec[2] << 8) | rec[3]);
  Accel_Z_RAW = (int16_t)((rec[4] << 8) | rec[5]);
  Ax = (float)Accel_X_RAW / 16384.0f;
  Ay = (float)Accel_Y_RAW / 16384.0f;
  Az = (float)Accel_Z_RAW / 16384.0f;
}

static void MPU6050_Read_Gyro(void)
{
  uint8_t rec[6];
  if (HAL_I2C_Mem_Read(&hi2c1, g_mpu6050_i2c_addr8, MPU6050_GYRO_XOUT_H, 1U, rec, 6U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
    I2C1_Recover();
    return;
  }
  Gyro_X_RAW = (int16_t)((rec[0] << 8) | rec[1]);
  Gyro_Y_RAW = (int16_t)((rec[2] << 8) | rec[3]);
  Gyro_Z_RAW = (int16_t)((rec[4] << 8) | rec[5]);
  Gx = (float)Gyro_X_RAW / 131.0f;
  Gy = (float)Gyro_Y_RAW / 131.0f;
  Gz = (float)Gyro_Z_RAW / 131.0f;
}

static void Nav_Calibrate(void)
{
  double sax = 0.0, say = 0.0, saz = 0.0;
  double sgx = 0.0, sgy = 0.0, sgz = 0.0;
  for (uint32_t i = 0U; i < NAV_CALIB_SAMPLES; i++) {
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();
    sax += (double)Ax;
    say += (double)Ay;
    saz += (double)Az;
    sgx += (double)Gx;
    sgy += (double)Gy;
    sgz += (double)Gz;
    HAL_Delay(NAV_CALIB_DELAY_MS);
  }
  const double n = (double)NAV_CALIB_SAMPLES;
  g_cal_ax = (float)(sax / n);
  g_cal_ay = (float)(say / n);
  g_cal_az = (float)(saz / n);
  g_cal_gx = (float)(sgx / n);
  g_cal_gy = (float)(sgy / n);
  g_cal_gz = (float)(sgz / n);
}

static float encoder_distance_m(const int32_t enc_rel[4])
{
  float c = 0.25f * ((float)enc_rel[0] + (float)enc_rel[1] + (float)enc_rel[2] + (float)enc_rel[3]);
  float d = (c * NAV_KF_ENC_SIGN) * NAV_KF_M_PER_AVG_COUNT;
  /* ±1 count jitter on one motor → tiny signed distance and "%.3f" prints "-0.000"; treat as zero. */
  const float eps = fabsf(NAV_KF_M_PER_AVG_COUNT) * 0.51f;
  if (fabsf(d) < eps)
    return 0.f;
  return d;
}

float NavKF_Get_encoder_distance_m(const int32_t enc_rel[4])
{
  return encoder_distance_m(enc_rel);
}

static void kf_reset_state(void)
{
  kf_s = 0.f;
  kf_v = 0.f;
  kf_P00 = 0.25f;
  kf_P01 = 0.f;
  kf_P10 = 0.f;
  kf_P11 = 1.0f;
}

static void kf_predict(float dt)
{
  kf_s += kf_v * dt;
  const float q00 = 1e-6f;
  const float q11 = 5e-4f;
  const float np00 = kf_P00 + dt * (kf_P01 + kf_P10) + dt * dt * kf_P11 + q00;
  const float np01 = kf_P01 + dt * kf_P11;
  const float np10 = kf_P10 + dt * kf_P11;
  const float np11 = kf_P11 + q11;
  kf_P00 = np00;
  kf_P01 = np01;
  kf_P10 = np10;
  kf_P11 = np11;
}

static void kf_update_scalar(float H0, float H1, float z, float R)
{
  const float hx = H0 * kf_s + H1 * kf_v;
  const float y = z - hx;
  const float S = H0 * (kf_P00 * H0 + kf_P10 * H1) + H1 * (kf_P01 * H0 + kf_P11 * H1) + R;
  if (S < 1e-9f)
    return;
  const float K0 = (kf_P00 * H0 + kf_P01 * H1) / S;
  const float K1 = (kf_P10 * H0 + kf_P11 * H1) / S;
  kf_s += K0 * y;
  kf_v += K1 * y;
  const float nP00 = kf_P00 - K0 * (kf_P00 * H0 + kf_P01 * H1);
  const float nP01 = kf_P01 - K0 * (kf_P10 * H0 + kf_P11 * H1);
  const float nP10 = kf_P10 - K1 * (kf_P00 * H0 + kf_P01 * H1);
  const float nP11 = kf_P11 - K1 * (kf_P10 * H0 + kf_P11 * H1);
  kf_P00 = nP00;
  kf_P01 = nP01;
  kf_P10 = nP10;
  kf_P11 = nP11;
}

void NavKF_ResetWithEncoders(const int32_t enc_rel[4])
{
  kf_bootstrapped = 0U;
  kf_reset_state();
  enc_s_prev = encoder_distance_m(enc_rel);
  memcpy(enc_rel_prev, enc_rel, sizeof(enc_rel_prev));
  kf_tick_ms = HAL_GetTick();
  nav_mall_fr_prev = motor_uart_rx_frames;
  nav_s_enc_at_mall = enc_s_prev;
  nav_tick_at_mall = HAL_GetTick();
  nav_have_mall_anchor = 0U;
  nav_last_venc_mps = 0.f;
  nav_zupt_ok = 1U;
  nav_imu_v_mps = 0.f;
}

void NavKF_Init(void)
{
  kf_bootstrapped = 0U;
  kf_reset_state();
  memset(enc_rel_prev, 0, sizeof(enc_rel_prev));
  enc_s_prev = 0.f;
  kf_tick_ms = HAL_GetTick();

  g_mpu_ready = 0U;
  nav_imu_v_mps = 0.f;
  for (uint32_t attempt = 0U; attempt < 4U; attempt++) {
    if (hi2c1.State != HAL_I2C_STATE_READY)
      I2C1_Recover();
    HAL_Delay(40);
    if (MPU6050_Init() == HAL_OK) {
      g_mpu_ready = 1U;
      Nav_Calibrate();
      return;
    }
  }
}

uint8_t NavKF_Imu_ok(void)
{
  return g_mpu_ready;
}

void NavKF_Get_imu(float acc_g[3], float gyro_dps[3])
{
  if (!acc_g || !gyro_dps)
    return;
  if (!g_mpu_ready) {
    acc_g[0] = acc_g[1] = acc_g[2] = 0.f;
    gyro_dps[0] = gyro_dps[1] = gyro_dps[2] = 0.f;
    return;
  }
  acc_g[0] = Ax;
  acc_g[1] = Ay;
  acc_g[2] = Az;
  gyro_dps[0] = Gx;
  gyro_dps[1] = Gy;
  gyro_dps[2] = Gz;
}

float NavKF_Get_distance_m(void)
{
  const float eps = fabsf(NAV_KF_M_PER_AVG_COUNT) * 0.51f;
  if (fabsf(kf_s) < eps)
    return 0.f;
  return kf_s;
}

float NavKF_Get_velocity_mps(void)
{
  if (fabsf(kf_v) >= 1e-4f)
    return kf_v;
  if (fabsf(nav_last_venc_mps) >= 5e-4f)
    return nav_last_venc_mps;
  return 0.f;
}

float NavKF_Get_imu_velocity_mps(void)
{
  if (!g_mpu_ready)
    return 0.f;
  /* Very small noise floor to avoid “-0.000”. */
  if (fabsf(nav_imu_v_mps) < 1e-4f)
    return 0.f;
  return nav_imu_v_mps;
}

void NavKF_Step(const int32_t enc_rel[4])
{
  uint32_t now = HAL_GetTick();
  static uint32_t last_imu_retry_ms;

  if (!g_mpu_ready) {
    if (last_imu_retry_ms == 0U || (now - last_imu_retry_ms) >= 4000U) {
      last_imu_retry_ms = now;
      if (hi2c1.State != HAL_I2C_STATE_READY)
        I2C1_Recover();
      HAL_Delay(35);
      if (MPU6050_Init() == HAL_OK) {
        Nav_Calibrate();
        g_mpu_ready = 1U;
      }
    }
  }

  float dt = (float)((int32_t)(now - kf_tick_ms)) * 0.001f;
  if (dt < 0.002f || dt > 0.5f)
    dt = 0.05f;
  kf_tick_ms = now;

  const float s_enc = encoder_distance_m(enc_rel);
  if (!kf_bootstrapped) {
    kf_reset_state();
    kf_s = s_enc;
    kf_v = 0.f;
    enc_s_prev = s_enc;
    memcpy(enc_rel_prev, enc_rel, sizeof(enc_rel_prev));
    nav_mall_fr_prev = motor_uart_rx_frames;
    nav_s_enc_at_mall = s_enc;
    nav_tick_at_mall = now;
    nav_have_mall_anchor = 0U;
    nav_last_venc_mps = 0.f;
    nav_zupt_ok = 1U;
    kf_bootstrapped = 1U;
    return;
  }

  const uint8_t new_mall = (motor_uart_rx_frames != nav_mall_fr_prev) ? 1U : 0U;
  if (new_mall)
    nav_mall_fr_prev = motor_uart_rx_frames;

  /* Between $MAll frames s_enc is flat → ds_enc/step≈0 and v was forced to 0 every loop. */
  if (!new_mall) {
    kf_predict(dt);
    kf_s = s_enc;
    memcpy(enc_rel_prev, enc_rel, sizeof(enc_rel_prev));
    if (g_mpu_ready) {
      MPU6050_Read_Accel();
      MPU6050_Read_Gyro();
      const float lax = Ax - g_cal_ax;
      const float lay = Ay - g_cal_ay;
      const float laz = Az - g_cal_az;
      nav_imu_integrate_velocity(dt, lax, lay, laz);
      const float dgx = Gx - g_cal_gx;
      const float dgy = Gy - g_cal_gy;
      const float dgz = Gz - g_cal_gz;
      const int still = (fabsf(lax) + fabsf(lay) + fabsf(laz) < ZUPT_ACC_SUM_G) &&
                        (fabsf(dgx) < ZUPT_GYRO_AXIS_DPS) && (fabsf(dgy) < ZUPT_GYRO_AXIS_DPS) &&
                        (fabsf(dgz) < ZUPT_GYRO_AXIS_DPS);
      if (still && nav_zupt_ok)
        kf_update_scalar(0.f, 1.f, 0.f, 1e-4f);
    }
    return;
  }

  /* New $MAll: Δs over inter-frame time → meaningful v_enc. */
  if (!nav_have_mall_anchor) {
    nav_s_enc_at_mall = s_enc;
    nav_tick_at_mall = now;
    nav_have_mall_anchor = 1U;
    enc_s_prev = s_enc;
    memcpy(enc_rel_prev, enc_rel, sizeof(enc_rel_prev));
    if (g_mpu_ready) {
      MPU6050_Read_Accel();
      MPU6050_Read_Gyro();
      const float lax = Ax - g_cal_ax;
      const float lay = Ay - g_cal_ay;
      const float laz = Az - g_cal_az;
      nav_imu_integrate_velocity(dt, lax, lay, laz);
      const float dgx = Gx - g_cal_gx;
      const float dgy = Gy - g_cal_gy;
      const float dgz = Gz - g_cal_gz;
      const int still = (fabsf(lax) + fabsf(lay) + fabsf(laz) < ZUPT_ACC_SUM_G) &&
                        (fabsf(dgx) < ZUPT_GYRO_AXIS_DPS) && (fabsf(dgy) < ZUPT_GYRO_AXIS_DPS) &&
                        (fabsf(dgz) < ZUPT_GYRO_AXIS_DPS);
      if (still && nav_zupt_ok)
        kf_update_scalar(0.f, 1.f, 0.f, 1e-4f);
    }
    return;
  }

  float dt_m = (float)((int32_t)(now - nav_tick_at_mall)) * 0.001f;
  if (dt_m < 0.003f || dt_m > 0.5f)
    dt_m = 0.05f;
  nav_tick_at_mall = now;

  const float ds_frame = s_enc - nav_s_enc_at_mall;
  nav_s_enc_at_mall = s_enc;

  const float still_eps = fabsf(NAV_KF_M_PER_AVG_COUNT) * NAV_KF_ENC_STILL_AVG_TICKS;
  const uint8_t enc_still = (fabsf(ds_frame) < still_eps) ? 1U : 0U;
  nav_zupt_ok = enc_still;
  enc_s_prev = s_enc;

  if (enc_still) {
    kf_s = s_enc;
    kf_v = 0.f;
    nav_last_venc_mps = 0.f;
    kf_P00 = 1e-6f;
    kf_P01 = 0.f;
    kf_P10 = 0.f;
    kf_P11 = 1e-3f;
  } else {
    kf_predict(dt);
    const float R_pos = 2e-5f;
    kf_update_scalar(1.f, 0.f, s_enc, R_pos);

    const float v_enc = ds_frame / dt_m;
    nav_last_venc_mps = v_enc;
    const float R_venc = 0.03f;
    kf_update_scalar(0.f, 1.f, v_enc, R_venc);
  }

  if (g_mpu_ready) {
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();
    const float lax = Ax - g_cal_ax;
    const float lay = Ay - g_cal_ay;
    const float laz = Az - g_cal_az;
    nav_imu_integrate_velocity(dt, lax, lay, laz);
    const float dgx = Gx - g_cal_gx;
    const float dgy = Gy - g_cal_gy;
    const float dgz = Gz - g_cal_gz;
    const int still = (fabsf(lax) + fabsf(lay) + fabsf(laz) < ZUPT_ACC_SUM_G) &&
                      (fabsf(dgx) < ZUPT_GYRO_AXIS_DPS) && (fabsf(dgy) < ZUPT_GYRO_AXIS_DPS) &&
                      (fabsf(dgz) < ZUPT_GYRO_AXIS_DPS);
    if (still && nav_zupt_ok)
      kf_update_scalar(0.f, 1.f, 0.f, 1e-4f);
  }

  memcpy(enc_rel_prev, enc_rel, sizeof(enc_rel_prev));
}
