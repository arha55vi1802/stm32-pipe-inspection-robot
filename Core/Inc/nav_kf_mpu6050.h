#ifndef NAV_KF_MPU6050_H
#define NAV_KF_MPU6050_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void NavKF_Init(void);
void NavKF_Step(const int32_t enc_rel[4]);
void NavKF_ResetWithEncoders(const int32_t enc_rel[4]);

float NavKF_Get_distance_m(void);
float NavKF_Get_velocity_mps(void);
float NavKF_Get_encoder_distance_m(const int32_t enc_rel[4]);
uint8_t NavKF_Imu_ok(void);
/* Last MPU6050 sample from NavKF_Step: accel in g (±2g scale), gyro in °/s. Zeros if IMU not ready. */
void NavKF_Get_imu(float acc_g[3], float gyro_dps[3]);
/* IMU-only longitudinal velocity estimate (m/s), for display. Not used as a KF measurement. */
float NavKF_Get_imu_velocity_mps(void);

#ifdef __cplusplus
}
#endif

#endif
