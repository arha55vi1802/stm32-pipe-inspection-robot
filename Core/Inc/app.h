/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app.h
  * @brief          : Application header file for motor control
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Motor Type Definition */
typedef enum {
    MOTOR_TYPE_1 = 1,
    MOTOR_TYPE_2 = 2,
    MOTOR_TYPE_3 = 3,
    MOTOR_TYPE_4 = 4
} motor_type_t;

/* Function Prototypes */
void Contrl_Speed(int16_t M1_speed, int16_t M2_speed, int16_t M3_speed, int16_t M4_speed);
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length);
void Send_Motor_U8(uint8_t Data);
void send_motor_type(motor_type_t data);
void send_motor_deadzone(uint16_t data);
void send_pulse_line(uint16_t data);
void send_pulse_phase(uint16_t data);
void send_wheel_diameter(float data);
void send_upload_data(bool ALLEncoder_Switch, bool TenEncoder_Switch, bool Speed_Switch);
void MotorDriver_UartRxPoll(void);
void MotorUartIRQ_Byte(uint8_t c);

extern int Encoder_Now[4];
extern int Encoder_Offset[4];
extern int32_t motor_encoder_mtep[4];
extern int32_t motor_encoder_mspd[4];
extern volatile uint32_t motor_uart_rx_bytes;
extern volatile uint32_t motor_uart_rx_frames;
extern char motor_uart_last_line[96];

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
