/**
 * @file ds18b20.h
 * @brief DS18B20 on USART6 half-duplex — pin PC6 (NOT PA9).
 *
 * Desktop project `temperature_sensor` uses USART1 / PA9. In Motor_4_integration, USART1 is
 * reserved for the motor driver, so DS18B20 DATA must be wired to PC6. GND and 3V3 same as before;
 * add a 4.7 kΩ pull-up from DATA to 3.3 V if the module does not include one.
 */

#ifndef DS18B20_H
#define DS18B20_H

#include "main.h"

int DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
int DS18B20_ReadTemperature(void);

extern int16_t temperature_raw;
extern float temperature;

#endif
