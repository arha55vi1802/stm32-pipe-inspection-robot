/**
 * @file DHT11 Driver
 * @brief DHT11 humidity and temperature sensor driver (integrated project)
 */

#ifndef DHT11_DRIVER_H
#define DHT11_DRIVER_H

#include "main.h"
#include <stdbool.h>

typedef struct {
    float humidity;
    float temperature;
    bool error;
} dht11_data;

/* tim: optional legacy parameter (unused); DHT11 timing uses DWT microsecond delays. */
void dht11_init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, TIM_HandleTypeDef *tim);
/* Force PA1 back to idle input+pull-up (call after USART6/DS18 activity or noisy buses). */
void dht11_bus_recover(void);
dht11_data dht11_data_read(void);

#endif
