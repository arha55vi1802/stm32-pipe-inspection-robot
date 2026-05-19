/**
 * @file dht11_driver.c
 * @brief DHT11 — timing aligned with Desktop/DHT11_sensor (ControllersTech-style).
 *
 * Previous TIM1 + HAL_Delay() driver held the bus high ~3 ms after start (spec: ~20–40 µs)
 * and used timer ticks that were not 1 µs, which often yields checksum errors / N/A on dashboard.
 * This file uses the Cortex-M4 DWT cycle counter for microsecond delays (same idea as reference project).
 */

#include "dht11_driver.h"
#include "stm32f4xx_hal.h"

static GPIO_TypeDef *dht11_gpio_port;
static uint16_t dht11_gpio_pin;

static void dht11_delay_us(uint32_t us)
{
  uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {
  }
}

static void dht11_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static int dht11_wait_until_pin_low(uint32_t timeout_us)
{
  uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * timeout_us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {
    if (HAL_GPIO_ReadPin(dht11_gpio_port, dht11_gpio_pin) == GPIO_PIN_RESET)
      return 0;
  }
  return -1;
}

static void dht11_set_output_pp(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = dht11_gpio_pin;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dht11_gpio_port, &g);
}

static void dht11_set_input_pu(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = dht11_gpio_pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dht11_gpio_port, &g);
}

static void dht11_start(void)
{
  dht11_set_output_pp();
  HAL_GPIO_WritePin(dht11_gpio_port, dht11_gpio_pin, GPIO_PIN_RESET);
  dht11_delay_us(20000); /* DHT11: host low ≥ 18 ms */
  HAL_GPIO_WritePin(dht11_gpio_port, dht11_gpio_pin, GPIO_PIN_SET);
  dht11_delay_us(35); /* host high ~20–40 µs before slave response */
  dht11_set_input_pu();
}

static int dht11_check_response(void)
{
  uint8_t response = 0;

  dht11_delay_us(40);
  if (HAL_GPIO_ReadPin(dht11_gpio_port, dht11_gpio_pin) == GPIO_PIN_RESET) {
    dht11_delay_us(80);
    if (HAL_GPIO_ReadPin(dht11_gpio_port, dht11_gpio_pin) == GPIO_PIN_SET)
      response = 1;
  }
  if (dht11_wait_until_pin_low(5000U) != 0)
    return 0;
  return (int)response;
}

static int dht11_wait_pin_state(GPIO_PinState want, uint32_t timeout_us)
{
  uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * timeout_us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) {
    if (HAL_GPIO_ReadPin(dht11_gpio_port, dht11_gpio_pin) == want)
      return 0;
  }
  return -1;
}

static int dht11_read_byte(uint8_t *out)
{
  uint8_t i = 0;
  for (uint8_t j = 0; j < 8; j++) {
    if (dht11_wait_pin_state(GPIO_PIN_SET, 120U) != 0)
      return -1;
    dht11_delay_us(40);
    if (HAL_GPIO_ReadPin(dht11_gpio_port, dht11_gpio_pin) == GPIO_PIN_RESET)
      i &= (uint8_t)~(1U << (7U - j));
    else
      i |= (uint8_t)(1U << (7U - j));
    if (dht11_wait_pin_state(GPIO_PIN_RESET, 200U) != 0)
      return -1;
  }
  *out = i;
  return 0;
}

void dht11_init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, TIM_HandleTypeDef *tim)
{
  (void)tim; /* legacy: reference project used TIM; this driver uses DWT only */

  dht11_gpio_port = gpio_port;
  dht11_gpio_pin = gpio_pin;

  dht11_dwt_init();
  dht11_set_input_pu();
  HAL_Delay(300); /* DHT11 power-up + longer settle when sharing a board with Ethernet/SPI */
}

void dht11_bus_recover(void)
{
  if (dht11_gpio_port == NULL)
    return;
  dht11_set_input_pu();
  dht11_delay_us(3000); /* ms-scale gaps help after USART6 / W5500 activity */
}

dht11_data dht11_data_read(void)
{
  dht11_data result = {0.0f, 0.0f, true};
  uint8_t b0, b1, b2, b3, b4;

  if (dht11_gpio_port == NULL)
    return result;

  /* Integrated firmware touches many peripherals; always reclaim PA1 before a read. */
  dht11_bus_recover();

  dht11_start();
  if (dht11_check_response() != 1)
    return result;

  if (dht11_read_byte(&b0) != 0)
    return result;
  if (dht11_read_byte(&b1) != 0)
    return result;
  if (dht11_read_byte(&b2) != 0)
    return result;
  if (dht11_read_byte(&b3) != 0)
    return result;
  if (dht11_read_byte(&b4) != 0)
    return result;

  if ((uint8_t)(b0 + b1 + b2 + b3) != b4)
    return result;

  result.humidity = (float)b0 + (float)b1 / 10.0f;
  result.temperature = (float)b2 + (float)b3 / 10.0f;
  result.error = false;
  return result;
}
