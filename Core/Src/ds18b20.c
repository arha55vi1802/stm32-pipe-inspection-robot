/**
 * @file ds18b20.c
 * @brief DS18B20 on USART6 half-duplex (PC6).
 *
 * Each 1-Wire bit is one UART byte at 115200. For READ slots the slave pulls low during our
 * 0xFF transmit — the echo must be sampled per byte (TX then RX for each bit). Sending 8 bytes
 * then calling Receive(8) misses the window on half-duplex; your temp_sensor project avoids that
 * by using DMA RX before TX. Here we use an equivalent per-bit TX/RX sequence.
 *
 * DS18 DATA = PC6 (USART6). Motor driver uses USART1 (PA9/PA10) — do not wire DS18 to PA9.
 */

#include "ds18b20.h"

extern UART_HandleTypeDef huart6;

int16_t temperature_raw = 0;
float temperature = 0.0f;

static void uart6_flush_rx(void)
{
  volatile uint32_t tmp;
  while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
    tmp = (uint32_t)READ_REG(huart6.Instance->DR);
    (void)tmp;
  }
  __HAL_UART_CLEAR_OREFLAG(&huart6);
}

static void uart6_init(uint32_t baudrate)
{
  HAL_UART_DeInit(&huart6);
  huart6.Instance = USART6;
  huart6.Init.BaudRate = baudrate;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart6);
  HAL_HalfDuplex_Init(&huart6);
  uart6_flush_rx();
  HAL_Delay(10);
}

int DS18B20_Start(void)
{
  uint8_t data = 0xF0;
  uart6_init(9600);
  if (HAL_UART_Transmit(&huart6, &data, 1, 120) != HAL_OK)
    return -1;
  if (HAL_UART_Receive(&huart6, &data, 1, 120) != HAL_OK)
    return -1;
  uart6_init(115200);
  if (data == 0xF0)
    return -2;
  return 1;
}

void DS18B20_Write(uint8_t data)
{
  for (int i = 0; i < 8; i++) {
    uint8_t ch = (data & (1 << i)) ? (uint8_t)0xFF : (uint8_t)0x00;
    (void)HAL_UART_Transmit(&huart6, &ch, 1, 200);
  }
}

/* One 1-Wire read bit = one 0xFF UART slot; must RX immediately after each TX on half-duplex. */
static int DS18B20_ReadByte(uint8_t *out)
{
  uint8_t value = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t tx = 0xFF;
    uint8_t rx = 0;
    if (HAL_UART_Transmit(&huart6, &tx, 1, 200) != HAL_OK)
      return -1;
    if (HAL_UART_Receive(&huart6, &rx, 1, 400) != HAL_OK)
      return -1;
    if (rx == 0xFF)
      value |= (uint8_t)(1U << i);
  }
  HAL_Delay(2);
  *out = value;
  return 0;
}

int DS18B20_ReadTemperature(void)
{
  uint8_t temp_lsb, temp_msb;

  if (DS18B20_Start() != 1)
    return -1;
  HAL_Delay(5);
  DS18B20_Write(0xCC);
  HAL_Delay(5);
  DS18B20_Write(0x44);
  HAL_Delay(750);

  if (DS18B20_Start() != 1)
    return -1;
  HAL_Delay(5);
  DS18B20_Write(0xCC);
  HAL_Delay(5);
  DS18B20_Write(0xBE);
  HAL_Delay(5);

  if (DS18B20_ReadByte(&temp_lsb) != 0)
    return -1;
  if (DS18B20_ReadByte(&temp_msb) != 0)
    return -1;

  temperature_raw = (int16_t)((uint16_t)((uint16_t)temp_msb << 8) | temp_lsb);
  temperature = (float)temperature_raw * 0.0625f;
  return 0;
}
