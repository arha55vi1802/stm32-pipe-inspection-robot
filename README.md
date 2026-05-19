# STM32 Pipe Inspection Robot — Multi-Sensor Firmware

Firmware for a pipe-inspection robot: **4-motor control**, **environmental sensors**, **encoder + IMU odometry (Kalman filter)**, and a **live web dashboard** over Ethernet (W5500).

**MCU:** STM32F401RE (Nucleo-F401RE) · **IDE:** STM32CubeIDE · **Language:** C (STM32 HAL)

---

## What this project does

| Feature | Description |
|--------|-------------|
| **Motors** | Yahboom 4-motor driver over UART (`$spd`, `$MAll` encoders) |
| **DHT11** | Temperature & humidity (GPIO + timing driver) |
| **DS18B20** | Temperature (1-Wire style via USART6) |
| **MQ-4** | Gas sensor via ADC + moving average filter |
| **MPXV7002DP** | Differential pressure via ADC, offset zero, status alerts |
| **MPU6050** | IMU on I2C — calibration, dashboard readout, ZUPT support in fusion |
| **Odometry** | 2-state Kalman filter: distance (m) + velocity (m/s), encoder-led |
| **Ethernet** | W5500 (SPI) — HTTP server, browser dashboard on port 80 |
| **Servo** | Continuous rotation servo, TIM3 PWM (50 Hz) |

**Validation:** ~1.56 m physical pipe travel → **~1.60 m** fused distance on dashboard (**~2.6% error**).

---

## Hardware (summary)

- **STM32:** Nucleo-F401RE  
- **Ethernet:** W5500 (SPI2)  
- **Motors:** USART1 → Yahboom driver  
- **IMU:** MPU6050 (I2C1)  
- **DHT11:** PA1  
- **ADC:** MQ4 (CH0), Pressure (CH7)  
- **Servo PWM:** TIM3 CH1 → PA6  
- **DS18B20:** USART6 / PC6  

---

## Project structure (one firmware, many modules)

```
Core/Src/main.c              → Main loop, HTTP dashboard, motors, sensor cache
Core/Src/dht11_driver.c      → DHT11 driver
Core/Src/ds18b20.c           → DS18B20
Core/Src/nav_kf_mpu6050.c    → MPU6050 + Kalman odometry
Drivers/Ethernet_W5500/      → W5500 + socket HTTP
Motor_4_integration.ioc      → CubeMX pin/clock config
```

---

## How to build & flash

1. Open **`Motor_4_integration`** in **STM32CubeIDE**.  
2. Build project (hammer icon).  
3. Flash to Nucleo (Run/Debug).  

---

## Dashboard (browser)

1. Connect PC Ethernet to robot network (often link-local).  
2. Set PC IP on same subnet as firmware (check `wizchip_port.c` / `main.c` for static IP, e.g. `169.254.52.10`).  
3. Open browser: `http://<board-ip>/`  

**Useful URLs:**

- `/` — full dashboard  
- `/?enc_zero=1` — zero encoders  
- `/?pressure_zero=1` — re-zero pressure  
- `/?motor=stop` — stop motors  
- `/?servo=90` — servo stop  

---

## Skills demonstrated

STM32 HAL · ADC · I2C · SPI · UART · PWM · embedded HTTP · sensor drivers · filtering · Kalman fusion · real-time embedded systems

---

## Author / contact

**Your Name** — Embedded Systems  
- GitHub: *(add your profile URL)*  
- Email: *(add your email)*  
- Fiverr: *(add when ready)*  

---

## License

MIT — see `LICENSE` file. STM32 HAL/CMSIS subject to ST license terms.
