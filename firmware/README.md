# Firmware

ESP32-C3 robot firmware using ESP-IDF v5.2 with plain UDP communication.

micro-ROS integration is planned as future work once ESP-IDF compatibility issues are resolved.

## Setup

1. Install [ESP-IDF v5.2+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/)
2. Configure WiFi credentials and server IP in `menuconfig` or source code

## Build and Flash

```bash
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

## UDP Protocol

| Command | Direction | Format | Response |
|---------|-----------|--------|----------|
| Move | Server → Robot | `M <linear> <angular>` | `OK` |
| Stop | Server → Robot | `S` | `OK` |
| LED on/off | Server → Robot | `L1` / `L0` | `OK` |
| RGB colour | Server → Robot | `C <r> <g> <b>` | `OK` |
| Display text | Server → Robot | `D <text>` | `OK` |
| Register | Robot → Server | `REG <tag_id> <radius_mm>` | `OK <robot_id>` |
| Heartbeat | Server → Robot | `H` | `OK` |
| Ping | Either | `P` | `PONG` |

## GPIO Pin Assignment (ESP32-C3 SuperMini)

| GPIO | Function              |
|------|-----------------------|
| 0    | DRV8833 IN1 (Motor A) |
| 1    | DRV8833 IN2 (Motor A) |
| 2    | DRV8833 IN3 (Motor B) |
| 3    | DRV8833 IN4 (Motor B) |
| 4    | WS2812B LED data      |
| 5    | TCRT5000 edge (ADC)   |
| 6    | I2C SCL (OLED)        |
| 7    | I2C SDA (OLED)        |
| 8    | Onboard blue LED      |
| 10   | Battery voltage (ADC) |
