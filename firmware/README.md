# Firmware

ESP32-C3 robot firmware using ESP-IDF v5.2 and micro-ROS.

## Setup

1. Install [ESP-IDF v5.2+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/)
2. Clone the [micro-ROS ESP-IDF component](https://github.com/micro-ROS/micro_ros_espidf_component) into `components/`
3. Configure WiFi credentials in `menuconfig`

## Build and Flash

```bash
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

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
