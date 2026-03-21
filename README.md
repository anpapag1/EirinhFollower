# EirinhFollower

A line-following robot using QTR-8A reflectance sensors and PID control with automatic tuning capabilities.

## Hardware

- **Microcontroller:** ESP32
- **Sensors:** QTR-8A reflectance sensor array (8 sensors)
- **Motors:** Dual DC motor driver (TB6612 or similar)
- **LED:** Built-in LED for status indication

## Pin Configuration

| Pin | Function |
|-----|----------|
| 2   | LED_BUILTIN |
| 17  | Button (calibration trigger) |
| 19  | Motor A PWM (AIN1) |
| 18  | Motor A direction (AIN2) |
| 21  | Motor B PWM (BIN1) |
| 22  | Motor B direction (BIN2) |
| 23  | Motor driver sleep pin |
| 13, 27, 26, 25, 33, 32, 35, 34 | Sensor array (8 channels) |

## Features

- **PID Line Following:** Uses proportional, integral, and derivative control
- **Auto-Tuning:** Automatic PID tuning using PIDAutotuner library
- **EEPROM Storage:** Calibration data (Kp, Ki, Kd, baseSpeed, blackLineValue) persists across reboots
- **Debug Mode:** Visual sensor reading display via serial monitor

## Getting Started

1. Upload the sketch to your ESP32
2. Open the Serial Monitor at 500000 baud
3. Press 'h' to see available commands

## Serial Commands

| Command | Description |
|---------|-------------|
| `p <value>` | Set proportional gain |
| `i <value>` | Set integral gain |
| `d <value>` | Set derivative gain |
| `s <value>` | Set base speed |
| `t` | Toggle robot state (enable/disable motors) |
| `g` | Toggle debug mode |
| `c` | Calibrate PID values (auto-tuning) |
| `r` | Reset calibration to defaults |
| `h` | Display help |

## Usage

### Manual Control
1. Short press the button to start line following
2. Use serial commands to adjust PID parameters in real-time

### Auto-Tuning (Calibration)
1. Long press the button (>2 seconds) to start auto-tuning
2. The robot will move back and forth over the line
3. When complete, calibration is automatically saved to EEPROM
4. The LED will blink twice to indicate completion

### Debug Mode
When enabled (`g` command), the serial monitor displays:
- Sensor state (▢ = on line, _ = off line)
- Position value
- Left and right motor speeds

## Calibration Data

Calibration values are stored in EEPROM and loaded automatically on startup:
- **Kp:** Proportional gain (default: 10)
- **Ki:** Integral gain (default: 0)
- **Kd:** Derivative gain (default: 100)
- **baseSpeed:** Base motor speed (default: 50)
- **blackLineValue:** Threshold for detecting black line (default: 4090)

Use the `r` command to reset all calibration values to defaults.

## Libraries Required

- [QTRSensors](https://github.com/pololu/qtr-sensors-arduino) - Pololu QTR sensor library
- [PIDAutotuner](https://github.com/pololu/pid-autotuner) - PID auto-tuning library

## License

MIT License
