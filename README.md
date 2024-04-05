# Secret Box

## Objective

Create a Secret Box with sensors for detecting opening/falling, supplemented by an alarm system, LCD display, and Telegram Bot notifications.

## Hardware

- ESP32 microcontroller
- MPU6050 sensor (movement/tilt detection)
- Ultrasonic sensor (proximity/open/close detection)
- LCD display
- Speaker or buzzer (for alarm)
- Wi-Fi module (for connectivity)

## Programming

C++ in Arduino IDE

## Modules

- **App**: Coordinates system, integrates sensor data, controls LCD and alarm.
- **MPU6050**: Monitors movement and tilt.
- **Ultrasonic**: Detects proximity, indicates box state.
- **LCD**: Provides visual feedback.
- **Telegram Bot**: Sends notifications via Telegram.

## Advantages

- Facilitates independent development and integration.
- Enhances scalability and maintainability.

## Future Improvements

- Additional sensors for security/functionality.
- Enhanced Telegram Bot features.
- Improved user interface.

## Version Control

Utilize Git for managing revisions and collaboration.
Each module may have its own repository.
