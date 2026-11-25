# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.1.0] - 2026-11-25

First public release of bare-metal STM32F405 flight controller firmware written from scratch.

### Added
- Startup, 168 MHz clock tree, GPIO/SPI/UART drivers, MPU6000 and ICM-42688-P gyro drivers
- DShot600 over TIM8 + DMA burst, CRSF/ELRS receiver parser with telemetry
- Madgwick and error-state EKF attitude estimation, selectable at runtime
- Cascaded angle to rate PID with feedforward, anti-windup and TPA; quad-X mixer with airmode; failsafe
- Closed-loop simulator: roll-rate step t90 = 41 ms; a 29 deg gust settles under 2 deg within 1 s (simulation only)

[0.1.0]: https://github.com/coopersimpson101-debug/skyforge-fc/releases/tag/v0.1.0
