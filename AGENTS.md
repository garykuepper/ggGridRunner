# AGENTS.md

This file provides guidance and project context for AI agents working with code in this repository.

## Project Overview

The **ggGridRunner** project is a modular, object-oriented C++ monorepo consisting of two main firmware components that communicate over an XBee serial radio link using a custom binary-safe protocol:

- **Rover (`rover-firmware/`)**: Targets an **STM32F103C8 ("Blue Pill", ARM Cortex-M3)**. Responsible for motor control (PWM), sensor data collection (MPU-9250 IMU, VL53L0X ToF, BME280 Environment), and broadcasting telemetry at 10Hz.
- **Controller (`controller-firmware/`)**: Targets an **Arduino Pro Micro (ATmega32U4, 8MHz)**. Interfaces with a PS4 DualShock 4 gamepad (via Hobbytronics USB Host adapter over I2C) and displays real-time telemetry on an SSD1306 OLED dashboard. Sends control commands at 20Hz.
- **Shared (`shared/`)**: Header-only protocol included by both firmwares via `-I ../shared` in each `platformio.ini`. Defines `ControlPacket` and `TelemetryPacket` structs.

## Architecture & Technology Stack

- **Language**: C++ (Arduino framework)
- **Build System**: PlatformIO
- **MCU 1 (Rover)**: STM32F103C8 (ARM Cortex-M3)
- **MCU 2 (Controller)**: ATmega32U4 (AVR, 8MHz)
- **Communication**: XBee Serial (57600 baud), I2C for local peripherals.
- **Key Libraries**:
  - SparkFun MPU-9250 DMP (IMU)
  - Adafruit VL53L0X (ToF)
  - Adafruit BME280 (Environmental)
  - Adafruit SSD1306 & GFX (OLED Display)

### Communication Protocol (`shared/Protocol.h`)

Two `#pragma pack(1)` structs ensure identical byte layout on both the 8-bit AVR and 32-bit STM32 architectures:

- **`ControlPacket`** (Controller → Rover): throttle (0–1000), steering (−500–500), mode, flags, CRC32.
- **`TelemetryPacket`** (Rover → Controller): battery voltage, IMU roll/pitch/yaw, encoder counts, temperature, pressure, ToF distance, CRC32.

Any change to these structs must maintain alignment compatibility across both architectures.

### Rover Class Responsibilities

| Class | Responsibility |
|---|---|
| `Drivetrain` | PWM output to 4 motors (FL/FR/RL/RR); simple throttle+steering mixing |
| `HeadingSystem` | MPU-9250 DMP at 10Hz; produces stable Euler angles via FIFO |
| `Odometry` | STM32 hardware-timer encoder counting — **currently incomplete** |

### Controller Class Responsibilities

| Class | Responsibility |
|---|---|
| `PS4Interface` | I2C reads from Hobbytronics USB Host (addr 0x29); maps 8-byte report to throttle/steering — **currently skeleton** |
| `Dashboard` | SSD1306 128×64 OLED; renders battery, yaw, ToF distance, temperature from incoming telemetry |

## Build and Verification Commands

Commands can be executed from the project root or the respective firmware directory.

```powershell
# Build Rover firmware
pio run -d rover-firmware

# Build Controller firmware
pio run -d controller-firmware

# Static analysis (clang-tidy) — fails on high-severity defects
pio check -d rover-firmware --fail-on-defect high
pio check -d controller-firmware --fail-on-defect high

# Clean build artifacts
pio run -d rover-firmware --target clean
pio run -d controller-firmware --target clean

# Upload to hardware (ST-Link / Serial)
pio run -d rover-firmware --target upload
pio run -d controller-firmware --target upload
```

*Note: There is no automated unit test suite; hardware validation is conducted on-device.*

## Development Conventions

- **Code Style**: Google C++ Style (2-space indent, 100-column limit) enforced via `.clang-format`.
- **Naming**:
  - `PascalCase` for Classes (e.g., `Drivetrain`, `HeadingSystem`).
  - `camelCase` for methods and variables (e.g., `getLeftTicks()`, `txPacket`).
  - `snake_case` for filenames (e.g., `main.cpp`).
- **Static Analysis**: `clang-tidy` checks include `bugprone-*`, `modernize-*`, `readability-*`, and `performance-*`.

## Current Status & Known Issues

- **Odometry**: STM32 hardware timer initialization for encoders is currently a TODO in `rover-firmware/src/Odometry.cpp`.
- **PS4Interface**: The I2C interface for the Hobbytronics USB Host is a skeleton and needs full implementation of the 8-byte report mapping.
- **Telemetry**: Battery voltage in `rover-firmware/src/main.cpp` is currently a placeholder constant.
