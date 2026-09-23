# ggGridRunner — Hardware Specification Sheet

Source of truth for **what hardware exists, what it's rated for, and how it's configured**. Pair with [docs/pinouts.md](docs/pinouts.md) (the source of truth for *where each piece is wired*).

Two MCUs in this project — sections are split by host:

- **[Rover hardware](#rover-hardware)** — everything driven by the STM32 Blue Pill (motors, sensors, chassis, battery).
- **[Controller hardware](#controller-hardware)** — everything on/with the Pro Micro (gamepad, OLED, controller-side XBee).
- **[Inter-MCU link](#inter-mcu-link)** — the radio bridge between the two.
- **[Firmware ↔ spec discrepancies](#firmware--spec-discrepancies)** — rover-side only; controller firmware is locked.

When the firmware and this doc disagree on **rover** hardware, this doc wins. On the **controller** side the firmware is authoritative.

---

## Rover hardware

### Blue Pill microcontroller

| | |
|---|---|
| Board | STM32F103C8T6 Blue Pill |
| Clock | 72 MHz |
| Logic voltage | 3.3V |
| Encoder timers | 4× hardware (TIM1–TIM4) |
| PWM channels | 8 needed (4× motor + 4× servo) |
| I2C bus | 1× shared across all sensors |

### Drive Motors ×4

| | |
|---|---|
| Model | JGA25-370 |
| Voltage | 12V |
| No-load speed | 280 RPM (21.3:1 gearbox) |
| At-load speed | ~220 RPM |
| Stall current | 1.3A per motor |
| Output shaft | 4mm Ø with flat |
| Encoder type | AB Hall effect, 11 PPR |
| Encoder wires | 6-wire — M+, M−, VCC, GND, A, B |
| Counts per wheel rev | 11 × 4 × 21.3 = ~937 |
| Encoder voltage | 3.3V — Blue Pill direct ✓ |

### Motor Drivers ×4 — **open**

ESC/H-bridge choice not locked. Candidates and requirements:

| | |
|---|---|
| Required continuous current | ≥1.5 A per channel (1.15× motor stall) |
| Required supply | 14.4 V nominal, 16.8 V peak (4S battery direct) |
| Required logic | 3.3 V tolerant (Blue Pill direct, no level-shifter) |
| Candidate A | DRV8871 — 3.6 A, IN1/IN2 (PWM + direction, **8 GPIOs total**) |
| Candidate B | TB6612FNG dual — 1.2 A cont. (under-spec at stall), PWMA/AIN1/AIN2 per channel |
| Candidate C | Brushed micro-ESC (RC-style, 1-wire PWM) — simplest pin count but no regen braking |

> Pin budget impact: Candidate A doubles rover PWM/GPIO usage (4 → 8) and forces a TIM2 reshuffle (see [pinouts.md](pinouts.md)). Candidate C keeps the current 1-pin-per-motor scheme.

### Steering Servos ×4 — **model open**

| | |
|---|---|
| Count | 4 total — 1 per wheel (4WIS) |
| Candidate model | MG996R (rejected: MG90S — too weak) |
| Min torque target | ~9 kg·cm @ 4.8V |
| Signal | 50Hz PWM, 1–2ms pulse |
| Gears | Metal preferred |
| Modes | Ackermann / Crab / Spin in place / Straight |

### Wheels

| | |
|---|---|
| Diameter | 75mm |
| Width | 30mm |
| Hub | 12mm hex |
| Type | RC rally — foam inserts |
| Adapter needed | 4mm shaft → 12mm hex |
| Top speed (est.) | ~4 km/h @ 12V PWM |

### Suspension

| | |
|---|---|
| Type | Double wishbone, independent ×4 |
| Upper arm | 42mm |
| Lower arm | 60mm |
| Shock absorbers | 100mm |
| Travel | ~25mm |
| Ride height | 30mm |
| Joints | M3 pin bolts — no ball joints |
| Kingpin | M4 stainless bolt (steering axis) |
| Tie rod | ~55mm |
| Steering arm | ~22mm |

### Chassis

| | |
|---|---|
| Wheelbase | 220mm |
| Track (c-c) | 240mm |
| Width | 120mm |
| Length | 200mm |
| Material | 3D printed ABS |
| Battery bay | 144 × 65 × 36mm (L × W × H) |

> ⚠ Chassis dimensions are design targets — verify in OnShape before printing.

### Battery

| | |
|---|---|
| Configuration | 4S4P — self-assembled |
| Cells | 16× 18650 |
| Nominal voltage | 14.4V |
| Max voltage | 16.8V fully charged |
| Pack dimensions | 144 × 65 × 36mm |
| BMS rating | 30A continuous |
| Motor PWM cap | ~71% duty = 12V equivalent |
| Logic supply | ⚠ Buck converter needed → 5V |
| Est. runtime | >3 hours |

### Rover sensors

**Baseline (required for v1):**

| | |
|---|---|
| IMU | MPU9250 — 9-axis with DMP, I2C |
| Current sensor | INA226 ×4 — per-motor, I2C |

**Future / not baseline** — present in firmware as forward-compat scaffolding; do not treat as required for v1:

| | |
|---|---|
| ToF range | VL53L0X — for collision avoidance (Phase 3) |
| Environment | BME280 — temp/pressure (Phase 3) |

### Rover I2C bus map

Blue Pill `PB6`/`PB7`. Pull-ups: 4.7 kΩ to 3.3 V.

| Device | Address |
|---|---|
| MPU9250 | 0x68 |
| INA226 #1 (FL) | 0x40 |
| INA226 #2 (FR) | 0x41 |
| INA226 #3 (RL) | 0x42 |
| INA226 #4 (RR) | 0x43 |
| VL53L0X (future) | 0x29 |
| BME280 (future) | 0x76 |

### Spare / unused rover hardware

| | |
|---|---|
| JGA25-370 1200RPM | 5× no encoder — spare |
| JGA25-370 280RPM encoder | 2× extra — spare |
| Brushless planetary motors | 4× — TBD |
| MG90S servos | Ordered — too weak for steering |
| Micro brushed ESCs | Various — not used |

### Outstanding rover items

| Item | Status |
|---|---|
| 4mm → 12mm hex adapter | ⚠ Not confirmed ordered |
| Buck converter (16.8V → 5V) | ⚠ Not yet sourced |
| Brushless motors | ❓ Use case TBD |

---

## Controller hardware

### Pro Micro microcontroller

| | |
|---|---|
| Board | SparkFun Pro Micro |
| MCU | ATmega32U4 |
| Clock | 16 MHz |
| Logic voltage | 5 V |
| PlatformIO env | `sparkfun_promicro16` ([controller-firmware/platformio.ini:1](controller-firmware/platformio.ini#L1)) |

### Gamepad input

| | |
|---|---|
| Adapter | Hobbytronics PS4BT USB-Host (I2C slave) |
| Interface | I2C @ `0x29` ([PS4Interface.h:16](controller-firmware/include/PS4Interface.h#L16)) |
| Pairs with | Sony DualShock 4 over Bluetooth |
| Stale-link timeout | 200 ms via accel-byte heartbeat (see comment in [PS4Interface.h:46-50](controller-firmware/include/PS4Interface.h#L46-L50)) |

### Display

| | |
|---|---|
| Module | 0.91" SSD1306 OLED |
| Resolution | 128 × 32 |
| Interface | I2C @ `0x3C` ([DiagnosticView.h:21](controller-firmware/include/DiagnosticView.h#L21)) |
| Render rate | 5 Hz (diagnostic page) |

### Controller I2C bus map

Pro Micro `D2` (SDA) / `D3` (SCL). Pull-ups: 4.7 kΩ to 5 V.

| Device | Address |
|---|---|
| Hobbytronics PS4BT adapter | 0x29 |
| SSD1306 OLED | 0x3C |

> `Wire.setWireTimeout(3000us, reset=true)` is mandatory — see [PS4Interface.cpp:27-30](controller-firmware/src/PS4Interface.cpp#L27-L30). Prevents a stuck SDA from hanging the USB CDC auto-reset path.

### Controller power

| | |
|---|---|
| Source | ❓ TBD — confirm: USB-only during dev, or onboard battery? |
| Pro Micro input | `RAW` accepts 5–9 V, or `VCC` direct 5 V from USB |

### Controller enclosure / handheld

| | |
|---|---|
| Status | ❓ TBD — no spec captured yet |

---

## Inter-MCU link

### XBee radio

| | |
|---|---|
| Endpoints | Rover (USART1 on `PA9`/`PA10`) ↔ Controller (Serial1 on `D0`/`D1`) |
| Baud | 57600, 8-N-1 |
| Wire format | Binary `ControlPacket` (20Hz C→R) / `TelemetryPacket` (10Hz R→C), see [shared/Protocol.h](shared/Protocol.h) |
| Pairing | Pre-paired XBee modules; no runtime negotiation |

---

## Firmware ↔ spec discrepancies

Captured at the time this doc was written; remove rows as the firmware catches up.

**Controller firmware is locked-in (validated 2026-05-24)** — its hardware addresses and wiring are the source of truth. All discrepancies below are rover-side, to be resolved on the rover.

| Area | Spec says | Firmware does | Action |
| --- | --- | --- | --- |
| Drive output | 4 channels, control scheme depends on ESC choice | 1× `analogWrite` per wheel ([rover-firmware/src/Drivetrain.cpp:20-23](rover-firmware/src/Drivetrain.cpp#L20-L23)) — no reverse | **blocked on ESC selection.** If DRV8871 → rewrite for IN1/IN2 pairs + TIM2 reshuffle. If micro-ESC → current 1-PWM scheme is fine, just add direction calibration. |
| Servos | 4× steering, model TBD | no servo code at all | add `Steering` class; reserve 4 PWM pins |
| Per-motor current | INA226 ×4 at `0x40`–`0x43` | not initialized; not in `TelemetryPacket` | add driver + extend `TelemetryPacket` (protocol break — bump version) |
| Battery voltage | 14.4 V nominal / 16.8 V peak (4S4P confirmed) | hardcoded `12.6f` ([rover-firmware/src/main.cpp:66](rover-firmware/src/main.cpp#L66)) | wire ADC divider; update telemetry scale |
| Encoder CPR | 937 counts / wheel-rev | `getDistance()` placeholder `* 0.5f` ([rover-firmware/src/Odometry.cpp:26](rover-firmware/src/Odometry.cpp#L26)) | use real CPR + wheel circumference (75 mm Ø → ~235.6 mm/rev) |
| Logic 5 V rail | "buck converter needed — not yet sourced" | n/a | flagged in [Outstanding rover items](#outstanding-rover-items) — blocks bring-up |
| ToF / BME280 init | future scope (not baseline) | `Adafruit_VL53L0X` + `Adafruit_BME280` initialized & read every loop ([rover-firmware/src/main.cpp:25-26](rover-firmware/src/main.cpp#L25-L26)) | acceptable as forward-compat scaffolding; consider `#ifdef`-gating for v1 firmware footprint |
