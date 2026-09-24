# ggGridRunner Development Roadmap

This document outlines the phased development strategy for the Semi-Autonomous Rover System.
Each phase includes a table of tasks organized by firmware target, class, and detail.

**Difficulty key:**
- `Quick` — isolated change, under an hour
- `Moderate` — a few hours, clear path
- `Involved` — multi-file or requires hardware testing to validate
- `Research` — needs investigation or datasheet work before coding

**Status key:**
- `[ ]` — not started
- `[x]` — complete
- `[~]` — blocked: hardware not yet available

---

## Milestones

The phases below are task lists. These milestones are the order they get
done in, with effort estimates (at ~5 bench hrs/week, no calendar dates).
Cross-project sequencing and the gate this opens for ggSkybit live in the
[ggSkyGrid plan](https://github.com/garykuepper/ggSkyGrid/blob/main/PLAN.md).
A milestone is done when its clip exists.

| Order | Milestone | Needs chassis | Effort | Weeks | Done when | Clip | Covers |
|---|-----------|---------------|--------|-------|-----------|------|--------|
| 1 | Motors spin | No | 12–20 h | 3–4 | Blue Pill flashed; motors spin forward and reverse with correct throttle scaling, wheels in the air | Scripted wheel pattern | Phase 1 `Drivetrain` |
| 2 | Safe remote link | No | 20–30 h | 4–6 | PS4 → XBee → motors respond; framed, CRC-checked packets; dead-man's switch; OLED telemetry; real battery voltage | Kill the remote: wheels stop | Phase 1 `PS4Interface`, `Dashboard`; Phase 2 |
| ‖ | Chassis build | n/a | 30–50 h + print time | 6–10 | Suspension designed properly, printed and assembled. Runs in parallel with the two above | Print time-lapse; articulation test | CAD ([hardware.md](hardware.md)) |
| 3 | Drives on remote | Yes | 5–10 h | 1–2 | Electronics installed; driven around on the remote; link timing holds under load | Driving around the house | Phase 2 XBee timing |
| 4 | Heading hold (stretch) | Yes | 15–25 h | 3–5 | Encoders plus IMU hold a straight line | Stays straight when pushed | Phase 1 `Odometry`; Phase 3 `HeadingSystem`; Phase 4 PID heading hold |

To Drives on remote: ~67–110 h, about 4–5 months. Reaching it opens the ggSkybit gate.

---

## Phase 1: Hardware Validation & Low-Level Control
**Goal**: Establish reliable, correctly-scaled low-level control of basic hardware.

| Status | Difficulty | Target | Class | Task | Detail |
|--------|------------|--------|-------|------|--------|
| [ ] | `Quick` | Rover | `Drivetrain` | Fix throttle scaling bug | `drive()` receives protocol values (throttle: 0–1000, steering: −500–500) but clamps directly to 0–255 PWM, losing 75% of resolution. Map 0–1000 → 0–255 before writing PWM outputs. |
| [ ] | `Moderate` | Rover | `Drivetrain` | Add direction control | No reverse currently. Define how throttle maps to forward/reverse (e.g. center at 500, or signed values with a direction flag). |
| [ ] | `Research` | Rover | `Odometry` | Initialize hardware timers | Implement `Odometry::begin()` to configure TIM2/TIM3 in encoder mode. Requires STM32 reference manual for timer register setup. Currently empty. |
| [ ] | `Quick` | Rover | `Odometry` | Read encoder registers | `getLeftTicks()` / `getRightTicks()` return stale member variables. Read directly from STM32 hardware timer registers once timers are configured. |
| [ ] | `Involved` | Rover | `Odometry` | Calibrate distance formula | `getDistance()` uses a placeholder multiplier. Calibrate using measured wheel circumference and encoder CPR; requires on-device testing. |
| [ ] | `Moderate` | Controller | `PS4Interface` | Install USB Host library | Add Hobbytronics USB Host Arduino library to `platformio.ini` `lib_deps`; include the header. Currently commented out. |
| [ ] | `Research` | Controller | `PS4Interface` | Parse I2C gamepad report | Parse the full 8-byte report from I2C address 0x29. Replace placeholder single-byte reads with correct axis/button field mapping per Hobbytronics adapter docs. |
| [ ] | `Quick` | Controller | `PS4Interface` | Implement button support | `isButtonPressed()` is hardcoded `return false`. Map button bits from the I2C report once report layout is known. |
| [ ] | `Moderate` | Controller | `Dashboard` | Validate OLED layout | Verify telemetry fields are readable on the 128×64 display. Adjust field positions, labels, and font sizes as needed. |

---

## Phase 2: Communication & Safety
**Goal**: Robust, error-detected binary link and failsafe mechanisms.

| Status | Difficulty | Target | Class | Task | Detail |
|--------|------------|--------|-------|------|--------|
| [ ] | `Moderate` | Shared | `Protocol.h` | Add packet framing | Prefix each packet with a sync byte (e.g. `0xAA`) so both sides can re-align after a glitch, power cycle, or startup race. |
| [ ] | `Moderate` | Shared | `Protocol.h` | Implement CRC32 | Add a `crc32()` helper in `Protocol.h`. Compute and write checksum before TX; validate and discard corrupted packets on RX. Fields exist in both structs but are never written or checked. |
| [ ] | `Quick` | Shared | `Protocol.h` | Verify struct alignment | Confirm `#pragma pack(1)` produces identical byte layouts on 8-bit AVR and 32-bit STM32. Validate with a `static_assert` on `sizeof` or a manual loopback test. |
| [ ] | `Quick` | Rover | `main.cpp` | Read battery voltage from ADC | Replace hardcoded `batteryVoltage = 12.6f` with a real ADC read through a voltage divider on a spare STM32 analog pin. |
| [ ] | `Involved` | System | XBee link | Validate timing under load | Confirm 20Hz command / 10Hz telemetry loops meet their deadlines under realistic radio conditions (range, interference). |
| [ ] | `Quick` | Rover | `main.cpp` | Dead-man's switch | Stop all motors if no valid `ControlPacket` is received within 500 ms. |
| [ ] | `Quick` | Controller | `Dashboard` | Low-voltage warning | Display a warning on the OLED when telemetry reports battery voltage below a configurable threshold. |

---

## Phase 3: Sensor Fusion & Environment
**Goal**: Integrate spatial awareness and environmental monitoring into the control loop.

| Status | Difficulty | Target | Class | Task | Detail |
|--------|------------|--------|-------|------|--------|
| [ ] | `Involved` | Rover | `HeadingSystem` | Validate DMP stability | Confirm MPU-9250 DMP FIFO drains reliably at 10 Hz and that yaw drift is acceptable for heading-hold use. Requires on-device testing over time. |
| [~] | `Moderate` | Rover | `main.cpp` | Collision avoidance | Use VL53L0X ToF distance to reduce throttle or stop when an obstacle is within a configurable range threshold. Requires VL53L0X module. |
| [~] | `Quick` | Rover | `main.cpp` | Write BME280 pressure to telemetry | Sensor is initialized but pressure is never written to `TelemetryPacket`. Read and populate the field each loop. Requires BME280 module. |
| [~] | `Moderate` | Controller | `Dashboard` | Environmental display | Add visual indicators for temperature, pressure, and ToF distance — consider bar graphs or threshold-based highlights. Requires BME280 and VL53L0X modules. |

---

## Phase 4: Autonomy & Advanced Features
**Goal**: Move beyond manual remote control into semi-autonomous operation.

| Status | Difficulty | Target | Class | Task | Detail |
|--------|------------|--------|-------|------|--------|
| [ ] | `Involved` | Rover | `Drivetrain` / `HeadingSystem` | PID heading hold | Use IMU yaw feedback to auto-correct steering while throttle > 0. Requires PID tuning on hardware. |
| [ ] | `Involved` | Rover | `Odometry` | Waypoint navigation | Implement dead-reckoning position tracking using encoder counts to navigate to a target (x, y) coordinate. |
| [ ] | `Research` | System | Both | Record & Replay | Log `ControlPacket` stream to EEPROM or external flash on the rover; replay on command. Needs storage capacity planning. |
| [ ] | `Moderate` | Controller | `Dashboard` / `PS4Interface` | Mode-select UI | Add a dashboard mode indicator and use gamepad buttons to switch between Manual, Heading-Hold, and Waypoint modes. |
