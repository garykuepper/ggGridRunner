# ggGridRunner Pinouts

Single source of truth for every electrical connection on both MCUs. Update this doc **in the same commit** as any firmware change that adds, moves, or repurposes a pin.

**Companion doc:** [docs/hardware.md](docs/hardware.md) — what the hardware *is* (BOM, ratings, I2C address plan). This doc is what each pin is *wired to*. Where the two disagree, see the **Firmware ↔ spec discrepancies** table in `hardware.md`.

**Conventions**
- MCU-native names use the part datasheet (e.g. `PA0`, `PD2`).
- Board labels use what's silk-screened on the dev board (e.g. Pro Micro `D0/RX1`).
- "TBD" = pin not yet chosen in firmware. "TODO" = chosen but driver code incomplete.
- Status: `live` = wired and working, `planned` = code references it but hardware not validated, `unused` = reserved/spare.

---

## Rover — STM32F103C8 "Blue Pill"

Clock: 72 MHz internal. Logic level: 3.3 V (NOT 5 V tolerant on analog/ADC pins).

### Power
| Pin | Net | Notes |
|---|---|---|
| `VBAT` | — | unused |
| `3V3` | 3.3 V rail | powers IMU / BME280 / VL53L0X |
| `5V` | 5 V rail | powers motor-driver logic; sourced from regulator on power board |
| `GND` | ground | star-ground at power board |

### Motor PWM (Drivetrain)
Defined in [rover-firmware/src/main.cpp:16-19](rover-firmware/src/main.cpp#L16-L19).

| MCU pin | Board pin | Net | Dir | Driver / Notes |
|---|---|---|---|---|
| `PA0` | A0 | `MOTOR_FL_PWM` | OUT | Front-Left motor PWM. TIM2_CH1 — conflicts with encoder use; resolve before Phase 1 odometry. |
| `PA1` | A1 | `MOTOR_FR_PWM` | OUT | Front-Right motor PWM. TIM2_CH2. |
| `PA2` | A2 | `MOTOR_RL_PWM` | OUT | Rear-Left motor PWM. TIM2_CH3. |
| `PA3` | A3 | `MOTOR_RR_PWM` | OUT | Rear-Right motor PWM. TIM2_CH4. |

> **Conflict warning:** PA0–PA3 are also the only TIM2 encoder-mode inputs. Phase 1 odometry (`Odometry::begin()`) will need to move motor PWM to TIM3 (PA6/PA7/PB0/PB1) or use TIM4 for encoders. See [docs/roadmap.md](docs/roadmap.md) Phase 1.

### I2C bus 1 (sensors)
Default `Wire` instance.

| MCU pin | Board pin | Function | Devices on bus |
|---|---|---|---|
| `PB6` | B6 | I2C1_SCL | BME280 @ `0x76`, VL53L0X @ `0x29` (default), MPU-9250 @ `0x68` (DMP) |
| `PB7` | B7 | I2C1_SDA | (same) |

External 4.7 kΩ pull-ups to 3.3 V required (Blue Pill has none).

### UART1 — XBee telemetry link
| MCU pin | Board pin | Function | Net |
|---|---|---|---|
| `PA9` | A9 | USART1_TX | → XBee DIN |
| `PA10` | A10 | USART1_RX | ← XBee DOUT |

Baud 57600, 8-N-1. `Serial1` in firmware. See [[project-controller-xbee-wiring]] for the matching controller side.

### Wheel encoders — TBD
`Odometry` is a skeleton ([rover-firmware/src/Odometry.cpp:5](rover-firmware/src/Odometry.cpp#L5)). Candidate pins once TIM2 is freed:

| MCU pin | Candidate function | Net |
|---|---|---|
| `PA6` / `PA7` | TIM3_CH1 / CH2 (encoder mode) | left encoder A/B — TBD |
| `PB6` / `PB7` | TIM4_CH1 / CH2 — **conflicts with I2C1**, skip | — |
| `PA15` / `PB3` | TIM2 remap (after PWM relocation) | right encoder A/B — TBD |

### Spare / unused
PA4, PA5, PA8, PB0, PB1, PB10–PB15, PC13 (onboard LED). Reserve PC13 for status indication only.

---

## Controller — SparkFun Pro Micro (ATmega32U4, 16 MHz, 5 V)

> Despite the `sparkfun_promicro16` board name, the build runs at 16 MHz / 5 V. PlatformIO env: [controller-firmware/platformio.ini:1](controller-firmware/platformio.ini#L1).

### Power
| Pin | Net | Notes |
|---|---|---|
| `RAW` | 5–9 V in | from XBee regulator or USB |
| `VCC` | 5 V | OLED & PS4-host adapter logic |
| `GND` | ground | |

### UART1 — XBee command link
See [[project-controller-xbee-wiring]] — load-bearing context (was originally mis-wired to D8/D9).

| MCU pin | Board label | Function | Net |
|---|---|---|---|
| `PD2` | D0 / RX1 | USART1_RX | ← XBee DOUT |
| `PD3` | D1 / TX1 | USART1_TX | → XBee DIN |

Baud 57600. `Serial1` in firmware. **Do not** move to D8/D9 (SoftwareSerial) — it drops bytes during concurrent `Wire` activity at this baud.

### I2C bus (PS4 host + OLED)
| MCU pin | Board label | Function | Devices |
|---|---|---|---|
| `PD1` | D2 / SDA | I2C SDA | Hobbytronics USB Host @ `0x29` ([controller-firmware/include/PS4Interface.h:16](controller-firmware/include/PS4Interface.h#L16)), SSD1306 OLED @ `0x3C` ([controller-firmware/include/DiagnosticView.h:21](controller-firmware/include/DiagnosticView.h#L21)) |
| `PD0` | D3 / SCL | I2C SCL | (same) |

`Wire.setWireTimeout(3000us, reset=true)` is required — see comment in [controller-firmware/src/PS4Interface.cpp:27-30](controller-firmware/src/PS4Interface.cpp#L27-L30) (prevents USB CDC auto-reset from hanging on a stuck SDA).

### Spare / unused
D4–D10, D14–D16, A0–A3. Reserve at least one for a future status LED.

---

## Bus & address summary

Quick cross-reference for "what's on which bus, at what address."

### I2C
| Board | Bus pins | Address | Device |
|---|---|---|---|
| Rover | `PB6/PB7` | `0x29` | VL53L0X ToF |
| Rover | `PB6/PB7` | `0x68` | MPU-9250 IMU (DMP) |
| Rover | `PB6/PB7` | `0x76` | BME280 |
| Controller | `D2/D3` | `0x29` | Hobbytronics USB Host (PS4) |
| Controller | `D2/D3` | `0x3C` | SSD1306 OLED |

> Note: address `0x29` appears on **both** I2C buses but for different devices — never mix the hardware.

### UART
| Link | Endpoint A | Endpoint B | Baud | Format |
|---|---|---|---|---|
| Telemetry/Command radio | Rover `PA9/PA10` (Serial1) | Controller `D0/D1` (Serial1) | 57600 | 8-N-1, binary `ControlPacket`/`TelemetryPacket` |

---

## When you change a pin

1. Update the firmware (`#define` in the relevant `main.cpp` or constructor).
2. Update this file in the **same commit**.
3. If the change affects the wire protocol or hardware topology, add a one-line note to [docs/roadmap.md](docs/roadmap.md) Phase 1.
4. If the change reflects a load-bearing reason (e.g. "must use hardware UART because…"), save a project memory and link it from here with `[[memory-slug]]`.
