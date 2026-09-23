# ggGridRunner Project Review & Action Plan

**Review Date:** 2026-06-08
**Reviewer:** Hermes Agent
**Project:** Dual-firmware rover (STM32 Blue Pill + Arduino Pro Micro over XBee)

---

## Executive Summary

The project has a **solid dual-firmware architecture** with good CI/CD, clean class separation, and a well-chosen shared packed-struct protocol for AVR↔STM32 communication. Both firmwares build successfully and pass high-severity clang-tidy checks.

However, there are **5 critical bugs** that will prevent safe operation or cause hardware misbehavior. These must be fixed before any on-the-ground test. A further 6 medium issues and 3 low/style items should be addressed before considering the project production-ready.

---

## Critical Issues (Fix Before Driving)

### 1. Drivetrain Motor Mixing Is Completely Broken

**File:** `rover-firmware/src/Drivetrain.cpp`

**Problem:** The header documents `drive(int throttle, int steering)` with `throttle` in `[-100, 100]`, but the controller sends `[0, 1000]` with center `500`.

Current mixing logic:
```cpp
int leftSpeed = throttle + steering;   // e.g. 500 + 0 = 500
analogWrite(_flPin, constrain(leftSpeed, 0, 255));  // → 255 (full speed!)
```

**Impact:**
- At neutral joystick, motors run at ~100% PWM (anything ≥255 clamps to max).
- Throttle has almost no dynamic range.
- Reverse is impossible because negative values are clamped to `0`.

**Action:**
- [ ] Decide on a single motor command range (e.g. signed `-255..255`).
- [ ] Map controller's `0..1000` to the signed range before mixing, OR change the controller to emit a signed range.
- [ ] Add differential steering math: `left = throttle + steering`, `right = throttle - steering`.
- [ ] Cap per-motor values while preserving sign for direction control pins.
- [ ] Update header documentation to match the actual contract.

---

### 2. No Packet Framing / CRC Validation on the XBee Link

**Files:** `shared/Protocol.h`, both `main.cpp` files

**Problem:** Both sides blast raw structs over the serial link with no framing. If a single byte is dropped, the receiver will be permanently misaligned. The `checksum` field exists but is never used:
- `buildControlPacket()` sets `checksum = 0`
- Rover never validates `rxPacket.checksum`
- Controller never validates `txPacket.checksum`

**Impact:**
- A single dropped byte causes permanent packet misalignment.
- No data integrity checks; corrupted commands can drive the motors.

**Action:**
- [ ] Add a start-of-frame marker (e.g. `0xAA`) + length field.
- [ ] Implement a lightweight CRC32 (or CRC16) on both sides.
- [ ] On the rover: implement a non-blocking state machine that searches for the start byte, then verifies CRC before acting.
- [ ] Consider a SLIP or COBS framing layer if packet sizes grow.

---

### 3. Rover Has No Command-Timeout Failsafe

**File:** `rover-firmware/src/main.cpp`

**Problem:** If the XBee link drops while the rover is moving, `Serial1.available()` returns `0`, `drive()` is never called again, and the motors keep spinning at the last commanded speed.

**Impact:**
- Runaway rover if the controller link drops.
- Potential hardware damage or safety hazard.

**Action:**
- [ ] Add a `lastRxMs` timestamp updated on every valid packet receipt.
- [ ] In `loop()`, check `if (millis() - lastRxMs > 300) drive.stop();`.
- [ ] Reset `lastRxMs` only when a *valid* (synced + CRC-verified) packet is received.

---

### 4. ASCII Error Messages on the Binary Serial Link

**File:** `rover-firmware/src/main.cpp`

**Problem:** During `setup()`:
```cpp
if (!heading.begin()) {
    Serial1.println("IMU Init Failed");  // Sends ASCII into the binary packet stream
}
```

**Impact:**
- If any peripheral fails during boot, ASCII text is injected into the XBee stream.
- This desynchronizes the controller's binary packet parsing and causes garbage `ControlPacket` values for several frames.

**Action:**
- [ ] Remove all `Serial1.println()` calls from the rover firmware.
- [ ] If debug logging is needed, use a separate USB debug serial (`Serial` on the STM32) if available.
- [ ] If `Serial1` must be shared, prefix debug messages with a known marker and strip them on the receiver, or log to a debug buffer instead.

---

### 5. DiagnosticView Renders Off-Screen / Wrong Display Size

**Files:** `controller-firmware/include/DiagnosticView.h`, `controller-firmware/src/DiagnosticView.cpp`

**Problem:** The constructor declares `kHeight = 32`, but `render()` draws **8 lines** of text (8 px each = 64 px). The `Adafruit_SSD1306` buffer is only 512 bytes (128×32). Lines 4–7 write past the buffer into adjacent RAM.

**Impact:**
- Memory corruption in the controller firmware.
- CLAUDE.md says the hardware is a 128×64 OLED, but the code says 32.

**Action:**
- [ ] Verify the actual OLED hardware (128×32 or 128×64?).
- [ ] If 128×64: change `kHeight` to `64` and verify the buffer allocation.
- [ ] If 128×32: redesign the layout to fit 4 lines maximum and remove the overflow.

---

## Medium Issues (Fix Soon)

### 6. CRC32 Is Never Computed

**File:** `controller-firmware/src/main.cpp`

**Problem:** The `checksum` field in `ControlPacket` is hardcoded to `0`. The rover ignores it.

**Action:**
- [ ] Implement a CRC32 function (or use a lightweight CRC16/Adler32) in the shared code.
- [ ] Compute CRC in `buildControlPacket()` before transmission.
- [ ] Verify CRC on the rover before accepting the packet.

---

### 7. `Serial1.readBytes` Blocks the Main Loop

**File:** `rover-firmware/src/main.cpp`

**Problem:**
```cpp
if (Serial1.available() >= sizeof(ControlPacket)) {
    Serial1.readBytes((uint8_t*)&rxPacket, sizeof(ControlPacket));
}
```

If the stream is corrupted, `readBytes` will wait for the full 10 bytes. On the STM32 Arduino core, the default timeout is 1000 ms, which can stall the 10 Hz loop.

**Action:**
- [ ] Replace `readBytes` with a non-blocking ring buffer or state machine.
- [ ] Implement a simple RX parser: search for start byte → read length → verify CRC → dispatch.

---

### 8. PS4 Disconnect Detection Can False-Trigger

**File:** `controller-firmware/src/PS4Interface.cpp`

**Problem:** `_lastChangeMs` is only updated when `accel_x` or `accel_y` changes between consecutive polls. If the controller is held very still, the cached report from the Hobbytronics adapter may return identical accelerometer bytes for a few polls (gravity is constant). The 200 ms stale window is only ~4 polls at 50 Hz.

**Impact:**
- A brief string of identical values engages the failsafe and snaps steering to `0`.

**Action:**
- [ ] Use a broader heartbeat source (e.g. any button or joystick change, not just accel).
- [ ] Widen the stale window to `500–1000 ms` to tolerate brief static periods.
- [ ] Or add a periodic heartbeat/ping from the Hobbytronics adapter if supported.

---

### 9. Rover Telemetry Loop Runs at Maximum CPU Speed

**File:** `rover-firmware/src/main.cpp`

**Problem:** The `loop()` has no delay, so it continuously reads the BME280 and ToF over I2C and hammers `Serial1.available()`. Sensors are polled at thousands of Hz while telemetry is only sent at 10 Hz.

**Impact:**
- Wasted CPU cycles and I2C bus bandwidth.
- Potential I2C bus contention.

**Action:**
- [ ] Structure the loop with the same 10 Hz timing as the telemetry TX.
- [ ] Gate sensor reads so they only happen once per telemetry frame.
- [ ] Use `delay()` or `millis()`-based scheduling to yield CPU time.

---

### 10. ToF Error Values Are Not Checked

**File:** `rover-firmware/src/main.cpp`

**Problem:** `Adafruit_VL53L0X::readRange()` returns `uint16_t`, but can return `0xFFFF` or `0` on error/timeout. `txPacket.distance` is `uint32_t`. If the sensor times out, you will transmit `65535` mm as a valid distance.

**Action:**
- [ ] Check `tof.timeoutOccurred()` before reading range.
- [ ] Validate the range against the sensor's max valid range (e.g. 1200 mm).
- [ ] Set `txPacket.distance` to a sentinel value (e.g. `0` or `UINT32_MAX`) on error.

---

## Low / Style Issues

### 11. Clang-Tidy Warnings Are Numerous

**Files:** All `.cpp` files

**Problem:** CI passes (no high-severity defects), but there are **105 medium warnings** across both firmwares:
- Magic numbers everywhere (`500`, `128`, `100`, `0x76`, etc.)
- Missing `const` on getters (`getYaw()`, `getLeftTicks()`, etc.)
- Trailing return types not used (despite `modernize-use-trailing-return-type` check)

**Action:**
- [ ] Run `pio check -d rover-firmware` and `pio check -d controller-firmware`.
- [ ] Batch-fix const-correctness on all getters.
- [ ] Extract magic numbers into `constexpr` constants in a `Constants.h` or inside relevant classes.
- [ ] Decide whether to adopt trailing return types or disable that check.

---

### 12. Flash Space Is Tight on the Rover

**File:** `rover-firmware/platformio.ini`

**Problem:** Build output shows **89.5% flash used** (58.7 KB / 65.5 KB). The DMP library is large. Only ~7 KB remains for Odometry implementation, additional safety logic, and debugging.

**Action:**
- [ ] Monitor size after every feature addition.
- [ ] If space runs out, consider dropping `-Wl,-u,_printf_float` if float printing is not needed.
- [ ] Evaluate whether DMP quaternion features can be disabled to save space.

---

### 13. Controller Board Mismatch in Documentation

**Files:** `CLAUDE.md`, `controller-firmware/platformio.ini`

**Problem:** The CLAUDE.md says the controller is an **8 MHz** Pro Micro, but `platformio.ini` uses `sparkfun_promicro16` (16 MHz). The build confirms 16 MHz.

**Action:**
- [ ] Update `CLAUDE.md` to reflect the actual 16 MHz hardware configuration.

---

## Recommended Implementation Order

1. **Fix Critical #5** (OLED height) — prevents memory corruption.
2. **Fix Critical #1** (drivetrain mapping) — prevents runaway motors at neutral.
3. **Fix Critical #3** (rover failsafe) — prevents runaway on link drop.
4. **Fix Critical #4** (ASCII on serial) — prevents protocol desync.
5. **Fix Critical #2** (packet framing + CRC) — makes the link robust.
6. **Fix Medium #7** (non-blocking RX) — complements the framing fix.
7. **Fix Medium #6** (CRC computation) — completes the integrity layer.
8. **Fix Medium #8** (PS4 false-trigger) — improves driveability.
9. **Fix Medium #9** (sensor rate limiting) — cleans up the loop timing.
10. **Fix Medium #10** (ToF error checking) — improves telemetry quality.
11. **Fix Low #11** (clang-tidy warnings) — improves maintainability.
12. **Fix Low #13** (doc mismatch) — keeps documentation accurate.
13. **Fix Low #12** (flash monitoring) — ongoing vigilance.

---

## Build Verification Checklist

After each batch of changes:

- [ ] `pio run -d rover-firmware` — builds without errors.
- [ ] `pio run -d controller-firmware` — builds without errors.
- [ ] `pio check -d rover-firmware --fail-on-defect high` — passes.
- [ ] `pio check -d controller-firmware --fail-on-defect high` — passes.
- [ ] Verify rover flash size remains < 95% (target: < 90%).
- [ ] Verify controller flash size remains < 85% (target: < 80%).

---

## Architecture Notes for Future Work

| Component | Status | Notes |
|-----------|--------|-------|
| `Drivetrain` | ⚠️ Broken mixing | Needs signed mapping + direction pins |
| `HeadingSystem` | ✅ Functional | DMP at 10 Hz, stable angles |
| `Odometry` | ⏳ Incomplete | Timer encoder mode not implemented |
| `PS4Interface` | ✅ Mostly functional | False-trigger risk on still controller |
| `Dashboard` | ⚠️ Buffer overflow | Height mismatch (32 vs 64) |
| `Protocol` | ⚠️ Unframed | Needs sync marker + CRC |

---

*End of Review*
