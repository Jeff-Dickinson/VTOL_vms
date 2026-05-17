# Wiring Reference — VTOL POC

## Overview

Three I2C devices and 8 RC inputs connect to the Raspberry Pi 4B.
Everything else (motors, servos) connects to the PCA9685 PWM board.

```
                         ┌─────────────────────┐
                         │   Raspberry Pi 4B    │
                         │                      │
  ┌──────────┐     I2C   │  GPIO 2 (SDA) ──────┼──┬── BNO085 SDA
  │ BNO085   │◄─────────►│  GPIO 3 (SCL) ──────┼──┼── BNO085 SCL
  │ IMU      │           │                      │  │
  └──────────┘           │                      │  │   ┌──────────┐
                         │                      │  ├── │ PCA9685  │ SDA
  ┌──────────┐     I2C   │                      │  └── │ PWM      │ SCL
  │ PCA9685  │◄─────────►│                      │      │ Driver   │
  │ PWM Drv  │           │                      │      └────┬─────┘
  └──────────┘           │                      │           │ 16 PWM channels
                         │  GPIO 5  (BCM) ──────┼── CH1     │
  ┌──────────┐   Level   │  GPIO 6  (BCM) ──────┼── CH2     ▼
  │ FS-R9B   │  Shifter  │  GPIO 13 (BCM) ──────┼── CH3   Motors
  │ RC Recv  │──(5V→3.3V)│  GPIO 19 (BCM) ──────┼── CH4   & Servos
  │ 8-CH PWM │───────────│  GPIO 26 (BCM) ──────┼── CH5
  └──────────┘           │  GPIO 12 (BCM) ──────┼── CH6
                         │  GPIO 16 (BCM) ──────┼── CH7
                         │  GPIO 20 (BCM) ──────┼── CH8
                         └─────────────────────┘
```

---

## I2C Bus 1 (shared bus, two devices)

Both BNO085 and PCA9685 share I2C bus 1. No conflict — different addresses.

| Pin (BCM) | Pin (Board) | Function | Connects To |
|-----------|-------------|----------|-------------|
| GPIO 2 | Pin 3 | SDA1 | BNO085 SDA **and** PCA9685 SDA |
| GPIO 3 | Pin 5 | SCL1 | BNO085 SCL **and** PCA9685 SCL |

**Pull-ups:** The BNO085 and PCA9685 breakout boards both have on-board pull-ups. If you get I2C errors, you may need to remove one set (desolder the pull-up resistors on one board).

### BNO085 IMU

| BNO085 Pin | Connects To | Notes |
|------------|-------------|-------|
| VIN | Pi 3.3V (Pin 1) | 3.3V supply — the Adafruit board has an onboard regulator that also accepts 5V |
| GND | Pi GND (Pin 6) | |
| SDA | Pi GPIO 2 (Pin 3) | Shared I2C bus |
| SCL | Pi GPIO 3 (Pin 5) | Shared I2C bus |
| RST | Leave unconnected | Optional: tie to a GPIO for hardware reset |
| INT | Leave unconnected | We poll, don't use interrupt |

**I2C address:** 0x4A (default)

### PCA9685 PWM Driver

| PCA9685 Pin | Connects To | Notes |
|-------------|-------------|-------|
| VCC | Pi 3.3V (Pin 1) | Logic power |
| GND | Pi GND (Pin 9) | |
| SDA | Pi GPIO 2 (Pin 3) | Shared I2C bus |
| SCL | Pi GPIO 3 (Pin 5) | Shared I2C bus |
| V+ | 5V BEC output | **Servo power rail** — NOT from the Pi! From the dedicated 5V BEC |
| GND (V+ side) | BEC ground | Common ground with battery/BEC |

**I2C address:** 0x40 (default)

**IMPORTANT:** The V+ screw terminal on the PCA9685 powers the servo rail. Connect your 5V BEC here. Do NOT jumper V+ to VCC — they are separate: VCC is logic power (3.3V from Pi), V+ is servo power (5V from BEC).

---

## PCA9685 → Actuator Wiring

| PCA9685 CH | Actuator | Connector | Wire Colors (typical) |
|------------|----------|-----------|----------------------|
| 0 | ESC Left Motor | 3-pin servo plug | Brn=GND, Red=5V, Org=Signal |
| 1 | ESC Right Motor | 3-pin servo plug | Same |
| 2 | Tilt Servo Left | 3-pin servo plug | Same |
| 3 | Tilt Servo Right | 3-pin servo plug | Same |
| 4 | Aileron Servo Left | 3-pin servo plug | Same |
| 5 | Aileron Servo Right | 3-pin servo plug | Same |
| 6 | Flap Servo Left | 3-pin servo plug | Same |
| 7 | Flap Servo Right | 3-pin servo plug | Same |
| 8 | Rudder Servo | 3-pin servo plug | Same |
| 9-15 | Unused | — | — |

All servos and ESCs plug directly into the PCA9685 breakout headers. Signal on the inside, power in the middle, ground on the outside (check your board's silkscreen).

### ESC Wiring Detail

Each ESC has three connections:

```
Battery ──[XT60]──┬── ESC Left  ──[3 bullet connectors]── Motor Left (CW)
                  │
                  ├── ESC Right ──[3 bullet connectors]── Motor Right (CCW)
                  │
                  └── 5V BEC ── PCA9685 V+ rail
                                Pi 5V (Pin 2/4) if BEC is shared
```

| ESC Wire | Connects To |
|----------|-------------|
| Signal (white/orange) | PCA9685 CH0 or CH1 signal pin |
| 5V (red) | **Cut or leave disconnected** if using separate BEC. Otherwise it will backfeed |
| GND (black/brown) | PCA9685 GND rail |
| Battery + (red thick) | Battery + via power distribution |
| Battery - (black thick) | Battery - via power distribution |
| Motor wires (3x) | Motor bullet connectors. Swap any 2 to reverse direction |

**IMPORTANT:** If you're using a separate 5V BEC (recommended), **cut the red 5V wire** on each ESC's servo connector, or remove the pin. Otherwise two BECs will fight each other on the servo rail.

---

## RC Receiver → Level Shifter → Pi GPIO

The FS-R9B outputs 5V PWM signals. The Pi GPIO is 3.3V. You **must** use a level shifter or you will damage the Pi.

```
FS-R9B CH1 ──► Level Shifter HV1 ──► LV1 ──► Pi GPIO 5  (Pin 29)
FS-R9B CH2 ──► Level Shifter HV2 ──► LV2 ──► Pi GPIO 6  (Pin 31)
FS-R9B CH3 ──► Level Shifter HV3 ──► LV3 ──► Pi GPIO 13 (Pin 33)
FS-R9B CH4 ──► Level Shifter HV4 ──► LV4 ──► Pi GPIO 19 (Pin 35)
FS-R9B CH5 ──► Level Shifter HV5 ──► LV5 ──► Pi GPIO 26 (Pin 37)
FS-R9B CH6 ──► Level Shifter HV6 ──► LV6 ──► Pi GPIO 12 (Pin 32)
FS-R9B CH7 ──► Level Shifter HV7 ──► LV7 ──► Pi GPIO 16 (Pin 36)
FS-R9B CH8 ──► Level Shifter HV8 ──► LV8 ──► Pi GPIO 20 (Pin 38)
```

### Level Shifter (TXS0108E) Connections

| TXS0108E Pin | Connects To |
|-------------|-------------|
| VA (low-voltage ref) | Pi 3.3V (Pin 17) |
| VB (high-voltage ref) | Receiver 5V (from FS-R9B) |
| GND | Common ground |
| A1-A8 (low side) | Pi GPIO pins (see table above) |
| B1-B8 (high side) | FS-R9B CH1-CH8 signal pins |
| OE (output enable) | Tie to VA (3.3V) to enable |

### RC Channel Assignment

| FS-R9B CH | GPIO (BCM) | Board Pin | Function |
|-----------|------------|-----------|----------|
| CH1 | 5 | 29 | Roll (right stick X) |
| CH2 | 6 | 31 | Pitch (right stick Y) |
| CH3 | 13 | 33 | Throttle (left stick Y) |
| CH4 | 19 | 35 | Yaw (left stick X) |
| CH5 | 26 | 37 | Flight mode (3-pos switch) |
| CH6 | 12 | 32 | Tilt override (pot) |
| CH7 | 16 | 36 | Flaps (2-pos switch) |
| CH8 | 20 | 38 | Kill switch (2-pos switch) |

---

## Power Distribution

```
         ┌──────────────────────────────────────────┐
         │          3S LiPo Battery (11.1V)         │
         │              XT60 connector               │
         └───────────┬──────────┬───────────────────┘
                     │          │
              ┌──────┴───┐  ┌──┴──────────┐
              │ ESC Left │  │ ESC Right   │
              │ 30A      │  │ 30A         │
              └──────────┘  └─────────────┘
                     │
              ┌──────┴───────┐
              │ 5V BEC       │
              │ (3A min)     │
              └──┬───────┬───┘
                 │       │
                 ▼       ▼
            PCA9685    Pi 5V
            V+ rail    (Pin 2/4)
```

### Ground Bus

**All grounds must be connected together:**
- Battery GND
- ESC GND (both)
- BEC GND
- Pi GND (Pin 6, 9, 14, 20, 25, 30, 34, 39 — any of these)
- PCA9685 GND
- BNO085 GND
- Level shifter GND
- FS-R9B GND

A missing ground connection is the #1 cause of I2C errors and servo glitches.

---

## Raspberry Pi 4B GPIO Header Quick Reference

Only the pins used by this project:

```
                    3.3V [ 1] [ 2] 5V (from BEC)
          I2C SDA GPIO2 [ 3] [ 4] 5V
          I2C SCL GPIO3 [ 5] [ 6] GND
                        [ 7] [ 8]
                    GND [ 9] [10]
                        [11] [12] GPIO12 ← RC CH6
                        [13] [14] GND
                        [15] [16] GPIO16 ← RC CH7
         3.3V (for LVS) [17] [18]
                        [19] [20] GND
                        [21] [22]
                        [23] [24]
                    GND [25] [26]
                        [27] [28]
          RC CH1 GPIO5  [29] [30] GND
          RC CH2 GPIO6  [31] [32] GPIO12 ← RC CH6
          RC CH3 GPIO13 [33] [34] GND
          RC CH4 GPIO19 [35] [36] GPIO16 ← RC CH7
          RC CH5 GPIO26 [37] [38] GPIO20 ← RC CH8
                    GND [39] [40]
```

---

## Verification Steps (after wiring)

1. **I2C scan** — verify both devices are visible:
   ```bash
   sudo i2cdetect -y 1
   ```
   Should show `40` (PCA9685) and `4a` (BNO085).

2. **IMU test:**
   ```bash
   cd ~/vms && source ~/vms-env/bin/activate
   sudo python3 tools/calibrate_imu.py
   ```

3. **Servo test** (no props!):
   ```bash
   sudo python3 tools/servo_test.py
   ```

4. **RC test:**
   ```bash
   sudo python3 tools/rc_monitor.py
   ```
   Move sticks and flip switches — values should change between 1000-2000.
