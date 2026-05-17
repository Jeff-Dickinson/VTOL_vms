# Bill of Materials — VTOL POC

## Amazon Shopping List

Specific parts to order, with servo sizing rationale.

### Compute & Sensors

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | Raspberry Pi 4 Model B 4GB | [Amazon](https://www.amazon.com/Raspberry-Pi-RPI4-MODBP-4GB-Model-4GB/dp/B09TTNF8BT) | Flight computer. A CanaKit starter kit is fine if you need SD card + power supply |
| 1 | Adafruit BNO085 9-DOF IMU Breakout | [Adafruit](https://www.adafruit.com/product/4754) | STEMMA QT / I2C. Also search "BNO085" on Amazon — availability varies. Adafruit direct is most reliable |
| 1 | 32GB MicroSD Card (A2 rated) | Search Amazon for "Samsung EVO 32GB microSD" | Any Class 10 / A2 card works |

### Power

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 2 | Zeee 3S LiPo 2200mAh 50C XT60 (2-pack) | [Amazon](https://www.amazon.com/Zeee-Vehicles-Airplane-Quadcopter-Helicopter/dp/B0BYNSH6Q7) | 11.1V 3S. One to fly, one to charge. 2200mAh is good for POC — upgrade to 3300mAh later for longer flights |
| 1 | Matek UBEC 5V 3A | Search Amazon for "Matek 5V BEC" or "Hobbywing 5V 6A UBEC" | Separate regulated 5V for Pi + servos. Do NOT rely on ESC BEC |
| 1 | XT60 power distribution harness | Search Amazon for "XT60 parallel harness" or "Matek PDB" | Split battery to 2 ESCs + BEC |
| 1 | LiPo battery voltage alarm | Search Amazon for "lipo voltage alarm buzzer" | ~$5, plugs into balance lead. Beeps at low voltage |

### Propulsion

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | Readytosky 2212 920KV Motors (4-pack) | [Amazon](https://www.amazon.com/Readytosky-Brushless-Motors-Phantom-Quadcopter/dp/B075DD16LK) | Comes as 2 CW + 2 CCW. You need 1 of each (spares are useful). 55g each, 3.175mm shaft |
| 1 | Hobbypower SimonK 30A ESC (4-pack) | [Amazon](https://www.amazon.com/Hobbypower-SimonK-Brushless-Controller-Quadcopter/dp/B00QRR7N32) | SimonK firmware, 2-4S, 5V/2A BEC. You need 2, extras for spares |
| 1 | 1045 Propellers CW/CCW (2 pairs) | [Amazon](https://www.amazon.com/uxcell-Propellers-10x4-5-Self-locking-Quadcopter/dp/B07ZMYTSMG) | 10x4.5" self-locking. Matched to 2212/920KV. Buy extra pairs — props break |

### Servos

**Tilt servos (2x) — DS3225 25kg metal gear:**

These carry the full motor+prop+mount assembly (~90g) through 90° of rotation, against aerodynamic load during transition. You need high torque (20kg-cm+), metal gears, and fast response.

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 2 | DS3225 25kg Metal Gear Digital Servo | [Amazon](https://www.amazon.com/DS3225-Degree-Torque-Digital-Servo/dp/B07ZG9XFGX) | 25kg-cm @ 6V. Metal gears, waterproof. 180° version (not 270°). ~60g each. 25T spline. These are massively overkill on paper for the 90g load, but you want the headroom for aerodynamic forces during transition and the stiffness of metal gears |

**Control surface servos (5x) — MG90S 9g metal gear:**

For a POC-scale airframe (800-1200mm wingspan), 9g micro servos with metal gears are the right size. Plastic-gear SG90s are too fragile for repeated deflections. MG90S gives you 2.0 kg-cm torque at 4.8V, which is sufficient for small foam/balsa control surfaces.

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 2 | MG90S 9g Micro Servo (5-pack) | [Amazon](https://www.amazon.com/Hosyond-Helicopter-Airplane-Control-Compatible/dp/B09V5BR7J5) | 2.0 kg-cm @ 4.8V, metal gears, 9g. You need 5 total (2 aileron + 2 flap + 1 rudder). A 5-pack gives you exactly that, or buy a 10-pack for spares |

**Servo sizing summary:**

| Role | Servo | Torque | Weight | Why |
|------|-------|--------|--------|-----|
| Tilt L/R | DS3225 | 25 kg-cm | 60g | Must resist aero loads on motor nacelle during transition. Metal gears resist stripping under impact |
| Aileron L/R | MG90S | 2.0 kg-cm | 9g | Small control surfaces, low aero loads at POC scale |
| Flap L/R | MG90S | 2.0 kg-cm | 9g | Same as ailerons |
| Rudder | MG90S | 2.0 kg-cm | 9g | Vertical tail surface, low load |

### PWM & Signal

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | PCA9685 16-Channel PWM Servo Driver | [Amazon](https://www.amazon.com/SunFounder-PCA9685-Channel-Arduino-Raspberry/dp/B014KTSMLA) | SunFounder or any PCA9685 board. I2C addr 0x40. Drives all 9 actuators from 2 wires |
| 1 | TXS0108E 8-Channel Logic Level Shifter | [Amazon](https://www.amazon.com/Icstation-TXS0108E-Level-Converter-Bidirectional/dp/B06XWVZHZJ) | 4-pack. You need 1 board (8 channels = 8 RC inputs). Shifts 5V receiver signals to 3.3V Pi GPIO |

### RC Control

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | FlySky FS-i6X 10CH Transmitter + FS-iA6B Receiver | [Amazon](https://www.amazon.com/Flysky-FS-i6X-Transmitter-FS-iA6B-Receiver/dp/B0744DPPL8) | 10-channel TX. The FS-iA6B outputs iBus (serial), not individual PWM. See note below |
| 1 | FlySky FS-R9B 8CH Receiver | [Amazon](https://www.amazon.com/FlySky-Original-FS-R9B-Receiver-Transmitter/dp/B01GTYSSN6) | 8 individual PWM outputs — what the software expects. Pairs with FS-i6X after binding |

**Note on receivers:** The FS-i6X ships with an FS-iA6B (iBus serial output). The VMS software is written for individual PWM channels via pigpio edge callbacks, so you need the FS-R9B as well. You could alternatively modify `rc_input.c` to read iBus serial instead — but individual PWM is simpler for a POC and doesn't require a UART.

### Wiring & Connectors

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | Dupont jumper wire kit (M/M, M/F, F/F) | Search Amazon for "dupont jumper wire kit 120pcs" | For breadboard prototyping and I2C/GPIO connections |
| 1 | Servo extension cable 300mm (10-pack) | Search Amazon for "servo extension cable 300mm" | Route servos to PCA9685 board |
| 1 | 14AWG silicone wire (red+black, 2m each) | Search Amazon for "14AWG silicone wire" | ESC and battery power runs |
| 1 | Heat shrink tubing assortment | Search Amazon for "heat shrink tubing assortment" | |

### Airframe & Linkage

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | Control horn + pushrod kit | Search Amazon for "RC airplane control horn pushrod kit" | Includes horns, clevises, 1.2mm steel pushrods |
| 1 | CA hinges (20-pack) | Search Amazon for "RC airplane CA hinge" | For foam/balsa control surfaces |
| 1 | Velcro battery straps (4-pack) | Search Amazon for "lipo battery strap velcro" | Battery retention |
| 1 | M3 standoff assortment | Search Amazon for "M3 nylon standoff kit" | Mount Pi, PCA9685, IMU |
| 1 | Vibration dampening gel pads | Search Amazon for "flight controller vibration dampening pads" | Under IMU mount — critical for clean sensor data |

### Safety & Test

| Qty | Item | Link | Notes |
|-----|------|------|-------|
| 1 | LiPo safe bag (large) | Search Amazon for "lipo safe bag large" | For charging and storage |
| 1 | Safety glasses | Already own? | Required during any prop-on testing |

### Not on Amazon (build/fabricate)

These items depend on your airframe design and need to be fabricated:

- **Airframe** — Foam board (Dollar Tree foam board works), 3D-printed, or balsa/plywood. Design TBD
- **Tilt nacelle brackets** — 3D-print or CNC. Must mount DS3225 servo horn to motor mount plate with 0-90° rotation
- **Motor mount plates** — Aluminum or 3D-printed. 22mm bolt pattern for 2212 motors
- **Landing gear** — Simple skids from aluminum rod or 3D-printed legs

---

## Approximate cost (electronics only, excluding airframe)

| Category | Estimate |
|----------|----------|
| Pi 4B + SD card | ~$55-65 |
| BNO085 IMU | ~$20-30 |
| Batteries (2-pack) | ~$25 |
| BEC + power distribution | ~$15-20 |
| Motors (4-pack) | ~$25 |
| ESCs (4-pack) | ~$20-25 |
| Props (2 pairs) | ~$8 |
| Tilt servos DS3225 (2x) | ~$25 |
| Surface servos MG90S (5-pack) | ~$12-15 |
| PCA9685 | ~$8-10 |
| Level shifter | ~$8 |
| TX + receivers | ~$55-65 |
| Wiring/connectors/misc | ~$25-30 |
| **Total electronics** | **~$300-370** |
