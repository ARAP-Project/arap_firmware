# ARAP Robot Controller

Firmware for the ARAP differential-drive robot platform. Runs on an Arduino Mega 2560 and provides closed-loop wheel velocity control, encoder feedback, GPS positioning, ultrasonic obstacle detection, and LED status indication. Communicates with the ROS navigation stack over serial UART at 57600 baud.

---

## Hardware

| Component | Model | Interface |
|---|---|---|
| Microcontroller | Arduino Mega 2560 | USB Serial |
| Motor Drivers (x2) | BTS7960 H-Bridge | PWM + Digital |
| Drive Motors (x2) | 24V wiper motors with quadrature encoders | Via BTS7960 |
| GPS Module | u-blox NEO-M9N | I2C |
| Ultrasonic Sensors (x2) | A0221AU | UART 9600 baud |
| LED Strip | WS2812B (16 LEDs) | Single-wire data |

---

## Pin Assignments

### Motors (BTS7960)

| Function | Left Motor | Right Motor |
|---|---|---|
| RPWM | 11 | 7 |
| LPWM | 10 | 6 |
| R_EN | 24 | 22 |
| L_EN | 25 | 23 |

Power the BTS7960 motor input from the drive battery. Do not power the motor side from the Arduino.

> **Note on the pin ordering.** The driver wired to pins 7/6/22/23 physically
> drives the **right** motor, and 11/10/24/25 the **left** — the opposite of
> what the original wiring suggested. Measured 2026-09-07: commanding a
> counter-clockwise turn drove the firmware's "left" channel while the robot's
> physical right wheel responded. The constants above reflect the real wiring,
> so `left` in code means the physical left wheel.

### Encoders (Quadrature A+B)

| Channel | Left Motor | Right Motor |
|---|---|---|
| A | Pin 18 (INT5) | Pin 2 (INT0) |
| B | Pin 19 (INT4) | Pin 3 (INT1) |
| VCC | 5V | 5V |
| GND | GND | GND |

> **Note on encoder direction.** The encoders are crossed relative to the
> motors *and* count backwards. Verified by turning the physical left wheel
> forward by hand with the motors idle: it drove `right_wheel_joint` negative.
> `INVERT_LEFT_ENCODER` / `INVERT_RIGHT_ENCODER` correct the sign.
>
> These two faults cancel each other during rotation — the wheels turn
> opposite ways, so a swap and an inversion negate — and only show up in
> straight-line travel. **Verify encoder wiring by hand-turning a single
> wheel, never by rotating the robot.**

### GPS (NEO-M9N)

| GPS Pin | Mega Pin |
|---|---|
| SDA | 20 |
| SCL | 21 |
| VCC | 3.3V or 5V |
| GND | GND |

### Ultrasonic Sensors (A0221AU)

Wire colors:

| Color | Function |
|---|---|
| Red | VCC — connect to 5V |
| Black | GND — connect to GND |
| Yellow | TX (data out) — connect to Arduino RX pin |
| White | RX — leave unconnected |

Sensor connections:

| Sensor | Yellow wire to |
|---|---|
| Left | Pin 17 (RX2) |
| Right | Pin 15 (RX3) |

### LED Strip (WS2812B)

| Wire | Mega Pin |
|---|---|
| Data | 12 |
| VCC | 5V |
| GND | GND |

Add a 330Ω resistor on the data line and a 470µF capacitor across VCC/GND at the strip.

---

## Velocity Control

The firmware runs a **closed-loop velocity PID per wheel**. The `m` command
carries a velocity setpoint in **encoder counts per control loop**, not a PWM
value. The PID measures how far each encoder actually moved and adjusts PWM to
hit the setpoint, so commanded speed is held on ramps and under load instead of
sagging.

The control loop runs at 40 Hz (`MOTOR_UPDATE_MS = 25`), which must match the
`loop_rate` parameter on the ROS side, since `arap_hardware_interfaces` computes:

```
counts_per_loop = rad_per_sec / rads_per_count / loop_rate
```

Integer maths throughout, following the `ros_arduino_bridge` convention:

```
output += (KP*err + KD*(err - prev_err) + KI*integral) / KO
```

`KO` is an output divisor, which is how integer gains express fractional ones.
Gains can be set at runtime with `y` (see below); `KO` must be greater than
zero and the firmware rejects anything else.

Measured tracking after tuning: **98–100% of setpoint** at 15–25 counts/loop.
Below about 8 counts/loop the `MOTOR_DEADBAND` floor creates a dead zone the
PID cannot correct within, so very slow speeds are less precise.

---

## Dependencies

Install both libraries before compiling.

| Library | Install via Library Manager |
|---|---|
| FastLED | Search "FastLED" by Daniel Garcia |
| SparkFun u-blox GNSS Arduino Library | Search "SparkFun u-blox GNSS" |

### Arduino IDE

Sketch → Include Library → Manage Libraries → search and install each library listed above.

### arduino-cli

```bash
arduino-cli core install arduino:avr
arduino-cli lib install "FastLED"
arduino-cli lib install "SparkFun u-blox GNSS Arduino Library"
```

### PlatformIO

```ini
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
monitor_speed = 57600
lib_deps =
    fastled/FastLED@^3.6.0
    sparkfun/SparkFun u-blox GNSS Arduino Library@^2.2.0
```

---

## Project Structure

The sketch folder and the main `.ino` must share a name, which Arduino
requires:

```
arap_firmware/
├── arap_firmware.ino   Main sketch, PID and command handling
├── motorParams.h       Pin config and tuning parameters
├── commands.h          Command character definitions
├── led.h               LED function declarations
└── led.cpp             LED animation implementations
```

Include names are case-sensitive on Linux even though they are not on Windows
or macOS.

---

## Build and Upload

Stop any ROS stack first — it holds `/dev/arduino` and the upload will fail
while the port is busy.

### arduino-cli

```bash
cd arap_firmware
arduino-cli compile --fqbn arduino:avr:mega .
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega .
```

### Arduino IDE

1. Connect the Arduino Mega via USB.
2. Open `arap_firmware.ino`.
3. Board: **Arduino Mega or Mega 2560**
4. Select the correct port.
5. Click Upload.
6. Open Serial Monitor at **57600 baud**.
7. Confirm startup message: `=== ARAP Ready ===`

---

## Serial Commands

All commands are a single character followed by optional parameters,
terminated by carriage return.

### Motor

| Command | Description |
|---|---|
| `m L R` | Set wheel velocity in **counts per loop**, ±120 |
| `y P:D:I:O` | Set velocity PID gains. `O` must be > 0 |
| `k` | Emergency stop |
| `b` | Active brake |

### Sensors (ROS Integration)

| Command | Response format |
|---|---|
| `e` | `e <left_ticks> <right_ticks>` |
| `r` | Resets encoder counts to zero |
| `g` | `g <lat> <lon> <alt> <fix> <sats> <hAcc>` |
| `u` | `u <left_mm> <right_mm>` |

### LED

| Command | Description |
|---|---|
| `l N` | Set LED mode (0–12) |
| `c R G B` | Set custom color |
| `d N` | Set brightness (0–255) |

### System

| Command | Description |
|---|---|
| `s` | Full status dump |
| `p` | Ping (returns PONG) |
| `h` | Help |

### Examples

At the current wheel radius, full speed is roughly 62 counts/loop.

```
m 8 8           Crawl forward
m 25 25         Steady forward
m -25 -25       Reverse
m -25 25        Spin left (counter-clockwise)
m 25 -25        Spin right
y 20:12:0:50    Set PID gains
k               Stop
```

---

## LED Behavior

| State | Effect |
|---|---|
| Idle | Blue breathing pulse |
| Moving | Rainbow cycle |
| Braking | Color pulse |
| Error | Solid white |

---

## Configuration

Key parameters in `motorParams.h`:

| Parameter | Default | Notes |
|---|---|---|
| `INVERT_LEFT_MOTOR` | true | Flip if left motor runs backwards |
| `INVERT_RIGHT_MOTOR` | true | Flip if right motor runs backwards |
| `INVERT_LEFT_ENCODER` | true | Flip if forward rotation counts negative |
| `INVERT_RIGHT_ENCODER` | true | As above, for the right wheel |
| `PID_KP_DEFAULT` | 20 | Proportional gain |
| `PID_KD_DEFAULT` | 12 | Derivative gain. Raise cautiously — it amplifies encoder quantisation noise |
| `PID_KI_DEFAULT` | 0 | Integral gain. Enable if speed droops under sustained load |
| `PID_KO_DEFAULT` | 50 | Output divisor. Larger values scale all gains down |
| `MAX_COUNTS_PER_LOOP` | 120 | Setpoint clamp |
| `MOTOR_DEADBAND` | 35 | Minimum PWM to start rotation. Creates a dead zone the PID cannot correct within |
| `MOTOR_UPDATE_MS` | 25 | Control loop period. **Must match `loop_rate` on the ROS side** |
| `NUM_LEDS` | 16 | Adjust to match your LED strip |
| `SERIAL_BAUD` | 57600 | Must match ROS serial node and Serial Monitor |

`ACCEL_RATE` and `DECEL_RATE` are unused. The PID's incremental form limits
how fast the output can change, so the old open-loop ramp is redundant.

---

## Troubleshooting

**Motors do not spin** — Check the drive supply to the BTS7960. Verify R_EN and L_EN wiring. Try increasing `MOTOR_DEADBAND`.

**Wrong direction** — Toggle `INVERT_LEFT_MOTOR` or `INVERT_RIGHT_MOTOR` in `motorParams.h`, or swap M+/M- at the driver.

**Robot turns the wrong way but drives straight correctly** — The left/right
motor channels are swapped. A swap preserves forward travel exactly and
inverts only rotation, which is what makes it easy to miss.

**Odometry reports backwards travel while the robot moves forward** — Encoder
polarity. Hand-turn one wheel forward with the motors idle and check which
joint moves and in which direction.

**Robot moves far slower than commanded** — The `m` command is a velocity
setpoint, not PWM. If something upstream sends PWM values it will be
interpreted as an unreachable velocity, or as a crawl. Check that
`MOTOR_UPDATE_MS` matches the ROS `loop_rate`.

**Speed sags on ramps or under load** — Enable the integral term
(`PID_KI_DEFAULT`), which removes steady-state error that P and D alone leave.

**GPS not found** — Check SDA/SCL wiring. The system continues without GPS if the module is absent.

**Ultrasonic returns -1** — Verify yellow wire on correct RX pin (17 left, 15 right). Ensure 5V power. Min range is 30mm.

**LEDs not working** — Confirm data pin 12, 5V power, GND, and that FastLED is installed.

**Garbled serial output** — Set baud to 57600.

**Upload fails with the port busy** — Stop the ROS stack; `ros2_control` holds `/dev/arduino` continuously.
