# Joy-IT Robot arm Raspberry Pi

Python library for the **Joy-IT Robot02** robot arm with six servos and the **RB-Moto3 (MotoPi)** servo controller on a Raspberry Pi. Control individual joints, move the arm between poses over a specified duration, or operate it using the **SBC-WLGamepad** wireless controller.

## Features

- Six logical servo indices with configurable MotoPi channel assignments
- Direct angle commands for individual servos or all six servos
- Individual pulse-width calibration and configurable home positions
- Timed movements using linear angle interpolation
- Optional SBC-WLGamepad control
- Access to the last commanded servo angles

## Requirements

- Joy-IT Robot02: robot arm
- Joy-IT RB-Moto3 (MotoPi)
- A suitable servo power supply
- Raspberry Pi with Python **3.9 or later** and the **I2C interface enabled**
- Optional Joy-IT SBC-WLGamepad with its receiver and Joy-IT RB-Port-Doubler

The library depends on the following Python packages, which are installed automatically:

| Package | Purpose |
| --- | --- |
| [`JoyIT_MotoPi`](https://github.com/joy-it/JoyIT_MotoPi) | Driver for the PCA9685 PWM controller of the RB-Moto3 |
| [`JoyIT_WLGamepad`](https://github.com/joy-it/JoyIT_WLGamepad) | Driver for the SBC-WLGamepad receiver |
| `smbus2` | I2C communication, used by `JoyIT_MotoPi` |
| `gpiozero`, `lgpio` | GPIO access, used for the enable pin of the RB-Moto3 |

`JoyIT_WLGamepad` uses the `RPi.GPIO` module. On Raspberry Pi OS it is available system-wide; create virtual environments with `--system-site-packages` so it can be found. If it is missing (for example in a virtual environment without system packages), install the compatible replacement with `pip install rpi-lgpio`. The original `RPi.GPIO` package does not work on the Raspberry Pi 5.

## Installation

Enable I2C using `sudo raspi-config` (Interface Options → I2C), then reboot if prompted.

Clone the repository and install the library into a virtual environment:

```bash
python -m venv --system-site-packages env
source env/bin/activate
pip install JoyIT-Robotarm-RaspberryPi
```

The Linux user must have access to the I2C device and the GPIO pins (groups `i2c` and `gpio`).

## Connections

Plug the RB-Moto3 onto the Raspberry Pi and power the servos according to its hardware documentation. The examples use I2C address **0x41**, enable pin **GPIO 7** and MotoPi channels **0–5**, ordered from the bottom of the arm to the top. The channels can be changed easily.

`begin()` sets the PWM frequency of the MotoPi to a nominal **50 Hz**. The clock settings of the PCA9685 (25 MHz external oscillator) are handled by `JoyIT_MotoPi`.

> [!NOTE]
> GPIO 7 is also the SPI chip-select pin CE1. If SPI is enabled, the enable pin may be reported as busy. Disable SPI or declare a different chip-select pin.

For the SBC-WLGamepad receiver, the Raspberry Pi connections used by the default pins of `JoyIT_WLGamepad` are:
> [!NOTE]
> If you want to use the gamepad, you need the [**RB-Port-Doubler**](https://joy-it.net/en/products/RB-Port-Doubler). Otherwise there are no options to connect the SBC-WLGamepad.
| Receiver connection | Raspberry Pi (BCM) |
| --- | --- |
| Pin 1 — Data | GPIO 22 |
| Pin 2 — Command | GPIO 27 |
| Pin 3 | --- |
| Pin 4 — Ground | GND |
| Pin 5 — Power | 3.3 V |
| Pin 6 — Attention | GPIO 17 |
| Pin 7 — Clock | GPIO 18 |
| Pin 8 | --- |
| Pin 9 | --- |

To use other pins, create the gamepad yourself and pass it to the library:

```python
from wlgamepad import WLGamepad

robotarm.use_external_controller(True, WLGamepad(ATT=17, CLK=18, CMD=27, DATA=22))
```

See the [SBC-WLGamepad manual](https://joy-it.net/files/files/Produkte/SBC-WLGamepad/SBC-WLGamepad_Manual_2024-03-27.pdf) for receiver pin identification and setup.

## Functions of the repository

### Getting started

```python
from MotoPi import MotoPi
from Robotarm import Robotarm

# MotoPi channels connected to the six servos, ordered from bottom to top
servo_channels = [0, 1, 2, 3, 4, 5]

with MotoPi(address=0x41, enable_pin=7) as motopi:
    robotarm = Robotarm(motopi, servo_channels)
    robotarm.begin()

    # command all servos to their home positions
    robotarm.reset_all_positions()

    robotarm.set_angle_of_servo(0, 120)
```

Use `MotoPi` as a context manager: all PWM outputs are switched off and the I2C bus is released when the `with` block ends, including on exceptions. Keep your program inside the `with` block for as long as the servos should hold their positions.

### Pulse width of servo motors

By default, **0° maps to 500 µs** and **180° maps to 2500 µs**. With this calibration, 90° corresponds to 1500 µs.

```python
# Configure a single servo.
robotarm.set_servo_calibration(0, 500, 2500)

# Apply the same calibration to all six servos.
robotarm.set_pulse_width_of_all_servos(500, 2500)
```

> [!NOTE]
> These methods configure the pulse-width mapping; they do not move the servos. The new range is used from the next angle command on. They are not PWM frequency setters.

### Home positions

The default home angles, in servo-index order, are:

```python
(80, 100, 55, 85, 25, 140)
```

Change a home position at runtime and command it as follows:

```python
robotarm.set_home_position(2, 30)
# move servo 2 to newly set home position
robotarm.reset_position(2)

# Command all servo motors to home positions immediately
robotarm.reset_all_positions()
```

The home positions and calibration changes are stored in memory and are lost when the program ends. Home positions are commanded positions, not positions detected by a homing procedure.

> [!NOTE]
> Use pulse widths and angles appropriate for your servos and the assembled arm. The software clamps angles to 0–180° but does not detect mechanical end stops, collisions, or excessive load.

### Timed movements

Use `move_to()` for one servo or `move_all_to()` for an entire pose. Both start a movement and return immediately. Call `update()` frequently to generate the intermediate angle commands.

> [!NOTE]
> After initialization, first establish a known commanded position, for example with `reset_all_positions()`. Timed movements return `False` if the required starting positions are unknown.

```python
robotarm.reset_all_positions()
robotarm.move_to(0, 110, 2000)

while robotarm.is_moving():
    robotarm.update()
    time.sleep(0.01)
```

A complete pose contains six absolute target angles:

```python
target_angles = [100, 85, 25, 115, 35, 145]

robotarm.move_all_to(target_angles, 3000)
```

Use `is_moving()` to determine whether a timed movement remains active. A duration of `0` applies the target immediately. Direct angle commands cancel timed movement for the affected servo. Avoid long `time.sleep()` calls while a timed movement is in progress, because interpolation only advances when `update()` runs. A short sleep of a few milliseconds per loop keeps the movement smooth and limits the CPU load.

### Gamepad control

After `begin()` and the initial position commands, enable the controller:

```python
if not robotarm.use_external_controller(True):
    print("Gamepad initialization failed.")
```

Keep the following in your main loop:

```python
while True:
    robotarm.update()
    time.sleep(0.005)
```

| Control | Servo index | Direction |
| --- | --- | --- |
| Left stick horizontal | 0 | Right increases the angle |
| Left stick vertical | 1 | Up increases the angle |
| Right stick vertical | 2 | Up increases the angle |
| Right stick horizontal | 3 | Right increases the angle |
| L1 / R1 | 4 | L1 increases; R1 decreases |
| L2 / R2 | 5 | L2 increases; R2 decreases |

The receiver is polled at intervals of at least 20 ms. Stick inputs use a dead zone around their center position. The sticks move the servos with up to 45°/s, the shoulder buttons with 60°/s. Controller commands change the stored angle incrementally; servos without a known position ignore controller input, so command the home positions first. A controller command overrides a timed movement on the affected servo.

The sticks only work in analog mode (controller IDs `0x73` and `0x79`). In digital mode (`0x41`) only the shoulder buttons are evaluated. If the controller is not recognized, consult the receiver manual's Mode-button instructions.

Disable controller input with:

```python
robotarm.use_external_controller(False)
```

If a controller read fails, `controller_ready` becomes `False`. The library keeps polling, and control resumes automatically as soon as the controller answers again.

`use_external_controller(True)` may also be called before `begin()`. The controller is then set up during `begin()`.

### Reading positions

```python
angle = robotarm.get_position_of_servo(0)
if angle is not None:
    print(angle)

angles = robotarm.get_position_of_all_servo()
```

Position values represent the **last commanded angles**. The library does not measure physical servo positions. Unknown positions are returned as `None`.

> [!NOTE]
> Direct calls to `motopi.write_microseconds()` bypass the library's position tracking. Establish the commanded positions through the Robotarm API before starting relative or timed movements.

### Shutting down

```python
robotarm.release()
```

`release()` disables the controller, stops all timed movements and commands the home positions. Leaving the `with MotoPi(...)` block afterwards switches all PWM outputs off.

## API reference

Servo arguments are logical indices **0–5**, independent of the MotoPi channel numbers. The `MotoPi` object must remain open for the lifetime of the Robotarm object. Call `begin()` before commanding any movement; otherwise a `RuntimeError` is raised. Invalid servo indices, pulse width ranges, durations or channel lists raise a `ValueError`.

| Method | Return | Description |
| --- | --- | --- |
| `Robotarm(motopi, servo=range(6))` | — | Construct an arm using an open `MotoPi` object and six distinct channel numbers (0–15). Does not access the hardware. |
| `begin()` | `bool` | Set the PWM frequency to 50 Hz and set up a previously enabled controller. Does not command home positions. |
| `use_external_controller(external=True, controller=None)` | `bool` | Enable or disable gamepad input. After `begin()`, enabling returns whether the controller answered. Without `controller`, a `WLGamepad` with default pins is created. |
| `controller_ready` | `bool` | Property: `True` if the controller delivered a valid reading at the last poll. |
| `set_servo_calibration(servo_index, min_pulse_us, max_pulse_us)` | `None` | Set pulse widths for 0° and 180°. Requires `0 < min < max < 20000`. |
| `set_pulse_width_of_all_servos(min_pulse_us, max_pulse_us)` | `None` | Apply the same calibration to all six servos. |
| `set_angle_of_servo(servo_index, angle)` | `None` | Command one angle immediately. |
| `set_angle_of_all_servos(angle)` | `None` | Command the same angle on all six servos. |
| `get_position_of_servo(servo_index)` | `int`, `float` or `None` | Return the last commanded angle, or `None` if unknown. |
| `get_position_of_all_servo()` | `list` | Return a list of the six last commanded angles (`None` for unknown positions). |
| `set_home_position(servo_index, angle)` | `None` | Configure one home angle without moving the servo. |
| `reset_position(servo_index)` | `None` | Command one configured home angle immediately. |
| `reset_all_positions()` | `None` | Command all configured home angles immediately. |
| `move_to(servo_index, angle, duration_ms)` | `bool` | Start a timed movement from a known commanded position. |
| `move_all_to(angles, duration_ms)` | `bool` | Start timed movements for all six servos. Requires six angles and six known starting positions. |
| `update()` | `None` | Advance timed movements and poll the enabled controller. |
| `is_moving()` | `bool` | Report whether any timed movement is active. Does not report physical motion or gamepad activity. |
| `stop()` | `None` | Stop all timed movements; the servos keep their current angles. |
| `release()` | `None` | Disable the controller, stop all movements and command the home positions. |

`Robotarm.SERVO_COUNT` is `6`. Angles are given in degrees as `int` or `float` and are clamped to 0–180°. Durations are given in milliseconds.

## Included examples

| Script | Purpose |
| --- | --- |
| `examples/robotarm_simple_example.py` | Alternate servo 0 between 20° and 160° using direct angle commands. |
| `examples/robotarm_movement_over_duration.py` | Run three poses relative to the home angles with timed movements and pauses. |
| `examples/robotarm_with_controller.py` | Control all six servos with the gamepad and print commanded positions every second. |
| `examples/calibrate_motors_mounting.py` | Output fixed pulse widths to position servos during assembly. |

Run an example inside the activated virtual environment, for example:

```bash
python examples/robotarm_simple_example.py
```

Stop the examples with **Ctrl+C**. The movement and controller examples then command the home positions before the outputs are switched off.

In the supplied assembly script, indices 0–4 receive 1500 µs, then index 5 is set to 1600 µs. Check that assignment against your intended gripper channel before using it for assembly.

## Links

- [GitHub repository](https://github.com/joy-it/JoyIT_Robotarm_RaspberryPi)
- [Arduino version of this library](https://github.com/joy-it/JoyIT_Robotarm_Arduino)
- [JoyIT_MotoPi](https://github.com/joy-it/JoyIT_MotoPi)
- [JoyIT_WLGamepad](https://github.com/joy-it/JoyIT_WLGamepad)
- [Joy-IT](https://joy-it.net/)

## AI assistance disclosure

AI tools were used to assist with parts of the code, code review, example sketches, and documentation AI-assisted tools were used in parts of this library's development, including code implementation, code review, example sketches, and documentation. All AI-assisted content has been reviewed, verified, and tested on target hardware by our engineering team prior to release. This software is provided under the terms and warranty disclaimers of the repository's license.