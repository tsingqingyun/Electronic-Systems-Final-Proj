# Fixed-Cube Slalom Code Logic

## 1. Project Goal

This project controls a Raspberry Pi car for a fixed-position cube course:

```text
start
-> avoid red cube
-> approach green cube
-> orbit green cube once
-> leave green cube
-> avoid yellow cube
-> finish
```

The car currently has:

```text
camera
KS103 ultrasonic sensor
left/right motor control
left/right wheel encoder signals
no MPU6050 gyro
no line sensor in this code
```

## 2. Project Structure

```text
main.py
```

Program entry point. It creates `Config`, creates `CubeSlalomController`, and starts the main loop.

```text
config.py
```

The single source of truth for all tunable parameters:

- GPIO pins
- motor speed and turn strength
- HSV color thresholds
- PID gains
- ultrasonic distance thresholds
- encoder calibration
- orbit completion thresholds

Hardware pin values remain those of this repository and are not imported from
the reference `rubik-cube` repository.

```text
control/
  controller.py
```

High-level state machine and task logic. It decides what the car should do based on camera detections, ultrasonic distance, encoder progress, and current state.

```text
motion/
  motor.py
  pid.py
```

The motion layer contains the low-level motor driver and incremental PID
controller. The motor driver converts:

```python
motor.drive(v, w)
```

into left/right wheel PWM values.

```text
perception/
  vision.py
  sensors.py
```

The perception layer contains camera color recognition and sensor helpers:

- red, green, and yellow HSV detection
- KS103 ultrasonic reader
- encoder progress meter
- optional line guard
- optional MPU6050 gyro reader

The gyro path is present but disabled because this car does not have a gyro.

```text
gpio_motor_test.py
```

Standalone motor wiring test.

```text
turn_sweep_test.py
```

Standalone steering sensitivity test. It sweeps multiple turn strengths to help tune `TURN_DUTY`.

## 3. GPIO Mapping

Motor pins use BCM numbering.

```text
Right motor:
EA  -> GPIO13
I2  -> GPIO19
I1  -> GPIO26

Left motor:
EB  -> GPIO16
I4  -> GPIO20
I3  -> GPIO21
```

Encoder pins:

```text
B1A -> GPIO6   -> left/B motor speed signal
B2A -> GPIO12  -> right/A motor speed signal
```

Important:

```text
GPIO6 and GPIO12 are encoder inputs, not line sensors.
```

Therefore:

```python
LINE_LEFT = None
LINE_RIGHT = None
```

## 4. Motor Control

The main motor command is:

```python
motor.drive(v, w)
```

Meaning:

```text
v: forward command
w: steering command
```

The conversion is:

```python
left = BASE_DUTY * v - TURN_DUTY * w
right = BASE_DUTY * v + TURN_DUTY * w
```

Therefore:

```text
w = 0       -> straight
w > 0       -> turn one direction
w < 0       -> turn the opposite direction
|w| larger  -> stronger turn
```

`TURN_DUTY` controls how strongly `w` affects wheel-speed difference.

If turning is weak, tune:

```python
TURN_DUTY
```

Typical values to test:

```text
22, 30, 38, 46
```

## 5. Vision Recognition

Vision uses HSV thresholding.

Pipeline:

```text
camera frame
-> resize to 640x480
-> Gaussian blur
-> BGR to HSV
-> color mask by HSV range
-> morphology open/close
-> find contours
-> choose largest contour
-> return Blob(cx, cy, area, w, h)
```

The returned `Blob` fields are used as:

```text
cx    horizontal position of cube
area  approximate target size
```

Relevant parameters:

```python
HSV_RANGES
MIN_BLOB_AREA
DEBUG_VIEW
DEBUG_LOG
```

## 6. PID Control

The code uses incremental PID for visual steering.

Formula:

```text
delta_u = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
u[k] = u[k-1] + delta_u
```

PID is used in:

```text
APPROACH_GREEN
AVOID_RED
AVOID_YELLOW
ORBIT_GREEN
```

Approach PID:

```python
VISION_KP
VISION_KI
VISION_KD
VISION_OUTPUT_LIMIT
```

Obstacle avoidance PID:

```python
AVOID_KP
AVOID_KI
AVOID_KD
AVOID_OUTPUT_LIMIT
```

Orbit PID:

```python
ORBIT_KP
ORBIT_KI
ORBIT_KD
ORBIT_OUTPUT_LIMIT
```

## 7. State Machine

The controller states are:

```text
AVOID_RED
APPROACH_GREEN
ORBIT_GREEN
EXIT_ORBIT
AVOID_YELLOW
FINISH
```

### AVOID_RED

Purpose:

```text
avoid the red cube
```

If red is visible, the car shifts its path so that it passes on:

```python
RED_PASS_SIDE = "left"
```

If green becomes the dominant target, the state changes to:

```text
APPROACH_GREEN
```

### APPROACH_GREEN

Purpose:

```text
center the green cube and approach it
```

The visual error is:

```python
center_err = (FRAME_W / 2 - green.cx) / (FRAME_W / 2)
```

PID converts this error into steering command `w`.

The car enters orbit when:

```text
green area is large enough
or
ultrasonic distance < GREEN_ORBIT_ENTER_CM
```

When orbit begins, the code records:

```text
green starting cx
green starting area
encoder progress = 0
```

### ORBIT_GREEN

Purpose:

```text
orbit around the green cube once
```

Current direction:

```python
ORBIT_DIRECTION = "clockwise"
```

For clockwise orbit, the target green-cube position is:

```python
target_x = FRAME_W * 0.72
```

That keeps the green cube on the side of the image while the car moves around it.

Orbit steering combines:

```text
fixed tangent turn
visual PID side-position correction
ultrasonic distance correction
```

The code does not use a gyro. Orbit completion uses:

```text
visual loop closure + encoder progress
```

It considers the orbit complete when:

```text
the car has left the starting visual view
and
encoder progress >= ORBIT_MIN_PROGRESS_CM
and
green cube appears again near the starting cx
and
green area ratio is within the configured range
```

Relevant parameters:

```python
ORBIT_MIN_PROGRESS_CM
ORBIT_LOOP_CX_TOL
ORBIT_LOOP_AREA_MIN_RATIO
ORBIT_LOOP_AREA_MAX_RATIO
ORBIT_TIMEOUT_S
```

### EXIT_ORBIT

Purpose:

```text
leave the green cube after completing the orbit
```

The car drives forward until it sees yellow, then switches to:

```text
AVOID_YELLOW
```

### AVOID_YELLOW

Purpose:

```text
avoid the yellow cube and finish the course
```

Yellow pass side:

```python
YELLOW_PASS_SIDE = "right"
```

After yellow disappears, the car continues forward for:

```python
POST_YELLOW_CLEAR_S
```

Then it enters:

```text
FINISH
```

## 8. Ultrasonic Safety

Before executing state-specific logic, the controller checks:

```python
if dist_cm < STOP_CM:
    motor.drive(0.0, 0.75)
```

Meaning:

```text
if something is too close, stop forward motion and turn away
```

Current threshold:

```python
STOP_CM = 12.0
```

The KS103 sensor should appear on I2C address:

```text
0x74
```

If the sensor is not connected correctly, logs may show:

```text
Remote I/O error
```

## 9. Test Commands

Run from the Raspberry Pi:

```bash
cd /home/pi/workspace/final_proj
```

Syntax check:

```bash
python3 -m compileall -q control motion perception config.py main.py gpio_motor_test.py turn_sweep_test.py
```

Motor wiring test:

```bash
python3 gpio_motor_test.py
```

Expected sequence:

```text
left forward
left backward
right forward
right backward
both forward
```

Steering sensitivity test:

```bash
python3 turn_sweep_test.py
```

Main program:

```bash
python3 main.py
```

## 10. Runtime Logs

When `DEBUG_LOG = True`, the controller prints:

```text
state=... dist=... yaw=... enc=... seen=[...] action=...
```

Fields:

```text
state   current state-machine state
dist    ultrasonic distance in cm
yaw     gyro yaw, usually 0 because gyro is disabled
enc     encoder-estimated progress in cm
seen    detected cube colors and positions
action  current control action
```

Use these logs to tune color thresholds, steering gains, and orbit completion parameters.
