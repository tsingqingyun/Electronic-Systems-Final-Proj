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

The motion layer contains the low-level motor driver, visual steering PID,
and independent left/right wheel-speed PID controllers. The motor driver
converts:

```python
motor.drive(v, w)
```

into left/right wheel-speed targets. Encoder feedback then corrects each
wheel's PWM duty independently.

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

```text
gpio_motor_test.py
```

Standalone motor wiring test.

```text
turn_sweep_test.py
```

Standalone steering sensitivity test. It sweeps multiple turn strengths to help tune `TURN_DUTY`.

```text
motor_pid_test.py
```

Interactive closed-loop wheel-speed test. It attaches `EncoderMeter` to
`MotorDriver` and prints target speed, measured speed, encoder pulses, and PID
PWM output.

```text
vision_test.py
sensor_test.py
```

Non-driving diagnostics for camera color detection, KS103 distance, and
left/right encoder counts.

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

These values are feed-forward PWM commands and also define the target wheel
speeds:

```text
target speed = abs(feed-forward duty) / 100 * MAX_WHEEL_SPEED_CMPS
```

Every `MOTOR_PID_PERIOD_S`, GPIO6 and GPIO12 pulse increments are converted
to measured left/right wheel speeds. Each wheel PID then computes:

```text
PWM = feed-forward PWM
    + Kp * speed_error
    + Ki * integrated_error
    + Kd * error_rate
```

The encoder counters are cumulative. Resetting task progress only moves the
distance origin, so it does not interrupt motor-speed measurement.

Therefore:

```text
w = 0       -> straight
w > 0       -> turn one direction
w < 0       -> turn the opposite direction
|w| larger  -> stronger turn
```

`TURN_DUTY` controls how strongly `w` affects wheel-speed difference.

Motor-speed PID tuning parameters are:

```python
MAX_WHEEL_SPEED_CMPS
MOTOR_PID_PERIOD_S
MOTOR_KP
MOTOR_KI
MOTOR_KD
MOTOR_INTEGRAL_LIMIT
```

Tune `MAX_WHEEL_SPEED_CMPS` and `MOTOR_KP` first, then add a small `MOTOR_KI`.
Keep `MOTOR_KD` small because encoder pulse measurements are discrete.

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
PASS_RED
PASS_YELLOW
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
FIND_RED
PASS_RED
CLEAR_RED
FIND_GREEN
APPROACH_GREEN
ORBIT_GREEN
EXIT_GREEN
FIND_YELLOW
PASS_YELLOW
CLEAR_YELLOW
RECOVERY
FINISH
```

Transitions follow four rules:

```text
the target must be visually identified
the condition must hold for consecutive frames
encoder distance must prove clearance where required
timeout enters RECOVERY instead of proving completion
```

### Red Cube

```text
FIND_RED
  red is visible for TARGET_CONFIRM_FRAMES
  -> PASS_RED

PASS_RED
  red has been stably seen
  + red has reached the near zone
  + red has moved to the expected image edge
  + red is then missing for TARGET_LOST_FRAMES
  -> CLEAR_RED

CLEAR_RED
  drive at CLEAR_V
  + encoder progress >= RED_CLEAR_CM
  -> FIND_GREEN
```

The configured red pass side is:

```python
RED_PASS_SIDE = "left"
```

For a left-side pass, the red cube is controlled toward the right edge of the
camera image.

### Green Cube

```text
FIND_GREEN
  green is visible for TARGET_CONFIRM_FRAMES
  -> APPROACH_GREEN
```

`APPROACH_GREEN` uses visual PID to center the green cube. Entry to orbit
requires all of the following for consecutive frames:

```text
green is visually present
green is within GREEN_CENTER_TOL_PX of image center
green area is large enough or ultrasonic distance is close enough
```

Ultrasonic distance is only supporting evidence here; it cannot identify the
green cube by itself.

`ORBIT_GREEN` combines:

```text
fixed tangent turn
visual PID side-position correction
ultrasonic distance correction
```

Orbit completion requires:

```text
the car has left the starting visual view
encoder progress >= ORBIT_MIN_PROGRESS_CM
green cube appears again near the starting cx
green area ratio is within the configured range
the complete condition holds for ORBIT_CONFIRM_FRAMES
```

After a confirmed orbit:

```text
EXIT_GREEN
  drive at CLEAR_V
  + encoder progress >= GREEN_EXIT_CM
  + green is missing for TARGET_LOST_FRAMES
  -> FIND_YELLOW
```

### Yellow Cube

```text
FIND_YELLOW
  yellow is visible for TARGET_CONFIRM_FRAMES
  -> PASS_YELLOW

PASS_YELLOW
  yellow has been stably seen
  + yellow has reached the near zone
  + yellow has moved to the expected image edge
  + yellow is then missing for TARGET_LOST_FRAMES
  -> CLEAR_YELLOW

CLEAR_YELLOW
  drive at CLEAR_V
  + encoder progress >= YELLOW_CLEAR_CM
  -> FINISH
```

The configured yellow pass side is:

```python
YELLOW_PASS_SIDE = "right"
```

For a right-side pass, the yellow cube is controlled toward the left edge of
the camera image.

### Recovery

If a required target is not found, a maneuver times out, or an encoder needed
to prove clearance is unavailable, the controller enters:

```text
RECOVERY
```

`RECOVERY` stops the motors, records the failure reason, exits the main loop,
and performs normal hardware cleanup. Time is used only to detect a stuck
state; it is not treated as successful task completion.

## 8. Ultrasonic Safety

Before executing state-specific logic, the controller checks:

```python
if dist_cm < STOP_CM:
    motor.drive(0.0, emergency_turn_direction)
```

Meaning:

```text
if something is too close, stop forward motion and turn toward the configured
pass direction for the current course stage
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
cd /home/pi/workspace/0612_final
```

### Stage 1: Static Check

Run before every hardware test:

```bash
python3 -m compileall -q .
```

This verifies Python syntax and imports. It does not prove that GPIO, I2C,
camera, or encoders work.

### Stage 2: Motor Wiring

Put the car on a stand:

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

This script applies PWM directly. It intentionally bypasses encoders and motor
PID. Do not tune PID until all five actions match the physical wheels.

If a wheel direction is wrong, fix:

```python
LEFT_MOTOR_FORWARD_HIGH
RIGHT_MOTOR_FORWARD_HIGH
```

Do not compensate for reversed wiring by changing PID gains.

### Stage 3: Encoder and Motor PID

Keep the car on a stand for the first run:

```bash
python3 motor_pid_test.py
```

Each case waits for Enter and prints:

```text
target=(left,right) cm/s
measured=(left,right) cm/s
pulses=(left,right)
pwm=(left,right)
```

Check the output in this order:

```text
both pulse counts increase
left wheel changes the left count
right wheel changes the right count
measured speeds are physically plausible
PID PWM settles instead of staying at a limit
```

Common symptoms:

```text
wheel moves but pulses stay at zero
  -> check GPIO6/GPIO12 wiring and EncoderMeter.available

left wheel changes the right count
  -> swap ENCODER_LEFT/ENCODER_RIGHT assignments or encoder wiring

measured speed is consistently scaled wrong
  -> calibrate ENCODER_PULSES_PER_REV and WHEEL_CIRCUMFERENCE_CM

measured speed is correct but target scale is wrong
  -> calibrate MAX_WHEEL_SPEED_CMPS

PWM immediately oscillates
  -> reduce MOTOR_KP, set MOTOR_KI and MOTOR_KD near zero

steady speed stays below target
  -> add MOTOR_KI gradually after MOTOR_KP is stable

PWM jumps because pulse readings are noisy
  -> keep MOTOR_KD small or zero; consider increasing MOTOR_PID_PERIOD_S
```

Recommended tuning sequence:

```text
1. Set MOTOR_KI = 0 and MOTOR_KD = 0.
2. Measure real wheel speed and calibrate MAX_WHEEL_SPEED_CMPS.
3. Increase MOTOR_KP until correction is responsive but not oscillatory.
4. Add a small MOTOR_KI to remove steady error.
5. Add MOTOR_KD only if there is a clear benefit.
```

### Stage 4: Steering

`turn_sweep_test.py` does not attach the encoder, so it is an open-loop
steering baseline:

```bash
python3 turn_sweep_test.py
```

Use it to verify turn direction and choose a reasonable `TURN_DUTY`. Then
repeat the left/right cases in `motor_pid_test.py` to inspect closed-loop wheel
behavior.

### Stage 5: Vision

Run without initializing the motors:

```bash
python3 vision_test.py
```

Verify red, green, and yellow detections separately. Check that the bounding
box is stable and that small reflections do not exceed `MIN_BLOB_AREA`.

### Stage 6: Sensors

Run without initializing the motors:

```bash
python3 sensor_test.py
```

Move an object in front of the KS103 and rotate each wheel by hand. Confirm:

```text
KS103 distance changes when an object moves
both encoder counts increase independently
progress_cm increases during forward travel
```

The controller log should report plausible `dist` and `enc` values.

### Stage 7: State Machine

Raise the drive wheels or use a large clear test area:

```bash
python3 main.py
```

Test one transition at a time:

```text
FIND_RED -> PASS_RED
PASS_RED -> CLEAR_RED
CLEAR_RED -> FIND_GREEN
APPROACH_GREEN -> ORBIT_GREEN
ORBIT_GREEN -> EXIT_GREEN
PASS_YELLOW -> CLEAR_YELLOW -> FINISH
```

Stop and fix the first incorrect transition instead of continuing through the
whole course.

## 10. Runtime Logs

When `DEBUG_LOG = True`, the controller prints:

```text
state=... dist=... enc=... seen=[...] action=...
```

Fields:

```text
state   current state-machine state
dist    ultrasonic distance in cm
enc     encoder-estimated progress in cm
seen    detected cube colors and positions
action  current control action
```

Use these logs to tune color thresholds, steering gains, and state completion
parameters. Motor PID details are printed by `motor_pid_test.py`, not by the
normal controller log.
