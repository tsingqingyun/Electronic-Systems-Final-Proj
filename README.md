# final_proj

Function:
Modular code for the fixed-position cube slalom task.

## Structure

The runtime follows the same layered organization as the reference
`rubik-cube` project:

```text
main.py
  program entry point

control/
  course state machine and task decisions

motion/
  differential drive, visual steering PID, and per-wheel speed PID

perception/
  camera, ultrasonic, encoders, line guard, and optional gyro

config.py
  the single source of truth for GPIO pins and tuning parameters
```

Hardware assignments were not copied from the reference repository. All motor,
encoder, I2C, camera, and optional sensor settings continue to come from this
repository's `Config`.

## Run

```bash
cd /home/pi/workspace/final_proj
python3 main.py
```

Motor test:

```bash
python3 gpio_motor_test.py
```

Put the car on a stand before running `gpio_motor_test.py`.

## Current Hardware Assumptions

Motor pins use BCM numbering:

```text
EA=13, I2=19, I1=26
EB=16, I4=20, I3=21
```

Mapping:

```text
EA/I1/I2 -> right motor
EB/I4/I3 -> left motor
```

Encoder pins:

```text
B1A -> GPIO6   -> left/B motor speed
B2A -> GPIO12  -> right/A motor speed
```

No MPU6050 gyro is installed:

```python
USE_GYRO = False
```

Orbit completion therefore uses visual loop closure plus encoder progress.
The same cumulative encoder counts also provide left/right speed feedback;
task-distance resets do not clear the motor PID counters.

## Control Logic

The code follows the referenced `rubik-cube` repository structure, but keeps
the verified hardware mapping of this car. Visual steering produces the turn
command, then independent left/right speed PID controllers use encoder feedback
to correct PWM duty.

```text
FIND_RED
  -> PASS_RED
  -> CLEAR_RED
  -> FIND_GREEN
  -> APPROACH_GREEN
  -> ORBIT_GREEN
  -> EXIT_GREEN
  -> FIND_YELLOW
  -> PASS_YELLOW
  -> CLEAR_YELLOW
  -> FINISH
```

Every visual transition requires consecutive-frame confirmation. Passing a
cube requires evidence that it was seen, approached, moved to the expected
image edge, and then disappeared for several frames. Encoder distance confirms
that the whole chassis has cleared the cube.

Ultrasonic distance never identifies a cube by itself. It is only used as
supporting evidence while the expected visual target is visible. State timeout
enters `RECOVERY` and stops the car instead of pretending that the state
completed successfully.
