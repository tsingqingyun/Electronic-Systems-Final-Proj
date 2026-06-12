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
  motor GPIO/PWM actuation and steering PID

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

## Control Logic

The code follows the referenced `rubik-cube` repository structure, but keeps
the verified hardware mapping of this car. Visual steering uses incremental
PID:

```text
APPROACH_GREEN -> PID keeps green cube near image center
AVOID_RED/YELLOW -> PID shifts obstacle to the selected pass side
ORBIT_GREEN -> PID keeps green cube at the orbit-side image position
```
