# final_proj

Function:
Modular code for the fixed-position cube slalom task.

## Files

- `main.py`: entry point.
- `config.py`: all tunable parameters and GPIO assignments.
- `controller.py`: finite-state machine and driving logic.
- `motor.py`: motor GPIO/PWM driver.
- `vision.py`: camera color detection.
- `sensors.py`: ultrasonic, encoder, line guard, optional gyro helpers.
- `gpio_motor_test.py`: standalone motor wiring test.
- `requirements.txt`: Python dependency notes.

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
