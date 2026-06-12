#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Standalone motor GPIO diagnosis.

Put the car on a stand before running this file. It tests left/right motor
forward and backward separately, then both wheels forward. Use it to verify
PWM pins, direction pins, and wheel polarity before running main.py.
"""

import time

import RPi.GPIO as GPIO

from config import Config
from motion import MotorDriver


TEST_DUTY = 35
TEST_SECONDS = 1.5
PAUSE_SECONDS = 0.8


def run_case(name, motor, left=0.0, right=0.0):
    print(name)
    motor.set_wheels(left, right)
    time.sleep(TEST_SECONDS)
    motor.stop()
    time.sleep(PAUSE_SECONDS)


def main():
    motor = MotorDriver(Config())
    try:
        run_case("1. left wheel forward", motor, left=TEST_DUTY)
        run_case("2. left wheel backward", motor, left=-TEST_DUTY)
        run_case("3. right wheel forward", motor, right=TEST_DUTY)
        run_case("4. right wheel backward", motor, right=-TEST_DUTY)
        run_case(
            "5. both wheels forward",
            motor,
            left=TEST_DUTY,
            right=TEST_DUTY,
        )
        print("Done. Check whether each printed action matched the real wheel motion.")
    finally:
        motor.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
