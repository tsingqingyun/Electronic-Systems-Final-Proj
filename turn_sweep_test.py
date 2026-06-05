#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Standalone steering sensitivity test.

This file sweeps multiple turn commands and TURN_DUTY values so you can
observe whether the car turns clearly enough. It does not use vision,
ultrasonic, or the task controller.
"""

import time
import RPi.GPIO as GPIO

from config import Config
from motor import MotorDriver


TEST_SECONDS = 1.2
PAUSE_SECONDS = 0.8


def run_case(turn_duty, v, w):
    cfg = Config()
    cfg.TURN_DUTY = turn_duty
    motor = MotorDriver(cfg)
    try:
        print(f"TURN_DUTY={turn_duty}, v={v}, w={w}")
        motor.drive(v, w)
        time.sleep(TEST_SECONDS)
        motor.stop()
        time.sleep(PAUSE_SECONDS)
    finally:
        motor.close()
        GPIO.cleanup()


def main():
    # In-place turns.
    for turn_duty in [22, 30, 38, 46]:
        run_case(turn_duty, 0.0, 0.75)
        run_case(turn_duty, 0.0, -0.75)

    # Moving turns, closer to what the main task uses.
    for turn_duty in [22, 30, 38, 46]:
        run_case(turn_duty, 0.35, 0.60)
        run_case(turn_duty, 0.35, -0.60)


if __name__ == "__main__":
    main()
