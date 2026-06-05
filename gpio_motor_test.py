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


EA, I2, I1 = 13, 19, 26
EB, I4, I3 = 16, 20, 21
PWM_FREQ = 50
LEFT_MOTOR_FORWARD_HIGH = True
RIGHT_MOTOR_FORWARD_HIGH = True
TEST_DUTY = 35
TEST_SECONDS = 1.5


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
    left_pwm = GPIO.PWM(EB, PWM_FREQ)
    right_pwm = GPIO.PWM(EA, PWM_FREQ)
    left_pwm.start(0)
    right_pwm.start(0)
    return left_pwm, right_pwm


def set_left_dir(forward):
    GPIO.output(I4, GPIO.HIGH if forward == LEFT_MOTOR_FORWARD_HIGH else GPIO.LOW)
    GPIO.output(I3, GPIO.LOW if forward == LEFT_MOTOR_FORWARD_HIGH else GPIO.HIGH)


def set_right_dir(forward):
    GPIO.output(I1, GPIO.HIGH if forward == RIGHT_MOTOR_FORWARD_HIGH else GPIO.LOW)
    GPIO.output(I2, GPIO.LOW if forward == RIGHT_MOTOR_FORWARD_HIGH else GPIO.HIGH)


def stop(left_pwm, right_pwm):
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)
    GPIO.output([I2, I1, I4, I3], GPIO.LOW)
    time.sleep(0.8)


def run_case(name, left_pwm, right_pwm, left=None, right=None):
    print(name)
    if left is not None:
        set_left_dir(left)
        left_pwm.ChangeDutyCycle(TEST_DUTY)
    if right is not None:
        set_right_dir(right)
        right_pwm.ChangeDutyCycle(TEST_DUTY)
    time.sleep(TEST_SECONDS)
    stop(left_pwm, right_pwm)


def main():
    left_pwm, right_pwm = setup()
    try:
        stop(left_pwm, right_pwm)
        run_case("1. left wheel forward", left_pwm, right_pwm, left=True)
        run_case("2. left wheel backward", left_pwm, right_pwm, left=False)
        run_case("3. right wheel forward", left_pwm, right_pwm, right=True)
        run_case("4. right wheel backward", left_pwm, right_pwm, right=False)
        run_case("5. both wheels forward", left_pwm, right_pwm, left=True, right=True)
        print("Done. Check whether each printed action matched the real wheel motion.")
    finally:
        stop(left_pwm, right_pwm)
        left_pwm.stop()
        right_pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
