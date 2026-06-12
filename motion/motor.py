#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Motor driver for the Raspberry Pi car.

This file maps high-level wheel commands to GPIO direction pins and PWM
duty cycles. It uses EA/I1/I2 as the right motor channel and EB/I4/I3 as
the left motor channel.
"""

from __future__ import annotations

import RPi.GPIO as GPIO

from config import Config


class MotorDriver:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup([cfg.EA, cfg.I2, cfg.I1, cfg.EB, cfg.I4, cfg.I3], GPIO.OUT)
        self.pwm_l = GPIO.PWM(cfg.EB, cfg.PWM_FREQ)
        self.pwm_r = GPIO.PWM(cfg.EA, cfg.PWM_FREQ)
        self.pwm_l.start(0)
        self.pwm_r.start(0)
        self.stop()

    def _set_left_dir(self, forward: bool) -> None:
        high_forward = self.cfg.LEFT_MOTOR_FORWARD_HIGH
        GPIO.output(self.cfg.I4, GPIO.HIGH if forward == high_forward else GPIO.LOW)
        GPIO.output(self.cfg.I3, GPIO.LOW if forward == high_forward else GPIO.HIGH)

    def _set_right_dir(self, forward: bool) -> None:
        high_forward = self.cfg.RIGHT_MOTOR_FORWARD_HIGH
        GPIO.output(self.cfg.I1, GPIO.HIGH if forward == high_forward else GPIO.LOW)
        GPIO.output(self.cfg.I2, GPIO.LOW if forward == high_forward else GPIO.HIGH)

    def set_wheels(self, left: float, right: float) -> None:
        self._set_left_dir(left >= 0)
        self._set_right_dir(right >= 0)
        self.pwm_l.ChangeDutyCycle(self._clip(abs(left)))
        self.pwm_r.ChangeDutyCycle(self._clip(abs(right)))

    def drive(self, v: float, w: float) -> None:
        v = max(-1.0, min(1.0, v))
        w = max(-1.0, min(1.0, w))
        left = self.cfg.BASE_DUTY * v - self.cfg.TURN_DUTY * w
        right = self.cfg.BASE_DUTY * v + self.cfg.TURN_DUTY * w
        self.set_wheels(left, right)

    def stop(self) -> None:
        GPIO.output([self.cfg.I2, self.cfg.I1, self.cfg.I4, self.cfg.I3], GPIO.LOW)
        self.pwm_l.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(0)

    def close(self) -> None:
        self.stop()
        self.pwm_l.stop()
        self.pwm_r.stop()

    def _clip(self, duty: float) -> float:
        return max(self.cfg.MIN_DUTY, min(self.cfg.MAX_DUTY, duty))
