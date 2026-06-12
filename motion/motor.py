#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Motor driver for the Raspberry Pi car.

The differential-drive behavior follows the rubik-cube motion layer:

* straight motion drives both wheels in the same direction,
* an in-place turn drives the wheels in opposite directions,
* a moving turn keeps both wheels travelling in the commanded direction
  while slowing the inner wheel and speeding up the outer wheel.

GPIO pins, PWM frequency, motor polarity, and duty limits still come only
from this project's Config.
"""

from __future__ import annotations

import time
from typing import Optional

import RPi.GPIO as GPIO

from config import Config
from .pid import WheelSpeedPID


LEFT = 1
RIGHT = 2


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
        self.encoder = None
        self.left_pid = self._new_speed_pid()
        self.right_pid = self._new_speed_pid()
        self.last_encoder_counts: Optional[tuple[int, int]] = None
        self.last_pid_time: Optional[float] = None
        self.last_left_duty = 0.0
        self.last_right_duty = 0.0
        self.last_left_sign = 0
        self.last_right_sign = 0
        self.stop()

    def _new_speed_pid(self) -> WheelSpeedPID:
        return WheelSpeedPID(
            kp=self.cfg.MOTOR_KP,
            ki=self.cfg.MOTOR_KI,
            kd=self.cfg.MOTOR_KD,
            output_min=self.cfg.MIN_DUTY,
            output_max=self.cfg.MAX_DUTY,
            integral_limit=self.cfg.MOTOR_INTEGRAL_LIMIT,
        )

    def attach_encoder(self, encoder) -> None:
        """Use an EncoderMeter for per-wheel speed feedback."""
        self.encoder = encoder
        self._reset_speed_control()

    def _set_left_dir(self, forward: bool) -> None:
        high_forward = self.cfg.LEFT_MOTOR_FORWARD_HIGH
        GPIO.output(self.cfg.I4, GPIO.HIGH if forward == high_forward else GPIO.LOW)
        GPIO.output(self.cfg.I3, GPIO.LOW if forward == high_forward else GPIO.HIGH)

    def _set_right_dir(self, forward: bool) -> None:
        high_forward = self.cfg.RIGHT_MOTOR_FORWARD_HIGH
        GPIO.output(self.cfg.I1, GPIO.HIGH if forward == high_forward else GPIO.LOW)
        GPIO.output(self.cfg.I2, GPIO.LOW if forward == high_forward else GPIO.HIGH)

    def set_wheels(self, left: float, right: float) -> None:
        """Apply signed PWM duties directly, bypassing speed PID."""
        self._reset_speed_control()
        self._apply_wheels(left, right)

    def _apply_wheels(self, left: float, right: float) -> None:
        self._set_left_dir(left >= 0)
        self._set_right_dir(right >= 0)
        self.pwm_l.ChangeDutyCycle(self._clip(abs(left)))
        self.pwm_r.ChangeDutyCycle(self._clip(abs(right)))

    def drive(self, v: float, w: float) -> None:
        """Drive using normalized forward and turn commands.

        ``v`` and ``w`` are clamped to [-1, 1]. Positive ``w`` turns left
        and negative ``w`` turns right, matching the existing controller.
        """
        left, right = self.wheel_commands(v, w)
        if left == 0.0 and right == 0.0:
            self.stop()
            return
        self._drive_with_speed_pid(left, right)

    def wheel_commands(self, v: float, w: float) -> tuple[float, float]:
        """Convert normalized chassis commands to signed wheel PWM targets."""
        v = max(-1.0, min(1.0, v))
        w = max(-1.0, min(1.0, w))

        if v == 0.0 and w == 0.0:
            return 0.0, 0.0

        turn = self.cfg.TURN_DUTY * w
        if v == 0.0:
            # Rubik-cube spin: equal wheel effort in opposite directions.
            left = -turn
            right = turn
        else:
            base = self.cfg.BASE_DUTY * v
            left = base - turn
            right = base + turn

            # Rubik-cube orbit behavior keeps both wheels moving along the
            # requested travel direction. A strong steering correction may
            # stop the inner wheel, but cannot reverse it into a spin.
            if v > 0.0:
                left = max(0.0, left)
                right = max(0.0, right)
            else:
                left = min(0.0, left)
                right = min(0.0, right)

        return left, right

    def _drive_with_speed_pid(self, left: float, right: float) -> None:
        if self.encoder is None or not self.encoder.available:
            self._apply_wheels(left, right)
            return

        now = time.monotonic()
        counts = self.encoder.counts()
        left_sign = self._sign(left)
        right_sign = self._sign(right)

        if self.last_encoder_counts is None or self.last_pid_time is None:
            self.last_encoder_counts = counts
            self.last_pid_time = now
            self.last_left_duty = abs(left)
            self.last_right_duty = abs(right)
        else:
            direction_changed = (
                left_sign != self.last_left_sign
                or right_sign != self.last_right_sign
            )
            if left_sign != self.last_left_sign:
                self.left_pid.reset()
                self.last_left_duty = abs(left)
            if right_sign != self.last_right_sign:
                self.right_pid.reset()
                self.last_right_duty = abs(right)

            if direction_changed:
                # Unsigned pulses collected before a reversal must not be
                # interpreted as speed in the new direction.
                self.last_encoder_counts = counts
                self.last_pid_time = now
            else:
                dt = now - self.last_pid_time
            if not direction_changed and dt + 1e-9 >= self.cfg.MOTOR_PID_PERIOD_S:
                previous_left, previous_right = self.last_encoder_counts
                left_pulses = max(0, counts[0] - previous_left)
                right_pulses = max(0, counts[1] - previous_right)
                cm_per_pulse = (
                    self.cfg.WHEEL_CIRCUMFERENCE_CM
                    / self.cfg.ENCODER_PULSES_PER_REV
                )
                measured_left = left_pulses * cm_per_pulse / dt
                measured_right = right_pulses * cm_per_pulse / dt
                target_left = (
                    abs(left) / 100.0 * self.cfg.MAX_WHEEL_SPEED_CMPS
                )
                target_right = (
                    abs(right) / 100.0 * self.cfg.MAX_WHEEL_SPEED_CMPS
                )

                self.last_left_duty = self.left_pid.update(
                    target_left,
                    measured_left,
                    abs(left),
                    dt,
                )
                self.last_right_duty = self.right_pid.update(
                    target_right,
                    measured_right,
                    abs(right),
                    dt,
                )
                self.last_encoder_counts = counts
                self.last_pid_time = now

        self.last_left_sign = left_sign
        self.last_right_sign = right_sign
        left_output = self.last_left_duty * left_sign
        right_output = self.last_right_duty * right_sign
        self._apply_wheels(left_output, right_output)

    def move_forward(self, speed: float = 1.0) -> None:
        self.drive(abs(speed), 0.0)

    def move_backward(self, speed: float = 1.0) -> None:
        self.drive(-abs(speed), 0.0)

    def turn_left_not_in_place(self, speed: float = 1.0, strength: float = 0.5) -> None:
        self.drive(abs(speed), abs(strength))

    def turn_right_not_in_place(self, speed: float = 1.0, strength: float = 0.5) -> None:
        self.drive(abs(speed), -abs(strength))

    def turn_in_place(self, direction: int, strength: float = 1.0) -> None:
        if direction == LEFT:
            self.drive(0.0, abs(strength))
        elif direction == RIGHT:
            self.drive(0.0, -abs(strength))
        else:
            raise ValueError(f"direction must be LEFT or RIGHT, got {direction!r}")

    def stop(self) -> None:
        self._reset_speed_control()
        self.pwm_l.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(0)
        GPIO.output([self.cfg.I2, self.cfg.I1, self.cfg.I4, self.cfg.I3], GPIO.LOW)

    def close(self) -> None:
        self.stop()
        self.pwm_l.stop()
        self.pwm_r.stop()

    def _clip(self, duty: float) -> float:
        if duty <= 0.0:
            return 0.0
        return max(self.cfg.MIN_DUTY, min(self.cfg.MAX_DUTY, duty))

    def _reset_speed_control(self) -> None:
        self.left_pid.reset()
        self.right_pid.reset()
        self.last_encoder_counts = None
        self.last_pid_time = None
        self.last_left_duty = 0.0
        self.last_right_duty = 0.0
        self.last_left_sign = 0
        self.last_right_sign = 0

    @staticmethod
    def _sign(value: float) -> int:
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0
