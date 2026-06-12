#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Incremental PID controller used by the visual steering loops.

The formula follows the PID style used in the referenced rubik-cube
repository and the course PID examples:

delta_u = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
u[k] = u[k-1] + delta_u
"""

from __future__ import annotations


class IncrementalPID:
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.errors = [0.0, 0.0, 0.0]
        self.output = 0.0

    def reset(self) -> None:
        self.errors = [0.0, 0.0, 0.0]
        self.output = 0.0

    def update(self, error: float) -> float:
        self.errors[0] = self.errors[1]
        self.errors[1] = self.errors[2]
        self.errors[2] = error

        delta = (
            self.kp * (self.errors[2] - self.errors[1])
            + self.ki * self.errors[2]
            + self.kd * (self.errors[2] - 2 * self.errors[1] + self.errors[0])
        )
        self.output += delta
        return self._clip(self.output)

    def _clip(self, value: float) -> float:
        if value > self.output_limit:
            return self.output_limit
        if value < -self.output_limit:
            return -self.output_limit
        return value


class WheelSpeedPID:
    """Positional PID that corrects one wheel's feed-forward PWM duty."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
        integral_limit: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def update(
        self,
        target_cmps: float,
        measured_cmps: float,
        feedforward_duty: float,
        dt: float,
    ) -> float:
        if dt <= 0.0 or target_cmps <= 0.0:
            self.reset()
            return 0.0

        error = target_cmps - measured_cmps
        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral),
        )
        derivative = (
            (error - self.previous_error) / dt
            if self.initialized
            else 0.0
        )

        output = (
            feedforward_duty
            + self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        clipped = max(self.output_min, min(self.output_max, output))

        # Do not keep integrating farther into saturation.
        if clipped != output:
            pushes_high = output > self.output_max and error > 0.0
            pushes_low = output < self.output_min and error < 0.0
            if pushes_high or pushes_low:
                self.integral -= error * dt

        self.previous_error = error
        self.initialized = True
        return clipped
