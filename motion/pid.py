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
