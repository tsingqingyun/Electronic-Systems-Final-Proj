#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Sensor drivers and low-level sensor helpers.

This file contains the KS103 ultrasonic reader, optional line/boundary
guard, and wheel encoder progress meter.
"""

from __future__ import annotations

import time
import threading
from typing import Optional, Tuple

import RPi.GPIO as GPIO
import smbus

from config import Config


class UltrasonicKS103:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.bus = smbus.SMBus(cfg.I2C_BUS)
        self.last_cm = 999.0
        self.last_t = 0.0

    def read_cm(self) -> float:
        now = time.time()
        if now - self.last_t < 0.035:
            return self.last_cm
        try:
            self.bus.write_byte_data(self.cfg.ULTRA_ADDR, 0x02, self.cfg.ULTRA_CMD_MM)
            time.sleep(0.035)
            high = self.bus.read_byte_data(self.cfg.ULTRA_ADDR, 0x02)
            low = self.bus.read_byte_data(self.cfg.ULTRA_ADDR, 0x03)
            mm = (high << 8) + low
            if 20 <= mm <= 5000:
                self.last_cm = mm / 10.0
        except OSError:
            pass
        self.last_t = time.time()
        return self.last_cm

    def close(self) -> None:
        self.bus.close()


class LineGuard:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        pins = [p for p in [cfg.LINE_LEFT, cfg.LINE_RIGHT] if p is not None]
        if pins:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pins, GPIO.IN)

    def correction(self) -> Tuple[float, bool]:
        left_hit = self._hit(self.cfg.LINE_LEFT)
        right_hit = self._hit(self.cfg.LINE_RIGHT)
        if left_hit and right_hit:
            return 0.0, True
        if left_hit:
            return -self.cfg.BOUNDARY_K, True
        if right_hit:
            return self.cfg.BOUNDARY_K, True
        return 0.0, False

    def _hit(self, pin: Optional[int]) -> bool:
        if pin is None:
            return False
        return GPIO.input(pin) == self.cfg.LINE_ACTIVE_LEVEL


class EncoderMeter:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.left_count = 0
        self.right_count = 0
        self.left_origin = 0
        self.right_origin = 0
        self.lock = threading.Lock()
        self.available = False
        pins = [p for p in [cfg.ENCODER_LEFT, cfg.ENCODER_RIGHT] if p is not None]
        if not pins:
            return
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pins, GPIO.IN)
            if cfg.ENCODER_LEFT is not None:
                GPIO.add_event_detect(cfg.ENCODER_LEFT, GPIO.RISING, callback=self._on_pulse)
            if cfg.ENCODER_RIGHT is not None:
                GPIO.add_event_detect(cfg.ENCODER_RIGHT, GPIO.RISING, callback=self._on_pulse)
            self.available = True
        except RuntimeError:
            self.available = False

    def _on_pulse(self, channel: int) -> None:
        with self.lock:
            if channel == self.cfg.ENCODER_LEFT:
                self.left_count += 1
            elif channel == self.cfg.ENCODER_RIGHT:
                self.right_count += 1

    def reset(self) -> None:
        with self.lock:
            self.left_origin = self.left_count
            self.right_origin = self.right_count

    def counts(self) -> Tuple[int, int]:
        """Return lifetime left/right pulse counts without resetting them."""
        with self.lock:
            return self.left_count, self.right_count

    def progress_cm(self) -> float:
        if not self.available:
            return 0.0
        left_cm, right_cm = self.wheel_progress_cm()
        return (left_cm + right_cm) / 2.0

    def wheel_progress_cm(self) -> Tuple[float, float]:
        """Return unsigned left/right wheel travel since the last reset."""
        if not self.available:
            return 0.0, 0.0
        with self.lock:
            left = self.left_count - self.left_origin
            right = self.right_count - self.right_origin
        cm_per_pulse = self.cfg.WHEEL_CIRCUMFERENCE_CM / self.cfg.ENCODER_PULSES_PER_REV
        return left * cm_per_pulse, right * cm_per_pulse

    def close(self) -> None:
        for pin in [self.cfg.ENCODER_LEFT, self.cfg.ENCODER_RIGHT]:
            if pin is None:
                continue
            try:
                GPIO.remove_event_detect(pin)
            except RuntimeError:
                pass
