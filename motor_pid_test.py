#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive encoder and wheel-speed PID test.

Put the car on a stand for the first run. Each case waits for confirmation,
drives for a short period, and prints target speed, measured speed, pulse
increments, and the PWM duty produced by each wheel PID.
"""

from __future__ import annotations

import time

import RPi.GPIO as GPIO

from config import Config
from motion import MotorDriver
from perception import EncoderMeter


CONTROL_SLEEP_S = 0.04
LOG_PERIOD_S = 0.5
TEST_SECONDS = 4.0

TEST_CASES = [
    ("straight low", 0.30, 0.0),
    ("straight medium", 0.50, 0.0),
    ("moving left", 0.40, 0.35),
    ("moving right", 0.40, -0.35),
]


def run_case(
    cfg: Config,
    motor: MotorDriver,
    encoder: EncoderMeter,
    name: str,
    v: float,
    w: float,
) -> None:
    input(f"\n{name}: v={v:.2f}, w={w:.2f}. Press Enter to start...")
    motor.stop()
    time.sleep(0.5)

    previous_counts = encoder.counts()
    previous_log_t = time.monotonic()
    start_t = previous_log_t

    try:
        while time.monotonic() - start_t < TEST_SECONDS:
            motor.drive(v, w)
            now = time.monotonic()
            if now - previous_log_t >= LOG_PERIOD_S:
                counts = encoder.counts()
                dt = now - previous_log_t
                left_pulses = counts[0] - previous_counts[0]
                right_pulses = counts[1] - previous_counts[1]
                cm_per_pulse = (
                    cfg.WHEEL_CIRCUMFERENCE_CM / cfg.ENCODER_PULSES_PER_REV
                )
                measured_left = left_pulses * cm_per_pulse / dt
                measured_right = right_pulses * cm_per_pulse / dt
                feed_left, feed_right = motor.wheel_commands(v, w)
                target_left = (
                    abs(feed_left) / 100.0 * cfg.MAX_WHEEL_SPEED_CMPS
                )
                target_right = (
                    abs(feed_right) / 100.0 * cfg.MAX_WHEEL_SPEED_CMPS
                )
                print(
                    f"target=({target_left:5.2f},{target_right:5.2f})cm/s "
                    f"measured=({measured_left:5.2f},{measured_right:5.2f})cm/s "
                    f"pulses=({left_pulses:4d},{right_pulses:4d}) "
                    f"pwm=({motor.last_left_duty:5.1f},{motor.last_right_duty:5.1f})"
                )
                previous_counts = counts
                previous_log_t = now
            time.sleep(CONTROL_SLEEP_S)
    finally:
        motor.stop()
        time.sleep(0.8)


def main() -> None:
    cfg = Config()
    motor = MotorDriver(cfg)
    encoder = EncoderMeter(cfg)
    motor.attach_encoder(encoder)

    if not encoder.available:
        motor.close()
        encoder.close()
        GPIO.cleanup()
        raise RuntimeError("Encoder inputs are unavailable; wheel PID cannot run")

    print("Wheel PID test. Put the car on a stand before the first case.")
    print(
        f"PID: kp={cfg.MOTOR_KP}, ki={cfg.MOTOR_KI}, kd={cfg.MOTOR_KD}, "
        f"period={cfg.MOTOR_PID_PERIOD_S}s, max_speed={cfg.MAX_WHEEL_SPEED_CMPS}cm/s"
    )

    try:
        for case in TEST_CASES:
            run_case(cfg, motor, encoder, *case)
    except KeyboardInterrupt:
        print("\nTest aborted.")
    finally:
        motor.close()
        encoder.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
