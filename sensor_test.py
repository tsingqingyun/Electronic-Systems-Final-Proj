#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read ultrasonic distance and encoder counts without driving the motors."""

from __future__ import annotations

import time

import RPi.GPIO as GPIO

from config import Config
from perception import EncoderMeter, UltrasonicKS103


def main() -> None:
    cfg = Config()
    ultrasonic = UltrasonicKS103(cfg)
    encoder = EncoderMeter(cfg)
    encoder.reset()

    print("Move an object in front of the KS103 and rotate each wheel by hand.")
    print("Press Ctrl+C to stop.")
    try:
        while True:
            left, right = encoder.counts()
            print(
                f"distance={ultrasonic.read_cm():6.1f}cm "
                f"counts=({left:6d},{right:6d}) "
                f"progress={encoder.progress_cm():6.2f}cm "
                f"encoder_available={encoder.available}"
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nSensor test stopped.")
    finally:
        ultrasonic.close()
        encoder.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
