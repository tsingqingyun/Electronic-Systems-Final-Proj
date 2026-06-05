#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Central configuration for the fixed-cube obstacle avoidance task.

This file contains only parameters: GPIO assignments, HSV thresholds,
speed settings, ultrasonic limits, encoder settings, and orbit-completion
thresholds. It does not touch Raspberry Pi hardware directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class Config:
    # Motor pins. EA/I1/I2 drive the right motor; EB/I3/I4 drive the left motor.
    EA: int = 13
    I2: int = 19
    I1: int = 26
    EB: int = 16
    I4: int = 20
    I3: int = 21
    PWM_FREQ: int = 50
    LEFT_MOTOR_FORWARD_HIGH: bool = True
    RIGHT_MOTOR_FORWARD_HIGH: bool = True

    # Optional line/boundary sensors. GPIO6/12 are encoders on this car.
    LINE_LEFT: Optional[int] = None
    LINE_RIGHT: Optional[int] = None
    LINE_ACTIVE_LEVEL: int = 0

    # Wheel encoder inputs. B1A measures B/left motor; B2A measures A/right motor.
    ENCODER_LEFT: Optional[int] = 6
    ENCODER_RIGHT: Optional[int] = 12
    ENCODER_PULSES_PER_REV: float = 585.0
    WHEEL_CIRCUMFERENCE_CM: float = 18.85

    # Ultrasonic KS103.
    I2C_BUS: int = 1
    ULTRA_ADDR: int = 0x74
    ULTRA_CMD_MM: int = 0xB0
    STOP_CM: float = 12.0
    SLOW_CM: float = 32.0
    GREEN_ORBIT_ENTER_CM: float = 24.0
    GREEN_ORBIT_MIN_CM: float = 15.0
    GREEN_ORBIT_MAX_CM: float = 34.0

    # Optional MPU6050 gyro. This car currently does not have one.
    USE_GYRO: bool = False
    MPU_ADDR: int = 0x68
    GYRO_Z_SCALE: float = 131.0

    # Camera and vision.
    CAM_INDEX: int = 0
    FRAME_W: int = 640
    FRAME_H: int = 480
    MIN_BLOB_AREA: int = 650
    DEBUG_VIEW: bool = False
    DEBUG_LOG: bool = True

    # HSV ranges in OpenCV scale: H 0..180, S/V 0..255.
    HSV_RANGES: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = None

    # Motion parameters. v is forward speed, w is turn command.
    BASE_DUTY: float = 34.0
    TURN_DUTY: float = 22.0
    MIN_DUTY: float = 0.0
    MAX_DUTY: float = 70.0
    CRUISE_V: float = 0.55
    SLOW_V: float = 0.30
    AVOID_V: float = 0.32
    ORBIT_V: float = 0.34
    CENTER_K: float = 0.95
    AVOID_K: float = 1.05
    ORBIT_SIDE_K: float = 1.10
    ORBIT_DIST_K: float = 0.035
    BOUNDARY_K: float = 0.85

    # Visual steering PID parameters. Error is normalized to roughly [-1, 1].
    # These are intentionally mild because wheel PWM is still open-loop.
    VISION_KP: float = 0.85
    VISION_KI: float = 0.015
    VISION_KD: float = 0.18
    VISION_OUTPUT_LIMIT: float = 0.85

    AVOID_KP: float = 0.95
    AVOID_KI: float = 0.01
    AVOID_KD: float = 0.12
    AVOID_OUTPUT_LIMIT: float = 0.95

    ORBIT_KP: float = 0.95
    ORBIT_KI: float = 0.01
    ORBIT_KD: float = 0.16
    ORBIT_OUTPUT_LIMIT: float = 0.85

    # Fixed-task behavior.
    RED_PASS_SIDE: str = "left"
    YELLOW_PASS_SIDE: str = "right"
    ORBIT_DIRECTION: str = "clockwise"
    ORBIT_TARGET_DEG: float = 360.0
    ORBIT_TIMEOUT_S: float = 16.0
    ORBIT_TIME_FALLBACK_S: float = 10.5

    # Visual loop-closure settings for orbit completion without a gyro.
    ORBIT_MIN_PROGRESS_CM: float = 80.0
    ORBIT_LOOP_CX_TOL: float = 75.0
    ORBIT_LOOP_AREA_MIN_RATIO: float = 0.45
    ORBIT_LOOP_AREA_MAX_RATIO: float = 2.20

    POST_YELLOW_CLEAR_S: float = 4.0
    NEXT_TARGET_DOMINANCE: float = 1.5

    def __post_init__(self) -> None:
        if self.HSV_RANGES is None:
            self.HSV_RANGES = {
                "red": [
                    ((0, 90, 50), (10, 255, 255)),
                    ((168, 90, 50), (180, 255, 255)),
                ],
                "green": [
                    ((38, 70, 45), (88, 255, 255)),
                ],
                "yellow": [
                    ((18, 85, 70), (36, 255, 255)),
                ],
            }
