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
    GREEN_ORBIT_RADIUS_CM: float = 29.0
    GREEN_ORBIT_RADIUS_TOL_CM: float = 3.0

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
    # Calibrate this value with motor_pid_test.py before tuning PID gains.
    MAX_WHEEL_SPEED_CMPS: float = 25.0
    MOTOR_PID_PERIOD_S: float = 0.10
    MOTOR_KP: float = 2.0
    MOTOR_KI: float = 0.0
    MOTOR_KD: float = 0.0
    MOTOR_INTEGRAL_LIMIT: float = 20.0
    CRUISE_V: float = 0.55
    SLOW_V: float = 0.30
    CLEAR_V: float = 0.40
    AVOID_V: float = 0.32
    ORBIT_V: float = 0.34
    BOUNDARY_K: float = 0.85

    # Visual steering PID parameters. Error is normalized to roughly [-1, 1].
    VISION_KP: float = 0.85
    VISION_KI: float = 0.015
    VISION_KD: float = 0.18
    VISION_OUTPUT_LIMIT: float = 0.85

    # Fixed-task behavior.
    RED_PASS_SIDE: str = "left"
    GREEN_PASS_SIDE: str = "left"
    YELLOW_PASS_SIDE: str = "right"
    ORBIT_DIRECTION: str = "clockwise"

    # State-transition confirmation. A transition is never based on one frame.
    TARGET_CONFIRM_FRAMES: int = 5
    TARGET_LOST_FRAMES: int = 8
    GREEN_CENTER_TOL_PX: float = 75.0
    GREEN_APPROACH_AREA_RATIO: float = 0.055
    PASS_NEAR_AREA_RATIO: float = 0.035
    PASS_NEAR_CM: float = 30.0

    # Symmetric lane change: move aside, pass, then return to the start-finish line.
    AVOID_TURN_W: float = 0.24
    AVOID_ARC_CM: float = 16.0
    AVOID_PASS_CM: float = 30.0
    AVOID_CLEAR_CM: float = 20.0

    # Green orbit: one regular octagon, then the same lane-change exit.
    GREEN_ORBIT_SEGMENTS: int = 8
    GREEN_ENTRY_TURN_CM: float = 10.5
    GREEN_CORNER_TURN_CM: float = 7.0
    GREEN_ORBIT_LEG_CM: float = 22.0
    GREEN_TURN_W: float = 0.72
    GREEN_RANGE_V: float = 0.18

    # Time only detects a stuck state; it never proves task completion.
    FIND_TARGET_TIMEOUT_S: float = 18.0
    PASS_CUBE_TIMEOUT_S: float = 14.0
    MANEUVER_TIMEOUT_S: float = 12.0
    ORBIT_TOTAL_TIMEOUT_S: float = 120.0
    CLEAR_CUBE_TIMEOUT_S: float = 20.0

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
