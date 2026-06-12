#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
High-level task controller.

This file implements the finite-state machine for:
avoid red cube -> approach green cube -> orbit green cube -> avoid yellow
cube -> finish. It combines motor control, camera detections, ultrasonic
distance, and encoder progress.
"""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import Dict, Optional

import RPi.GPIO as GPIO

from config import Config
from motion import IncrementalPID, MotorDriver
from perception import Blob, ColorCamera, EncoderMeter, GyroYaw, LineGuard, UltrasonicKS103


class State(Enum):
    AVOID_RED = auto()
    APPROACH_GREEN = auto()
    ORBIT_GREEN = auto()
    EXIT_ORBIT = auto()
    AVOID_YELLOW = auto()
    FINISH = auto()


class CubeSlalomController:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.motor = MotorDriver(cfg)
        self.ultra = UltrasonicKS103(cfg)
        self.line = LineGuard(cfg)
        self.encoder = EncoderMeter(cfg)
        self.gyro = GyroYaw(cfg)
        self.camera = ColorCamera(cfg)
        self.state = State.AVOID_RED
        self.state_t0 = time.time()
        self.last_seen = {"red": 0.0, "green": 0.0, "yellow": 0.0}
        self.orbit_start_t = 0.0
        self.clear_start_t: Optional[float] = None
        self.last_log_t = 0.0
        self.action = "init"
        self.orbit_start_cx: Optional[float] = None
        self.orbit_start_area: Optional[float] = None
        self.orbit_left_start_view = False
        self.approach_pid = IncrementalPID(
            cfg.VISION_KP, cfg.VISION_KI, cfg.VISION_KD, cfg.VISION_OUTPUT_LIMIT
        )
        self.avoid_pid = IncrementalPID(
            cfg.AVOID_KP, cfg.AVOID_KI, cfg.AVOID_KD, cfg.AVOID_OUTPUT_LIMIT
        )
        self.orbit_pid = IncrementalPID(
            cfg.ORBIT_KP, cfg.ORBIT_KI, cfg.ORBIT_KD, cfg.ORBIT_OUTPUT_LIMIT
        )

    def run(self) -> None:
        try:
            while self.state != State.FINISH:
                self.step()
                time.sleep(0.04)
        finally:
            self.shutdown()

    def step(self) -> None:
        blobs = self.camera.detect()
        dist_cm = self.ultra.read_cm()
        yaw = self.gyro.update()
        now = time.time()
        for color, blob in blobs.items():
            if blob is not None:
                self.last_seen[color] = now
        self._log(blobs, dist_cm, yaw)

        if dist_cm < self.cfg.STOP_CM:
            self.action = "too_close_turn"
            self.motor.drive(0.0, 0.75)
            return

        line_w, line_hit = self.line.correction()
        if line_hit:
            self.action = f"line_guard w={line_w:.2f}"
            self.motor.drive(0.18, line_w)
            return

        if self.state == State.AVOID_RED:
            self._avoid_color("red", blobs["red"], blobs["green"], self.cfg.RED_PASS_SIDE, dist_cm)
        elif self.state == State.APPROACH_GREEN:
            self._approach_green(blobs["green"], dist_cm)
        elif self.state == State.ORBIT_GREEN:
            self._orbit_green(blobs["green"], dist_cm, yaw)
        elif self.state == State.EXIT_ORBIT:
            self._exit_orbit(blobs["yellow"], dist_cm)
        elif self.state == State.AVOID_YELLOW:
            self._avoid_yellow(blobs["yellow"], dist_cm)

    def _avoid_color(
        self,
        color: str,
        obstacle: Optional[Blob],
        next_target: Optional[Blob],
        pass_side: str,
        dist_cm: float,
    ) -> None:
        passed_obstacle = (
            obstacle is None
            or next_target is not None
            and next_target.size_score > obstacle.size_score * self.cfg.NEXT_TARGET_DOMINANCE
        )
        if next_target is not None and passed_obstacle and self.elapsed() > 1.2:
            self.set_state(State.APPROACH_GREEN)
            return

        if obstacle is None:
            self.avoid_pid.reset()
            self.action = f"{color}_not_seen_forward"
            self.motor.drive(self._speed_by_distance(dist_cm), 0.0)
            return

        target_x = self.cfg.FRAME_W * (0.70 if pass_side == "left" else 0.30)
        err = (target_x - obstacle.cx) / (self.cfg.FRAME_W / 2.0)
        w = self.avoid_pid.update(err)
        if pass_side == "left" and obstacle.cx < self.cfg.FRAME_W * 0.48:
            w += 0.25
        if pass_side == "right" and obstacle.cx > self.cfg.FRAME_W * 0.52:
            w -= 0.25
        self.action = f"avoid_{color} cx={obstacle.cx:.0f} w={self._clip_unit(w):.2f}"
        self.motor.drive(self.cfg.AVOID_V, self._clip_unit(w))

    def _approach_green(self, green: Optional[Blob], dist_cm: float) -> None:
        if green is None:
            self.approach_pid.reset()
            self.action = "green_not_seen_forward"
            self.motor.drive(self.cfg.SLOW_V, 0.0)
            return

        center_err = (self.cfg.FRAME_W / 2.0 - green.cx) / (self.cfg.FRAME_W / 2.0)
        close_by_size = green.size_score > self.cfg.FRAME_W * self.cfg.FRAME_H * 0.055
        close_by_dist = dist_cm < self.cfg.GREEN_ORBIT_ENTER_CM
        if close_by_size or close_by_dist:
            self.gyro.reset()
            self.encoder.reset()
            self._start_visual_orbit(green)
            self.orbit_start_t = time.time()
            self.set_state(State.ORBIT_GREEN)
            return

        w = self._clip_unit(self.approach_pid.update(center_err))
        self.action = f"approach_green cx={green.cx:.0f} err={center_err:.2f} w={w:.2f}"
        self.motor.drive(self._speed_by_distance(dist_cm), w)

    def _orbit_green(self, green: Optional[Blob], dist_cm: float, yaw: float) -> None:
        now = time.time()
        clockwise = self.cfg.ORBIT_DIRECTION == "clockwise"
        target_x = self.cfg.FRAME_W * (0.72 if clockwise else 0.28)

        if green is None:
            self.orbit_pid.reset()
            search_w = -0.55 if clockwise else 0.55
            self.action = f"orbit_search w={search_w:.2f}"
            self.motor.drive(0.12, search_w)
        else:
            side_err = (target_x - green.cx) / (self.cfg.FRAME_W / 2.0)
            dist_err = self._orbit_distance_error(dist_cm)
            tangent_w = -0.38 if clockwise else 0.38
            w = tangent_w + self.orbit_pid.update(side_err) + dist_err
            self.action = f"orbit_green cx={green.cx:.0f} w={self._clip_unit(w):.2f} yaw={yaw:.0f}"
            self.motor.drive(self.cfg.ORBIT_V, self._clip_unit(w))

        gyro_done = self.gyro.available and abs(yaw) >= self.cfg.ORBIT_TARGET_DEG
        visual_done = (not self.gyro.available) and self._visual_orbit_done(green)
        fallback_done = (
            not self.gyro.available
            and not self.encoder.available
            and now - self.orbit_start_t >= self.cfg.ORBIT_TIME_FALLBACK_S
        )
        timeout = now - self.orbit_start_t >= self.cfg.ORBIT_TIMEOUT_S
        if gyro_done or visual_done or fallback_done or timeout:
            self.set_state(State.EXIT_ORBIT)

    def _exit_orbit(self, yellow: Optional[Blob], dist_cm: float) -> None:
        if yellow is not None:
            self.set_state(State.AVOID_YELLOW)
            return
        self.action = "exit_orbit_forward"
        self.motor.drive(self._speed_by_distance(dist_cm), 0.0)

    def _avoid_yellow(self, yellow: Optional[Blob], dist_cm: float) -> None:
        now = time.time()
        if yellow is None:
            self.action = "yellow_clear_forward"
            self.motor.drive(self._speed_by_distance(dist_cm), 0.0)
            if self.clear_start_t is None:
                self.clear_start_t = now
            if now - self.clear_start_t >= self.cfg.POST_YELLOW_CLEAR_S:
                self.set_state(State.FINISH)
            return

        self.clear_start_t = None
        self._avoid_color("yellow", yellow, None, self.cfg.YELLOW_PASS_SIDE, dist_cm)

    def _speed_by_distance(self, dist_cm: float) -> float:
        if dist_cm < self.cfg.SLOW_CM:
            return self.cfg.SLOW_V
        return self.cfg.CRUISE_V

    def _orbit_distance_error(self, dist_cm: float) -> float:
        if dist_cm > 400:
            return 0.0
        if dist_cm < self.cfg.GREEN_ORBIT_MIN_CM:
            return -0.35 if self.cfg.ORBIT_DIRECTION == "clockwise" else 0.35
        if dist_cm > self.cfg.GREEN_ORBIT_MAX_CM:
            return 0.20 if self.cfg.ORBIT_DIRECTION == "clockwise" else -0.20
        target = (self.cfg.GREEN_ORBIT_MIN_CM + self.cfg.GREEN_ORBIT_MAX_CM) / 2.0
        err = (target - dist_cm) * self.cfg.ORBIT_DIST_K
        return -err if self.cfg.ORBIT_DIRECTION == "clockwise" else err

    def _start_visual_orbit(self, green: Blob) -> None:
        self.orbit_start_cx = green.cx
        self.orbit_start_area = max(green.area, 1.0)
        self.orbit_left_start_view = False

    def _visual_orbit_done(self, green: Optional[Blob]) -> bool:
        if self.orbit_start_cx is None or self.orbit_start_area is None:
            return False
        elapsed = time.time() - self.orbit_start_t
        progress = self.encoder.progress_cm()

        if green is None:
            if elapsed > 1.0:
                self.orbit_left_start_view = True
            return False

        cx_delta = abs(green.cx - self.orbit_start_cx)
        area_ratio = green.area / self.orbit_start_area
        if cx_delta > self.cfg.ORBIT_LOOP_CX_TOL * 1.5:
            self.orbit_left_start_view = True
        if self.encoder.available and progress > self.cfg.ORBIT_MIN_PROGRESS_CM * 0.5:
            self.orbit_left_start_view = True

        enough_progress = (
            progress >= self.cfg.ORBIT_MIN_PROGRESS_CM
            if self.encoder.available
            else elapsed >= self.cfg.ORBIT_TIME_FALLBACK_S * 0.6
        )
        similar_view = (
            cx_delta <= self.cfg.ORBIT_LOOP_CX_TOL
            and self.cfg.ORBIT_LOOP_AREA_MIN_RATIO <= area_ratio <= self.cfg.ORBIT_LOOP_AREA_MAX_RATIO
        )
        if self.orbit_left_start_view and enough_progress and similar_view:
            self.action = f"visual_loop_closed progress={progress:.1f} area_ratio={area_ratio:.2f}"
            return True
        return False

    def set_state(self, state: State) -> None:
        print(f"{time.time():.2f}: {self.state.name} -> {state.name}")
        self.state = state
        self.state_t0 = time.time()
        self.approach_pid.reset()
        self.avoid_pid.reset()
        self.orbit_pid.reset()
        if state == State.FINISH:
            self.motor.stop()

    def elapsed(self) -> float:
        return time.time() - self.state_t0

    @staticmethod
    def _clip_unit(x: float) -> float:
        return max(-1.0, min(1.0, x))

    def _log(self, blobs: Dict[str, Optional[Blob]], dist_cm: float, yaw: float) -> None:
        if not self.cfg.DEBUG_LOG:
            return
        now = time.time()
        if now - self.last_log_t < 1.0:
            return
        self.last_log_t = now
        seen = []
        for color in ["red", "green", "yellow"]:
            blob = blobs.get(color)
            if blob is not None:
                seen.append(f"{color}:cx={blob.cx:.0f},area={blob.area:.0f}")
        seen_text = "; ".join(seen) if seen else "none"
        print(
            f"state={self.state.name} dist={dist_cm:.1f}cm yaw={yaw:.0f} "
            f"enc={self.encoder.progress_cm():.1f}cm seen=[{seen_text}] action={self.action}"
        )

    def shutdown(self) -> None:
        self.motor.close()
        self.camera.close()
        self.ultra.close()
        self.encoder.close()
        self.gyro.close()
        GPIO.cleanup()
