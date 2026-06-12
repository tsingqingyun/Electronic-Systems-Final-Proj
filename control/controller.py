#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
High-level task controller.

This file implements a confirmed finite-state machine for the fixed course.
Vision identifies targets, ultrasonic ranging only supports near-distance
checks, and encoders prove that the chassis has cleared each cube.
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
    FIND_RED = auto()
    PASS_RED = auto()
    CLEAR_RED = auto()
    FIND_GREEN = auto()
    APPROACH_GREEN = auto()
    ORBIT_GREEN = auto()
    EXIT_GREEN = auto()
    FIND_YELLOW = auto()
    PASS_YELLOW = auto()
    CLEAR_YELLOW = auto()
    RECOVERY = auto()
    FINISH = auto()


class CubeSlalomController:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.motor = MotorDriver(cfg)
        self.ultra = UltrasonicKS103(cfg)
        self.line = LineGuard(cfg)
        self.encoder = EncoderMeter(cfg)
        self.motor.attach_encoder(self.encoder)
        self.gyro = GyroYaw(cfg)
        self.camera = ColorCamera(cfg)
        self.state = State.FIND_RED
        self.state_t0 = time.time()
        self.last_seen = {"red": 0.0, "green": 0.0, "yellow": 0.0}
        self.orbit_start_t = 0.0
        self.last_log_t = 0.0
        self.action = "init"
        self.failure_reason = ""
        self.visible_frames = 0
        self.lost_frames = 0
        self.completion_frames = 0
        self.target_armed = False
        self.reached_near_zone = False
        self.reached_expected_edge = False
        self.last_pass_w = 0.0
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

        if self.state == State.RECOVERY:
            self.action = f"recovery_stop reason={self.failure_reason}"
            self.motor.stop()
            self._log(blobs, dist_cm, yaw)
            return

        timeout_limit = self._state_timeout_limit()
        if timeout_limit is not None and self._timed_out(timeout_limit):
            self._recover(f"{self.state.name.lower()}_timeout")
            self._log(blobs, dist_cm, yaw)
            return

        if dist_cm < self.cfg.STOP_CM:
            turn_w = self._emergency_turn(blobs)
            self.action = f"too_close_turn w={turn_w:.2f}"
            self.motor.drive(0.0, turn_w)
            self._log(blobs, dist_cm, yaw)
            return

        line_w, line_hit = self.line.correction()
        if line_hit:
            self.action = f"line_guard w={line_w:.2f}"
            self.motor.drive(0.18, line_w)
            self._log(blobs, dist_cm, yaw)
            return

        if self.state == State.FIND_RED:
            self._find_target("red", blobs["red"], State.PASS_RED)
        elif self.state == State.PASS_RED:
            self._pass_cube("red", blobs["red"], self.cfg.RED_PASS_SIDE, dist_cm)
        elif self.state == State.CLEAR_RED:
            self._clear_cube("red", self.cfg.RED_CLEAR_CM, State.FIND_GREEN)
        elif self.state == State.FIND_GREEN:
            self._find_target("green", blobs["green"], State.APPROACH_GREEN)
        elif self.state == State.APPROACH_GREEN:
            self._approach_green(blobs["green"], dist_cm)
        elif self.state == State.ORBIT_GREEN:
            self._orbit_green(blobs["green"], dist_cm, yaw)
        elif self.state == State.EXIT_GREEN:
            self._exit_green(blobs["green"])
        elif self.state == State.FIND_YELLOW:
            self._find_target("yellow", blobs["yellow"], State.PASS_YELLOW)
        elif self.state == State.PASS_YELLOW:
            self._pass_cube("yellow", blobs["yellow"], self.cfg.YELLOW_PASS_SIDE, dist_cm)
        elif self.state == State.CLEAR_YELLOW:
            self._clear_cube("yellow", self.cfg.YELLOW_CLEAR_CM, State.FINISH)

        self._log(blobs, dist_cm, yaw)

    def _find_target(self, color: str, target: Optional[Blob], next_state: State) -> None:
        self._update_visibility(target)
        if self.visible_frames >= self.cfg.TARGET_CONFIRM_FRAMES:
            self.set_state(next_state)
            return
        if self._timed_out(self.cfg.FIND_TARGET_TIMEOUT_S):
            self._recover(f"{self.state.name.lower()}_timeout")
            return

        self.action = f"find_{color} seen={self.visible_frames}"
        self.motor.drive(self.cfg.SLOW_V, 0.0)

    def _pass_cube(
        self,
        color: str,
        obstacle: Optional[Blob],
        pass_side: str,
        dist_cm: float,
    ) -> None:
        self._update_visibility(obstacle)

        if obstacle is not None:
            if self.visible_frames >= self.cfg.TARGET_CONFIRM_FRAMES:
                self.target_armed = True

            area_ratio = obstacle.size_score / (self.cfg.FRAME_W * self.cfg.FRAME_H)
            near = area_ratio >= self.cfg.PASS_NEAR_AREA_RATIO or (
                0 < dist_cm <= self.cfg.PASS_NEAR_CM
            )
            if self.target_armed and near:
                self.reached_near_zone = True

            edge_x = self.cfg.FRAME_W * self.cfg.PASS_EDGE_RATIO
            at_expected_edge = (
                obstacle.cx >= edge_x
                if pass_side == "left"
                else obstacle.cx <= self.cfg.FRAME_W - edge_x
            )
            if self.target_armed and at_expected_edge:
                self.reached_expected_edge = True

            target_x = edge_x if pass_side == "left" else self.cfg.FRAME_W - edge_x
            err = (target_x - obstacle.cx) / (self.cfg.FRAME_W / 2.0)
            w = self.avoid_pid.update(err)
            if pass_side == "left" and obstacle.cx < self.cfg.FRAME_W * 0.48:
                w += 0.25
            if pass_side == "right" and obstacle.cx > self.cfg.FRAME_W * 0.52:
                w -= 0.25
            self.last_pass_w = self._clip_unit(w)
            self.action = (
                f"pass_{color} cx={obstacle.cx:.0f} near={self.reached_near_zone} "
                f"edge={self.reached_expected_edge} w={self.last_pass_w:.2f}"
            )
            self.motor.drive(self.cfg.AVOID_V, self.last_pass_w)
        else:
            hold_w = self.last_pass_w * 0.65
            self.action = (
                f"pass_{color}_lost lost={self.lost_frames} "
                f"armed={self.target_armed} w={hold_w:.2f}"
            )
            self.motor.drive(self.cfg.SLOW_V, hold_w)

        passed = (
            self.target_armed
            and self.reached_near_zone
            and self.reached_expected_edge
            and self.lost_frames >= self.cfg.TARGET_LOST_FRAMES
        )
        if passed:
            next_state = State.CLEAR_RED if color == "red" else State.CLEAR_YELLOW
            self.set_state(next_state)
            return
        if self._timed_out(self.cfg.PASS_CUBE_TIMEOUT_S):
            self._recover(f"pass_{color}_timeout")

    def _clear_cube(self, color: str, required_cm: float, next_state: State) -> None:
        if not self.encoder.available:
            self._recover(f"clear_{color}_encoder_unavailable")
            return

        progress = self.encoder.progress_cm()
        if progress >= required_cm:
            self.set_state(next_state)
            return
        if self._timed_out(self.cfg.CLEAR_CUBE_TIMEOUT_S):
            self._recover(f"clear_{color}_timeout")
            return

        self.action = f"clear_{color} progress={progress:.1f}/{required_cm:.1f}cm"
        self.motor.drive(self.cfg.SLOW_V, 0.0)

    def _approach_green(self, green: Optional[Blob], dist_cm: float) -> None:
        self._update_visibility(green)
        if green is None:
            self.approach_pid.reset()
            self.completion_frames = 0
            if self.lost_frames >= self.cfg.TARGET_LOST_FRAMES:
                self.set_state(State.FIND_GREEN)
                return
            if self._timed_out(self.cfg.APPROACH_GREEN_TIMEOUT_S):
                self._recover("approach_green_timeout")
                return
            self.action = f"approach_green_lost lost={self.lost_frames}"
            self.motor.drive(self.cfg.SLOW_V, 0.0)
            return

        center_err = (self.cfg.FRAME_W / 2.0 - green.cx) / (self.cfg.FRAME_W / 2.0)
        centered = abs(green.cx - self.cfg.FRAME_W / 2.0) <= self.cfg.GREEN_CENTER_TOL_PX
        close_by_size = (
            green.size_score
            >= self.cfg.FRAME_W * self.cfg.FRAME_H * self.cfg.GREEN_APPROACH_AREA_RATIO
        )
        close_by_dist = 0 < dist_cm < self.cfg.GREEN_ORBIT_ENTER_CM
        ready = (
            centered
            and (close_by_size or close_by_dist)
        )
        self.completion_frames = self.completion_frames + 1 if ready else 0
        if self.completion_frames >= self.cfg.TARGET_CONFIRM_FRAMES:
            self._start_visual_orbit(green)
            self.set_state(State.ORBIT_GREEN)
            return

        w = self._clip_unit(self.approach_pid.update(center_err))
        self.action = (
            f"approach_green cx={green.cx:.0f} centered={centered} "
            f"ready={self.completion_frames} w={w:.2f}"
        )
        self.motor.drive(self._speed_by_distance(dist_cm), w)
        if self._timed_out(self.cfg.APPROACH_GREEN_TIMEOUT_S):
            self._recover("approach_green_timeout")

    def _orbit_green(self, green: Optional[Blob], dist_cm: float, yaw: float) -> None:
        if not self.gyro.available and not self.encoder.available:
            self._recover("orbit_completion_sensor_unavailable")
            return

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
        visual_done = not self.gyro.available and self._visual_orbit_condition(green)
        self.completion_frames = self.completion_frames + 1 if visual_done else 0
        if gyro_done or self.completion_frames >= self.cfg.ORBIT_CONFIRM_FRAMES:
            self.set_state(State.EXIT_GREEN)
            return
        if self._timed_out(self.cfg.ORBIT_TIMEOUT_S):
            self._recover("orbit_green_timeout")

    def _exit_green(self, green: Optional[Blob]) -> None:
        if not self.encoder.available:
            self._recover("exit_green_encoder_unavailable")
            return

        self._update_visibility(green)
        progress = self.encoder.progress_cm()
        cleared_view = self.lost_frames >= self.cfg.TARGET_LOST_FRAMES
        if progress >= self.cfg.GREEN_EXIT_CM and cleared_view:
            self.set_state(State.FIND_YELLOW)
            return
        if self._timed_out(self.cfg.EXIT_GREEN_TIMEOUT_S):
            self._recover("exit_green_timeout")
            return

        self.action = (
            f"exit_green progress={progress:.1f}/{self.cfg.GREEN_EXIT_CM:.1f}cm "
            f"lost={self.lost_frames}"
        )
        self.motor.drive(self.cfg.SLOW_V, 0.0)

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

    def _visual_orbit_condition(self, green: Optional[Blob]) -> bool:
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
            else False
        )
        similar_view = (
            cx_delta <= self.cfg.ORBIT_LOOP_CX_TOL
            and self.cfg.ORBIT_LOOP_AREA_MIN_RATIO <= area_ratio <= self.cfg.ORBIT_LOOP_AREA_MAX_RATIO
        )
        return self.orbit_left_start_view and enough_progress and similar_view

    def set_state(self, state: State) -> None:
        print(f"{time.time():.2f}: {self.state.name} -> {state.name}")
        self.state = state
        self.state_t0 = time.time()
        self.visible_frames = 0
        self.lost_frames = 0
        self.completion_frames = 0
        self.target_armed = False
        self.reached_near_zone = False
        self.reached_expected_edge = False
        self.last_pass_w = 0.0
        self.approach_pid.reset()
        self.avoid_pid.reset()
        self.orbit_pid.reset()
        if state in {State.CLEAR_RED, State.ORBIT_GREEN, State.EXIT_GREEN, State.CLEAR_YELLOW}:
            self.encoder.reset()
        if state == State.ORBIT_GREEN:
            self.gyro.reset()
            self.orbit_start_t = time.time()
        if state in {State.RECOVERY, State.FINISH}:
            self.motor.stop()

    def elapsed(self) -> float:
        return time.time() - self.state_t0

    def _update_visibility(self, blob: Optional[Blob]) -> None:
        if blob is None:
            self.visible_frames = 0
            self.lost_frames += 1
        else:
            self.visible_frames += 1
            self.lost_frames = 0

    def _emergency_turn(self, blobs: Dict[str, Optional[Blob]]) -> float:
        if self.state in {State.FIND_RED, State.PASS_RED, State.CLEAR_RED}:
            return 0.75 if self.cfg.RED_PASS_SIDE == "left" else -0.75
        if self.state in {State.FIND_YELLOW, State.PASS_YELLOW, State.CLEAR_YELLOW}:
            return 0.75 if self.cfg.YELLOW_PASS_SIDE == "left" else -0.75
        if self.state == State.ORBIT_GREEN:
            return -0.75 if self.cfg.ORBIT_DIRECTION == "clockwise" else 0.75

        for color in ("green", "red", "yellow"):
            blob = blobs.get(color)
            if blob is not None:
                return -0.75 if blob.cx < self.cfg.FRAME_W / 2.0 else 0.75
        return 0.75

    def _timed_out(self, limit_s: float) -> bool:
        return self.elapsed() >= limit_s

    def _state_timeout_limit(self) -> Optional[float]:
        if self.state in {State.FIND_RED, State.FIND_GREEN, State.FIND_YELLOW}:
            return self.cfg.FIND_TARGET_TIMEOUT_S
        if self.state in {State.PASS_RED, State.PASS_YELLOW}:
            return self.cfg.PASS_CUBE_TIMEOUT_S
        if self.state == State.APPROACH_GREEN:
            return self.cfg.APPROACH_GREEN_TIMEOUT_S
        if self.state == State.ORBIT_GREEN:
            return self.cfg.ORBIT_TIMEOUT_S
        if self.state in {State.CLEAR_RED, State.CLEAR_YELLOW}:
            return self.cfg.CLEAR_CUBE_TIMEOUT_S
        if self.state == State.EXIT_GREEN:
            return self.cfg.EXIT_GREEN_TIMEOUT_S
        return None

    def _recover(self, reason: str) -> None:
        self.failure_reason = reason
        self.action = f"recovery_stop reason={reason}"
        self.set_state(State.RECOVERY)

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
