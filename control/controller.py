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
from perception import Blob, ColorCamera, EncoderMeter, LineGuard, UltrasonicKS103


class State(Enum):
    DRIVE_TO_TARGET = auto()
    APPROACH_TARGET = auto()
    AVOID_OFFSET_OUT = auto()
    AVOID_OUT_ALIGN = auto()
    AVOID_PASS = auto()
    AVOID_OFFSET_BACK = auto()
    AVOID_BACK_ALIGN = auto()
    CLEAR_TARGET = auto()
    GREEN_ENTRY_TURN = auto()
    GREEN_LEG = auto()
    GREEN_CORNER_TURN = auto()
    GREEN_RESTORE_HEADING = auto()
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
        self.camera = ColorCamera(cfg)
        self.course = ("red", "green", "yellow")
        self.target_index = 0
        self.state = State.DRIVE_TO_TARGET
        self.state_t0 = time.time()
        self.orbit_total_t0 = 0.0
        self.orbit_segment = 0
        self.last_log_t = 0.0
        self.action = "init"
        self.failure_reason = ""
        self.visible_frames = 0
        self.lost_frames = 0
        self.completion_frames = 0
        self.approach_pid = IncrementalPID(
            cfg.VISION_KP, cfg.VISION_KI, cfg.VISION_KD, cfg.VISION_OUTPUT_LIMIT
        )

    def run(self) -> None:
        try:
            while self.state not in {State.FINISH, State.RECOVERY}:
                self.step()
                time.sleep(0.04)
        finally:
            self.shutdown()

    def step(self) -> None:
        blobs = self.camera.detect()
        dist_cm = self.ultra.read_cm()

        if self.state == State.RECOVERY:
            self.action = f"recovery_stop reason={self.failure_reason}"
            self.motor.stop()
            self._log(blobs, dist_cm)
            return

        timeout_limit = self._state_timeout_limit()
        if timeout_limit is not None and self._timed_out(timeout_limit):
            self._recover(f"{self.state.name.lower()}_timeout")
            self._log(blobs, dist_cm)
            return

        if self._in_green_orbit() and self.orbit_total_t0 > 0.0 and (
            time.time() - self.orbit_total_t0 >= self.cfg.ORBIT_TOTAL_TIMEOUT_S
        ):
            self._recover("green_orbit_total_timeout")
            self._log(blobs, dist_cm)
            return

        if 0 < dist_cm < self.cfg.STOP_CM:
            if self.state == State.APPROACH_TARGET:
                self.completion_frames = 0
                self.action = "too_close_back"
                self.motor.drive(-self.cfg.GREEN_RANGE_V, 0.0)
                self._log(blobs, dist_cm)
                return
            self._recover(f"{self.state.name.lower()}_too_close")
            self._log(blobs, dist_cm)
            return

        line_w, line_hit = self.line.correction()
        if line_hit:
            self.action = f"line_guard w={line_w:.2f}"
            self.motor.drive(0.18, line_w)
            self._log(blobs, dist_cm)
            return

        color = self._target_color()
        target = blobs[color] if color is not None else None
        if self.state == State.DRIVE_TO_TARGET:
            self._drive_to_target(color, target)
        elif self.state == State.APPROACH_TARGET:
            self._approach_target(color, target, dist_cm)
        elif self.state == State.AVOID_OFFSET_OUT:
            self._avoid_arc("offset_out", self._pass_sign(), State.AVOID_OUT_ALIGN)
        elif self.state == State.AVOID_OUT_ALIGN:
            self._avoid_arc("out_align", -self._pass_sign(), State.AVOID_PASS)
        elif self.state == State.AVOID_PASS:
            self._drive_encoder_distance(
                "avoid_pass", self.cfg.AVOID_PASS_CM, self.cfg.AVOID_V,
                State.AVOID_OFFSET_BACK,
            )
        elif self.state == State.AVOID_OFFSET_BACK:
            self._avoid_arc("offset_back", -self._pass_sign(), State.AVOID_BACK_ALIGN)
        elif self.state == State.AVOID_BACK_ALIGN:
            self._avoid_arc("back_align", self._pass_sign(), State.CLEAR_TARGET)
        elif self.state == State.CLEAR_TARGET:
            self._clear_target()
        elif self.state == State.GREEN_ENTRY_TURN:
            self._green_turn(
                "green_entry", self._green_entry_sign(),
                self.cfg.GREEN_ENTRY_TURN_CM, State.GREEN_LEG,
            )
        elif self.state == State.GREEN_LEG:
            self._drive_encoder_distance(
                f"green_leg_{self.orbit_segment + 1}",
                self.cfg.GREEN_ORBIT_LEG_CM,
                self.cfg.ORBIT_V,
                State.GREEN_CORNER_TURN,
            )
        elif self.state == State.GREEN_CORNER_TURN:
            self._green_corner()
        elif self.state == State.GREEN_RESTORE_HEADING:
            self._green_turn(
                "green_restore", -self._green_entry_sign(),
                self.cfg.GREEN_ENTRY_TURN_CM, State.AVOID_OFFSET_OUT,
            )

        self._log(blobs, dist_cm)

    def _target_color(self) -> Optional[str]:
        if self.target_index >= len(self.course):
            return None
        return self.course[self.target_index]

    def _drive_to_target(self, color: Optional[str], target: Optional[Blob]) -> None:
        if color is None:
            self.set_state(State.FINISH)
            return
        self._update_visibility(target)
        if self.visible_frames >= self.cfg.TARGET_CONFIRM_FRAMES:
            self.set_state(State.APPROACH_TARGET)
            return

        self.action = f"drive_to_{color} seen={self.visible_frames}"
        self.motor.drive(self.cfg.CRUISE_V, 0.0)

    def _approach_target(
        self,
        color: Optional[str],
        target: Optional[Blob],
        dist_cm: float,
    ) -> None:
        if color is None:
            self.set_state(State.FINISH)
            return
        self._update_visibility(target)
        if target is None:
            self.approach_pid.reset()
            self.completion_frames = 0
            if self.lost_frames >= self.cfg.TARGET_LOST_FRAMES:
                self.set_state(State.DRIVE_TO_TARGET)
                return
            self.action = f"approach_{color}_lost lost={self.lost_frames}"
            self.motor.drive(self.cfg.SLOW_V, 0.0)
            return

        center_err = (self.cfg.FRAME_W / 2.0 - target.cx) / (self.cfg.FRAME_W / 2.0)
        centered = abs(target.cx - self.cfg.FRAME_W / 2.0) <= self.cfg.GREEN_CENTER_TOL_PX
        area_ratio = target.size_score / (self.cfg.FRAME_W * self.cfg.FRAME_H)
        distance_valid = 2.0 <= dist_cm <= 400.0
        if color == "green":
            close_by_size = area_ratio >= self.cfg.GREEN_APPROACH_AREA_RATIO
            range_error = dist_cm - self.cfg.GREEN_ORBIT_RADIUS_CM
            ready = centered and (
                abs(range_error) <= self.cfg.GREEN_ORBIT_RADIUS_TOL_CM
                if distance_valid
                else close_by_size
            )
        else:
            close_by_size = area_ratio >= self.cfg.PASS_NEAR_AREA_RATIO
            close_by_dist = distance_valid and dist_cm <= self.cfg.PASS_NEAR_CM
            range_error = 0.0
            ready = centered and (close_by_size or close_by_dist)

        self.completion_frames = self.completion_frames + 1 if ready else 0
        if self.completion_frames >= self.cfg.TARGET_CONFIRM_FRAMES:
            if color == "green":
                self.orbit_segment = 0
                self.orbit_total_t0 = time.time()
                self.set_state(State.GREEN_ENTRY_TURN)
            else:
                self.set_state(State.AVOID_OFFSET_OUT)
            return

        w = self._clip_unit(self.approach_pid.update(center_err))
        if (
            color == "green"
            and distance_valid
            and centered
            and range_error < -self.cfg.GREEN_ORBIT_RADIUS_TOL_CM
        ):
            v = -self.cfg.GREEN_RANGE_V
            w = 0.0
        else:
            v = self._speed_by_distance(dist_cm)
        self.action = (
            f"approach_{color} cx={target.cx:.0f} centered={centered} "
            f"dist={dist_cm:.1f} ready={self.completion_frames} w={w:.2f}"
        )
        self.motor.drive(v, w)

    def _avoid_arc(self, label: str, turn_sign: float, next_state: State) -> None:
        if not self.encoder.available:
            self._recover(f"{label}_encoder_unavailable")
            return

        progress = self.encoder.progress_cm()
        if progress >= self.cfg.AVOID_ARC_CM:
            self.set_state(next_state)
            return

        self.action = (
            f"{label} progress={progress:.1f}/{self.cfg.AVOID_ARC_CM:.1f}cm"
        )
        self.motor.drive(self.cfg.AVOID_V, turn_sign * self.cfg.AVOID_TURN_W)

    def _drive_encoder_distance(
        self,
        label: str,
        required_cm: float,
        speed: float,
        next_state: State,
    ) -> None:
        if not self.encoder.available:
            self._recover(f"{label}_encoder_unavailable")
            return
        progress = self.encoder.progress_cm()
        if progress >= required_cm:
            self.set_state(next_state)
            return
        self.action = f"{label} progress={progress:.1f}/{required_cm:.1f}cm"
        self.motor.drive(speed, 0.0)

    def _clear_target(self) -> None:
        color = self._target_color()
        if color is None:
            self.set_state(State.FINISH)
            return
        if not self.encoder.available:
            self._recover(f"clear_{color}_encoder_unavailable")
            return

        progress = self.encoder.progress_cm()
        if progress >= self.cfg.AVOID_CLEAR_CM:
            self.target_index += 1
            next_state = (
                State.FINISH
                if self.target_index >= len(self.course)
                else State.DRIVE_TO_TARGET
            )
            self.set_state(next_state)
            return

        self.action = (
            f"clear_{color} progress={progress:.1f}/{self.cfg.AVOID_CLEAR_CM:.1f}cm"
        )
        self.motor.drive(self.cfg.CLEAR_V, 0.0)

    def _pass_sign(self) -> float:
        color = self._target_color()
        pass_side = {
            "red": self.cfg.RED_PASS_SIDE,
            "green": self.cfg.GREEN_PASS_SIDE,
            "yellow": self.cfg.YELLOW_PASS_SIDE,
        }.get(color, "left")
        return 1.0 if pass_side == "left" else -1.0

    def _green_entry_sign(self) -> float:
        # From the near side of the cube, clockwise travel starts to the left.
        return 1.0 if self.cfg.ORBIT_DIRECTION == "clockwise" else -1.0

    def _green_turn(
        self,
        label: str,
        turn_sign: float,
        required_cm: float,
        next_state: State,
    ) -> None:
        if not self.encoder.available:
            self._recover(f"{label}_encoder_unavailable")
            return

        progress = self._turn_progress_cm()
        if progress >= required_cm:
            self.set_state(next_state)
            return

        self.action = f"{label} progress={progress:.1f}/{required_cm:.1f}cm"
        self.motor.drive(0.0, turn_sign * self.cfg.GREEN_TURN_W)

    def _green_corner(self) -> None:
        if not self.encoder.available:
            self._recover("green_corner_encoder_unavailable")
            return

        progress = self._turn_progress_cm()
        if progress >= self.cfg.GREEN_CORNER_TURN_CM:
            self.orbit_segment += 1
            next_state = (
                State.GREEN_RESTORE_HEADING
                if self.orbit_segment >= self.cfg.GREEN_ORBIT_SEGMENTS
                else State.GREEN_LEG
            )
            self.set_state(next_state)
            return

        corner_sign = -self._green_entry_sign()
        self.action = (
            f"green_corner_{self.orbit_segment + 1} "
            f"progress={progress:.1f}/{self.cfg.GREEN_CORNER_TURN_CM:.1f}cm"
        )
        self.motor.drive(0.0, corner_sign * self.cfg.GREEN_TURN_W)

    def _turn_progress_cm(self) -> float:
        left_cm, right_cm = self.encoder.wheel_progress_cm()
        return min(left_cm, right_cm)

    def _speed_by_distance(self, dist_cm: float) -> float:
        if 0 < dist_cm < self.cfg.SLOW_CM:
            return self.cfg.SLOW_V
        return self.cfg.CRUISE_V

    def set_state(self, state: State) -> None:
        print(f"{time.time():.2f}: {self.state.name} -> {state.name}")
        self.motor.stop()
        self.state = state
        self.state_t0 = time.time()
        self.visible_frames = 0
        self.lost_frames = 0
        self.completion_frames = 0
        self.approach_pid.reset()
        if state in {
            State.AVOID_OFFSET_OUT,
            State.AVOID_OUT_ALIGN,
            State.AVOID_PASS,
            State.AVOID_OFFSET_BACK,
            State.AVOID_BACK_ALIGN,
            State.CLEAR_TARGET,
            State.GREEN_ENTRY_TURN,
            State.GREEN_LEG,
            State.GREEN_CORNER_TURN,
            State.GREEN_RESTORE_HEADING,
        }:
            self.encoder.reset()

    def elapsed(self) -> float:
        return time.time() - self.state_t0

    def _update_visibility(self, blob: Optional[Blob]) -> None:
        if blob is None:
            self.visible_frames = 0
            self.lost_frames += 1
        else:
            self.visible_frames += 1
            self.lost_frames = 0

    def _timed_out(self, limit_s: float) -> bool:
        return self.elapsed() >= limit_s

    def _state_timeout_limit(self) -> Optional[float]:
        if self.state == State.DRIVE_TO_TARGET:
            return self.cfg.FIND_TARGET_TIMEOUT_S
        if self.state == State.APPROACH_TARGET:
            return self.cfg.PASS_CUBE_TIMEOUT_S
        if self.state == State.CLEAR_TARGET:
            return self.cfg.CLEAR_CUBE_TIMEOUT_S
        if self.state in {
            State.AVOID_OFFSET_OUT,
            State.AVOID_OUT_ALIGN,
            State.AVOID_PASS,
            State.AVOID_OFFSET_BACK,
            State.AVOID_BACK_ALIGN,
            State.GREEN_ENTRY_TURN,
            State.GREEN_LEG,
            State.GREEN_CORNER_TURN,
            State.GREEN_RESTORE_HEADING,
        }:
            return self.cfg.MANEUVER_TIMEOUT_S
        return None

    def _in_green_orbit(self) -> bool:
        return self.state in {
            State.GREEN_ENTRY_TURN,
            State.GREEN_LEG,
            State.GREEN_CORNER_TURN,
            State.GREEN_RESTORE_HEADING,
        }

    def _recover(self, reason: str) -> None:
        self.failure_reason = reason
        self.action = f"recovery_stop reason={reason}"
        self.set_state(State.RECOVERY)

    @staticmethod
    def _clip_unit(x: float) -> float:
        return max(-1.0, min(1.0, x))

    def _log(self, blobs: Dict[str, Optional[Blob]], dist_cm: float) -> None:
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
            f"state={self.state.name} dist={dist_cm:.1f}cm "
            f"enc={self.encoder.progress_cm():.1f}cm seen=[{seen_text}] action={self.action}"
        )

    def shutdown(self) -> None:
        self.motor.close()
        self.camera.close()
        self.ultra.close()
        self.encoder.close()
        GPIO.cleanup()
