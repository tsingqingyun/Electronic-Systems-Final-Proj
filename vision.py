#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Camera-based color perception.

This file reads frames from the camera, segments red/green/yellow cubes
using HSV thresholds, and returns the largest visible blob for each color.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import cv2
import numpy as np

from config import Config


@dataclass
class Blob:
    color: str
    cx: float
    cy: float
    area: float
    w: float
    h: float
    visible: bool

    @property
    def size_score(self) -> float:
        return max(self.area, self.w * self.h)


class ColorCamera:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.cap = cv2.VideoCapture(cfg.CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.FRAME_H)

    def detect(self) -> Dict[str, Optional[Blob]]:
        ok, frame = self.cap.read()
        if not ok:
            return {"red": None, "green": None, "yellow": None}
        frame = cv2.resize(frame, (self.cfg.FRAME_W, self.cfg.FRAME_H))
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        found: Dict[str, Optional[Blob]] = {}
        for color, ranges in self.cfg.HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for low, high in ranges:
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)),
                )
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            found[color] = self._largest_blob(mask, color)

        if self.cfg.DEBUG_VIEW:
            self._draw_debug(frame, found)
        return found

    def close(self) -> None:
        self.cap.release()
        if self.cfg.DEBUG_VIEW:
            cv2.destroyAllWindows()

    def _largest_blob(self, mask: np.ndarray, color: str) -> Optional[Blob]:
        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[0] if len(contours_result) == 2 else contours_result[1]
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < self.cfg.MIN_BLOB_AREA:
            return None
        x, y, w, h = cv2.boundingRect(cnt)
        return Blob(color, x + w / 2.0, y + h / 2.0, area, w, h, True)

    def _draw_debug(self, frame: np.ndarray, blobs: Dict[str, Optional[Blob]]) -> None:
        for blob in blobs.values():
            if blob is None:
                continue
            p1 = (int(blob.cx - blob.w / 2), int(blob.cy - blob.h / 2))
            p2 = (int(blob.cx + blob.w / 2), int(blob.cy + blob.h / 2))
            cv2.rectangle(frame, p1, p2, (255, 255, 255), 2)
            cv2.putText(frame, blob.color, p1, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("cube-slalom-debug", frame)
        cv2.waitKey(1)
