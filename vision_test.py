#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Display color detections without initializing motors or other sensors."""

from __future__ import annotations

import time

from config import Config
from perception import ColorCamera


def main() -> None:
    cfg = Config(DEBUG_VIEW=True, DEBUG_LOG=False)
    camera = ColorCamera(cfg)

    print("Show red, green, and yellow cubes to the camera. Press Ctrl+C to stop.")
    try:
        while True:
            blobs = camera.detect()
            detections = []
            for color in ("red", "green", "yellow"):
                blob = blobs[color]
                if blob is not None:
                    detections.append(
                        f"{color}:cx={blob.cx:.0f},cy={blob.cy:.0f},area={blob.area:.0f}"
                    )
            print("seen=[" + "; ".join(detections) + "]")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nVision test stopped.")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
