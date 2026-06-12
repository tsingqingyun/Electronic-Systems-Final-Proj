#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Function:
Program entry point.

Run this file on the Raspberry Pi to start the fixed-cube slalom task.
All parameters are stored in config.py; the task state machine is in
controller.py.
"""

from config import Config
from control import CubeSlalomController


def main() -> None:
    cfg = Config()
    robot = CubeSlalomController(cfg)
    robot.run()


if __name__ == "__main__":
    main()
