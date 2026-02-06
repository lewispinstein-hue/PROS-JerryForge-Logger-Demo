#!/usr/bin/env python3
import time

path = "/Users/David/Documents/VEXcode Robot/Solve For X Robotics Code/Logs/2026-02-05_16-13-26.log"

with open(path, "r", encoding="utf-8", errors="replace") as f:
    for line in f:
        print(line, end="", flush=True)
        time.sleep(0.12)
