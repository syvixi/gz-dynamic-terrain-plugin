#!/usr/bin/env python3
"""Bounded, read-only Gazebo RGB-camera probe (before the video encoder).

Saves the first sampled frame and the frame with the most strongly blue pixels.
Blue detection is only a diagnostic heuristic, not proof of a rendering error.
Subscribing may activate the sensor's standard image publication temporarily.
Run with system Python providing gz.transport13, numpy and Pillow.
"""
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PillowImage
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic")
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=20)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error("--seconds must be in (0, 60]")
    args.output.mkdir(parents=True, exist_ok=False)
    lock = threading.Lock()
    state = {"sampled_frames": 0, "max_blue_pixels": -1}
    last_sample = 0.0
    stopped = False

    def receive(msg):
        nonlocal last_sample
        with lock:
            now = time.monotonic()
            if stopped or now - last_sample < 0.2:
                return
            last_sample = now
            enum = msg.DESCRIPTOR.fields_by_name["pixel_format_type"].enum_type
            fmt = enum.values_by_number[msg.pixel_format_type].name
            if fmt != "RGB_INT8" or msg.step < msg.width * 3:
                state["unsupported_format"] = fmt
                return
            if len(msg.data) != msg.height * msg.step:
                state["invalid_payload"] = True
                return
            pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step)[:, :msg.width * 3].reshape(
                    msg.height, msg.width, 3)
            blue = int(np.count_nonzero((pixels[:, :, 2] > 160) &
                       (pixels[:, :, 0] < 70) & (pixels[:, :, 1] < 70)))
            state["sampled_frames"] += 1
            if state["sampled_frames"] == 1:
                PillowImage.fromarray(pixels).save(args.output / "first.png")
            if blue > state["max_blue_pixels"]:
                state["max_blue_pixels"] = blue
                PillowImage.fromarray(pixels).save(args.output / "most_blue.png")

    node = Node()
    node.subscribe(Image, args.topic, receive)
    try:
        time.sleep(args.seconds)
    finally:
        with lock:
            stopped = True
        node.unsubscribe(args.topic)
    print(json.dumps(state), flush=True)


if __name__ == "__main__":
    main()
