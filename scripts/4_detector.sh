#!/bin/bash
# Terminal 4: YOLOv8 person detector.
source "$(dirname "$0")/_common.sh"
python3 "$REPO/ros/yolo_detector.py"
