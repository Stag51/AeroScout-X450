#!/bin/bash
# Optional: window showing the camera image with the detection box.
source "$(dirname "$0")/_common.sh"
python3 "$REPO/ros/view.py"
