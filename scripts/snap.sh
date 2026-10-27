#!/bin/bash
# Saves one annotated camera frame to C:\Users\<you>\annotated.png (path is set inside ros/snap.py).
source "$(dirname "$0")/_common.sh"
python3 "$REPO/ros/snap.py"
