#!/bin/bash
# Terminal 5: take off and follow the person. Ctrl+C lands the drone.
source "$(dirname "$0")/_common.sh"
python3 "$REPO/ros/person_tracker.py"
