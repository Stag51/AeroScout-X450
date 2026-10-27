#!/bin/bash
# Terminal 6: make the person walk back and forth. Start after the drone has locked on.
source "$(dirname "$0")/_common.sh"
python3 "$REPO/ros/move_target.py"
