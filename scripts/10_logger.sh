#!/bin/bash
# Measurement terminal C: log estimate vs Gazebo truth to ~/aero_logs/run_<time>.csv at 10 Hz.
# Start it when the drone has taken off; stop with Ctrl+C when the flight is done.
source "$(dirname "$0")/_common.sh"
mkdir -p "$HOME/aero_logs"
cd "$REPO/ros" && python3 metrics_logger.py
