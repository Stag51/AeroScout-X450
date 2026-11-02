#!/bin/bash
# Optional: record the annotated camera view with live distance overlay to ~/aero_videos/distance_<time>.mp4.
source "$(dirname "$0")/_common.sh"
mkdir -p "$HOME/aero_videos"
cd "$REPO/ros" && python3 record_distance.py
