#!/bin/bash
# Measurement terminal B: distance estimator (publishes /target/estimate).
source "$(dirname "$0")/_common.sh"
cd "$REPO/ros" && python3 distance_estimator_node.py
