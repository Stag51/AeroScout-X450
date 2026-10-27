#!/bin/bash
# Stops PX4, Gazebo and the agent. Restart the agent before the simulator afterwards.
pkill -f px4; pkill -f "gz sim"; pkill -f MicroXRCEAgent; pkill -f ros_gz_bridge
echo "stopped"
