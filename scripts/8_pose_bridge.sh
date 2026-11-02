#!/bin/bash
# Measurement terminal A: bridge Gazebo ground-truth poses into ROS 2 (stays running).
source "$(dirname "$0")/_common.sh"
ros2 run ros_gz_bridge parameter_bridge "/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
