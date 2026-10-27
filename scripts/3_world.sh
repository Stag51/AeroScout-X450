#!/bin/bash
# Terminal 3: spawn the person, then bridge the drone's camera into ROS 2 (stays running).
source "$(dirname "$0")/_common.sh"
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 30000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Walking person", name: "target_person", pose: {position: {x: 6, y: 0, z: 0}}'
T=/world/default/model/$MODEL/link/camera_link/sensor/camera/image
ros2 run ros_gz_bridge parameter_bridge "$T@sensor_msgs/msg/Image[gz.msgs.Image" --ros-args -r "$T:=/camera/image_raw"
