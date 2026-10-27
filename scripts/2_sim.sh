#!/bin/bash
# Terminal 2: PX4 + Gazebo with the AeroScout.
# When "Ready for takeoff!" appears, type at the pxh> prompt:  param set COM_RC_IN_MODE 4
source "$(dirname "$0")/_common.sh"
cd "$HOME/PX4-Autopilot" && make px4_sitl gz_aeroscout_x450
