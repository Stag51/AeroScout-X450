#!/bin/bash
# Run ONCE in WSL on the machine where everything works. Copies your live files into this repository
# and records the exact software versions, so the repository matches what actually ran.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$REPO/ros" "$REPO/cad_export/stl" "$REPO/x500_ref" "$REPO/docs"

cp -v ~/aero_nodes/*.py "$REPO/ros/" 2>/dev/null || echo "WARNING: ~/aero_nodes not found"
cp -v ~/check_stl.py "$REPO/sim/" 2>/dev/null
cp -v ~/aeroscout_meshes/*.stl "$REPO/cad_export/stl/" 2>/dev/null || echo "WARNING: ~/aeroscout_meshes not found"

P=~/PX4-Autopilot
cp -v $P/Tools/simulation/gz/models/x500_base/model.sdf "$REPO/x500_ref/x500_base_model.sdf" 2>/dev/null
cp -v $P/Tools/simulation/gz/models/x500/model.sdf "$REPO/x500_ref/x500_model.sdf" 2>/dev/null
cp -v $P/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 "$REPO/x500_ref/4001_gz_x500.txt" 2>/dev/null
# the generated AeroScout files, so you can inspect them without running the builder
mkdir -p "$REPO/sim/generated"
cp -v $P/ROMFS/px4fmu_common/init.d-posix/airframes/4090_gz_aeroscout_x450 "$REPO/sim/generated/" 2>/dev/null
cp -v $P/Tools/simulation/gz/models/aeroscout_x450/model.sdf "$REPO/sim/generated/aeroscout_x450_model.sdf" 2>/dev/null

{
  echo "Collected: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "--- OS";        lsb_release -d 2>/dev/null; uname -r
  echo "--- PX4 commit"; git -C $P rev-parse HEAD 2>/dev/null; git -C $P describe --tags --always 2>/dev/null
  echo "--- Gazebo";    gz sim --version 2>/dev/null | head -2
  echo "--- ROS";       echo "${ROS_DISTRO:-unknown}"; dpkg -l | grep -E "ros-humble-(ros-gzharmonic|cv-bridge|desktop) " | awk '{print $2, $3}'
  echo "--- Micro XRCE-DDS Agent"; git -C ~/Micro-XRCE-DDS-Agent describe --tags --always 2>/dev/null
  echo "--- px4_msgs"; git -C ~/ws_aero/src/px4_msgs log -1 --format=%H 2>/dev/null
  echo "--- Python"; python3 --version; pip3 list 2>/dev/null | grep -iE "^(ultralytics|torch|torchvision|numpy|opencv-python) "
  echo "--- GPU";     nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null
} > "$REPO/docs/versions.txt"
echo; echo "Done. Review $REPO/docs/versions.txt and the copied files, then commit."
