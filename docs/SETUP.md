# One-time setup (Windows 11 + WSL2 + Ubuntu 22.04)

This is what was used for the demo. The laptop had an Intel i5-10300H, 16 GB RAM and a GTX 1650 (4 GB).
Use an NVIDIA GPU if you can: YOLO ran at about 19 ms per frame on it.

Exact versions are in [versions.txt](versions.txt) (created by `scripts/collect_from_wsl.sh`).

## 1. Windows side

1. Install the current NVIDIA driver on Windows (not inside Ubuntu).
2. In PowerShell as administrator: `wsl --install -d Ubuntu-22.04`, then restart and create your Linux user.
3. In Ubuntu run `nvidia-smi`. It should list your GPU.

## 2. ROS 2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 3. PX4 and Gazebo

```bash
cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
# reopen the terminal, then test the stock drone:
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

The demo was built on PX4 `main` when it reported version `1.18.0~beta1`. The exact commit is saved in
`versions.txt`. If a newer PX4 changes topic names, check out that commit:
`cd ~/PX4-Autopilot && git checkout <commit> && git submodule update --init --recursive`.

Gazebo should be Harmonic (`gz sim --version` printed 8.15.0).

## 4. Make Gazebo use the NVIDIA GPU

Without this, Gazebo used the Intel GPU and the camera ran at 3 Hz. With it, about 15 Hz.

```bash
echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc
```

## 5. ROS-Gazebo bridge

```bash
sudo apt install -y ros-humble-ros-gzharmonic
```

## 6. Micro XRCE-DDS Agent (the ROS 2 to PX4 link)

Version 2.4.2 needed a small patch to build, because its CMake file asks for two branch names that no longer exist:

```bash
cd ~ && git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
sed -i "s/2\.12\.x/v2.12.1/g; s/1\.1\.x/v1.1.1/g" CMakeLists.txt
mkdir build && cd build && cmake .. && make -j2 && sudo make install && sudo ldconfig /usr/local/lib/
```

## 7. ROS 2 workspace with PX4 messages

The message definitions must match your PX4 build exactly, so copy them from the PX4 tree
(using the generic `px4_msgs` release caused a "payload size" error):

```bash
mkdir -p ~/ws_aero/src && cd ~/ws_aero/src
git clone https://github.com/PX4/px4_msgs.git
rm -rf px4_msgs/msg px4_msgs/srv
cp -r ~/PX4-Autopilot/msg px4_msgs/msg
cp -r ~/PX4-Autopilot/srv px4_msgs/srv
cd ~/ws_aero && source /opt/ros/humble/setup.bash && colcon build --packages-select px4_msgs
echo "source ~/ws_aero/install/setup.bash" >> ~/.bashrc
```

Do not build `px4_ros_com` (it failed against this PX4 version and is not needed).
Repeat step 7 whenever you update PX4.

## 8. Python packages for the AI part

```bash
pip3 install ultralytics "numpy<2"
```

`numpy<2` is needed because ROS 2 Humble's `cv_bridge` was built against numpy 1.x.
The first run downloads `yolov8n.pt` automatically.

## 9. This repository and the AeroScout model

```bash
cd ~ && git clone <your repo url> AeroScout_X450 && cd AeroScout_X450
mkdir -p ~/aeroscout_meshes && cp cad_export/stl/*.stl ~/aeroscout_meshes/
python3 sim/build_aeroscout_model.py
cd ~/PX4-Autopilot && make px4_sitl gz_aeroscout_x450      # first build takes a few minutes
```

The builder script writes into your PX4 folder: a model in `Tools/simulation/gz/models/aeroscout_x450/`,
the airframe `ROMFS/px4fmu_common/init.d-posix/airframes/4090_gz_aeroscout_x450`, and one line in that
folder's `CMakeLists.txt`. If airframe number 4090 is already used, the script stops and tells you.

Close the simulator (Ctrl+C). You are ready for [RUN_DEMO.md](RUN_DEMO.md).
