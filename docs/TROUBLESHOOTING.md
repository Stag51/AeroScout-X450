# Troubleshooting (problems met while building this, and the fixes)

| Symptom | Cause | Fix |
|---|---|---|
| `Segmentation fault` right after `previous session found on the agent, deleting it` | PX4 restarted while the XRCE agent still held the old session | Stop the agent, kill PX4 and Gazebo, start the agent first, then PX4 |
| `PX4 server already running for instance 0` | Old PX4 still alive | `bash scripts/kill_all.sh` |
| `A link named rotor_0 has invalid inertia` and the drone does not appear | Rotor inertia broke the triangle rule (ixx + iyy must be >= izz) | Use the values in `build_aeroscout_model.py` (copied from the x500) |
| `Arming denied` / `no GCS` | No ground station or radio in simulation | `param set COM_RC_IN_MODE 4` at `pxh>`. The airframe already sets `NAV_DLL_ACT 0` |
| `commander: command not found` | Typed in a normal shell | Type PX4 commands only at the `pxh>` prompt |
| Topic `/fmu/out/vehicle_status` does not exist | PX4 topics are versioned | Use `/fmu/out/vehicle_status_v5`, `/fmu/out/vehicle_local_position_v1`, `/fmu/in/vehicle_command_v1` (check with `ros2 topic list` after PX4 updates) |
| `RTPS_READER_HISTORY` payload size error (96 vs 91 bytes) | `px4_msgs` does not match the PX4 build | Redo SETUP step 7 (copy `msg` and `srv` from the PX4 tree) |
| Micro XRCE-DDS Agent fails with `Failed to checkout tag: '2.12.x'` | Branch names removed upstream | The `sed` command in SETUP step 6 |
| `px4_ros_com` build fails | Header mismatch with this PX4 | Do not build it (not needed) |
| Camera at 3 Hz and Gazebo sluggish | Gazebo rendering on the Intel GPU | `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (SETUP step 4) |
| `rqt_image_view` window huge and unclickable under WSLg | WSLg scaling | Use `scripts/7_viewer.sh` or `scripts/snap.sh` instead |
| `cv_bridge` or numpy errors | numpy 2 installed | `pip3 install "numpy<2"` |
| Drone turns slowly and the person drifts off-centre | `K_YAW` too low | Raise `K_YAW` (2.8 worked) |
| Person walks but the drone never takes off | Tracker waiting for PX4 data (agent not connected) | Check terminal 1 shows a session, restart in the order above |
