# Measuring distance estimation

How the person's distance is estimated from one camera image, and how to repeat the measurement.
Results are in [analysis/RESULTS.md](../analysis/RESULTS.md).

## Method

1. YOLO gives the person's box. The bottom-centre of the box is where the feet meet the ground.
2. That pixel defines a ray from the camera (focal length from the 1.74 rad horizontal field of view, 1280x960).
3. The ray is rotated by the drone's roll and pitch (from PX4 `vehicle_attitude`).
4. The ray is intersected with the ground plane at z = 0, starting at the camera height
   (PX4 altitude plus the camera offset in the body frame).
5. The intersection gives forward, lateral and straight-line distance.

Gazebo's pose of the drone and the person is the ground truth. The code is in
`ros/distance_estimator.py` (pure functions) and `ros/distance_estimator_node.py` (ROS node,
publishes `/target/estimate`).

Calibration: two constants (`FY_SCALE`, `V_OFFSET`) were fitted on flight 1 only. Raw geometry
reads about 0.23 m short and the cause is not known, so the constants should be seen as an empirical correction.

## Run it

Start the normal demo first (terminals 1-6 in [RUN_DEMO.md](RUN_DEMO.md)), then, one terminal each:

```bash
bash scripts/8_pose_bridge.sh     # ground truth into ROS 2
bash scripts/9_estimator.sh       # /target/estimate
bash scripts/10_logger.sh         # start after take-off, Ctrl+C at the end
bash scripts/11_recorder.sh       # optional video with overlay
```

Logs land in `~/aero_logs/`, videos in `~/aero_videos/`. Copy a log into `analysis/`, then:

```bash
pip install numpy matplotlib
python3 analysis/make_plots.py analysis/flight3.csv analysis/plots   # table + two plots
python3 analysis/analyze_distance.py analysis/flight3.csv            # detail incl. lag test
```

Convert a recording so it plays everywhere:

```bash
ffmpeg -i ~/aero_videos/distance_XXXX.mp4 -c:v libx264 -pix_fmt yuv420p -crf 23 -movflags +faststart out.mp4
```

## Things that went wrong once (so they do not again)

* The order of entries in `/world/default/dynamic_pose/info` changes between runs and the bridge leaves names empty.
  The logger asks `gz topic -e -n 1` for the names at startup and picks the drone and person by name.
* Estimates in the first ~5 s after take-off are 2-3.5 m off (feet almost at the horizon). The analysis
  only uses frames above 1 m altitude. Start a demo video after take-off.
* Run `bash scripts/kill_all.sh` before restarting the simulator, or Gazebo says "already running".

## `/target/estimate`

`Float32MultiArray`: `[valid, range, ground, fwd, lat, cam_height, roll_deg, pitch_down_deg]`
(metres and degrees; `fwd`/`lat` are in the heading frame).
