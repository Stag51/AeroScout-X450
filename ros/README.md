# ROS 2 nodes

These files are copied from the working WSL install by `scripts/collect_from_wsl.sh`.

| File | Purpose |
|---|---|
| `offboard_takeoff.py` | Arm, climb, hold, land. First flight test from ROS 2 |
| `yolo_detector.py` | Camera image to YOLOv8n. Publishes `/camera/image_annotated` and `/target/bbox` |
| `person_tracker.py` | Take off, then follow the person (turn, forward/back, hold altitude, search when lost) |
| `move_target.py` | Walks the simulated person back and forth |
| `snap.py` | Saves one annotated frame as a PNG |
| `view.py` | Live window of the annotated camera image |

`/target/bbox` is a `Float32MultiArray`: `[found, cx, cy, w, h, conf]`, all normalised to the image.
