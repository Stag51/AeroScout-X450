# Running the demo

Use one WSL terminal per step. Run them from the repository folder (the scripts find their own paths).
Order matters.

| # | Terminal | Command | What to wait for |
|---|---|---|---|
| 1 | Agent | `bash scripts/1_agent.sh` | Stays open, waiting |
| 2 | Simulator | `bash scripts/2_sim.sh` | Gazebo opens, then `Ready for takeoff!`. At the `pxh>` prompt type `param set COM_RC_IN_MODE 4` |
| 3 | World | `bash scripts/3_world.sh` | Prints `data: true` (person spawned), then keeps running as the camera bridge |
| 4 | Detector | `bash scripts/4_detector.sh` | Prints `person: centre(...)` lines |
| 5 | Tracker | `bash scripts/5_tracker.sh` | Takes off to 1.5 m and locks on |
| 6 | Walker | `bash scripts/6_walker.sh` | Start after step 5 has locked on. The person walks back and forth |

Optional: `bash scripts/7_viewer.sh` opens a window with the camera image and detection box.
`bash scripts/snap.sh` saves one annotated frame to `C:\Users\<you>\annotated.png`.

## Stopping

* Tracker: Ctrl+C. The drone lands by itself.
* Everything else: Ctrl+C in each terminal, then `bash scripts/kill_all.sh`.

## Rules that save time

* **Always restart the agent (step 1) when you restart the simulator (step 2).** Starting PX4 while the agent still
  holds the old session crashed PX4 with a segmentation fault.
* Always type `param set COM_RC_IN_MODE 4` at `pxh>` after a fresh start. Without a radio, PX4 otherwise refuses to arm.
* The simulator terminal is the only place where `commander` and `param` commands work.

## First flight test (no AI)

```bash
python3 ros/offboard_takeoff.py
```

It arms, climbs, holds, and lands. The demo drone reached the target height smoothly and landed at about 0.7 m/s.

## Tuning

The constants are at the top of `ros/person_tracker.py`:

| Constant | Meaning |
|---|---|
| `ALT` | Flight height in metres |
| `H_TARGET` | Wanted person height as a fraction of the image (bigger = closer) |
| `K_YAW` | Turn rate per image-width of error |
| `K_FWD` | Forward speed per unit of size error |
| `TRACK_SECONDS` | Time before the tracker lands by itself |
