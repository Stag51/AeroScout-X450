#!/usr/bin/env python3
"""Makes the simulated person walk back and forth so the drone has something to follow."""
import math
import subprocess
import time

X = 6.0        # distance ahead of the start point, metres
SPAN = 5.0     # walks between y = -SPAN and y = +SPAN
SPEED = 0.5    # m/s


def set_pose(x, y, yaw):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "target_person" position: {{x: {x:.3f} y: {y:.3f} z: 0}} '
           f'orientation: {{x: 0 y: 0 z: {qz:.4f} w: {qw:.4f}}}')
    subprocess.run(['gz', 'service', '-s', '/world/default/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '500', '--req', req],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


t0 = time.time()
loop_len = 4 * SPAN
print('Person is walking. Press Ctrl+C to stop.')
try:
    while True:
        d = (SPAN + (time.time() - t0) * SPEED) % loop_len   # distance along the loop
        if d < 2 * SPAN:
            y, yaw = -SPAN + d, math.pi / 2
        else:
            y, yaw = SPAN - (d - 2 * SPAN), -math.pi / 2
        set_pose(X, y, yaw)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
