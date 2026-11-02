#!/usr/bin/env python3
"""Ground-plane distance estimation from the bottom edge of a person's bounding box.

Pure functions, no ROS. Frames: body is FLU (x forward, y left, z up). Pitch is positive nose-DOWN.
"""
import math

W, H = 1280, 960                      # image size (PX4 mono_cam)
HFOV = 1.74                           # rad
FX = (W / 2) / math.tan(HFOV / 2)     # pixels; fy is the same (square pixels)
FY = FX


def rot_rpy(roll, pitch, yaw):
    """Body -> world rotation for Z(yaw) * Y(pitch) * X(roll), angles in radians."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def ground_distance(u_n, v_bottom_n, cam_height, roll=0.0, pitch=0.0, yaw=0.0):
    """Where does the ray through the box-bottom pixel hit the ground?

    u_n, v_bottom_n : box bottom-centre pixel, normalised 0..1 (u = cx, v = cy + h/2)
    cam_height      : camera height above the ground, metres
    roll, pitch, yaw: drone attitude in radians (pitch positive nose-down)
    Returns dict with range, ground, fwd, lat (heading frame, left positive) or None if the ray
    does not hit the ground (box bottom above the horizon).
    """
    rb = (1.0, -(u_n - 0.5) * W / FX, -(v_bottom_n - 0.5) * H / FY)     # ray in body frame
    R = rot_rpy(roll, pitch, yaw)
    rw = [sum(R[i][j] * rb[j] for j in range(3)) for i in range(3)]
    if rw[2] >= -1e-6:
        return None
    t = cam_height / -rw[2]
    dx, dy = t * rw[0], t * rw[1]
    return {
        'range': t * math.sqrt(sum(c * c for c in rb)),
        'ground': math.hypot(dx, dy),
        'fwd': dx * math.cos(yaw) + dy * math.sin(yaw),
        'lat': -dx * math.sin(yaw) + dy * math.cos(yaw),
    }


def height_proxy(h_n, k):
    """The old method: distance from box height only. Distance = k / h (k fitted or guessed)."""
    return k / h_n if h_n > 1e-3 else float('nan')


# ---- helpers for using PX4's own height and tilt (instead of Gazebo's truth) --------------------
BASE_Z = 0.087                        # drone origin height above ground when sitting on the ground
CAM_OFFSET = (0.064, 0.0, 0.0225)     # camera_link in the drone body frame, metres


def px4_roll_pitch_down(q):
    """PX4 attitude quaternion (w, x, y, z; FRD -> NED) -> (roll, pitch_down) in radians.
    PX4 pitch is positive nose-UP, the estimator wants nose-DOWN, so the sign is flipped."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch_ned = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, -pitch_ned


def camera_height(altitude, roll, pitch_down, base_z=BASE_Z):
    """Camera height above the ground from PX4 altitude (-z_NED, zero where the drone started)."""
    R = rot_rpy(roll, pitch_down, 0.0)
    return base_z + altitude + sum(R[2][j] * CAM_OFFSET[j] for j in range(3))