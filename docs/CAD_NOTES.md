# CAD notes

## Axes

SolidWorks assembly: origin at the centre of the underside of the bottom plate. X forward, Y up, Z right.
Gazebo and ROS: x forward, y left, z up.

| Gazebo | SolidWorks |
|---|---|
| x | X |
| y | -Z |
| z | Y |

Meshes are loaded with a pose roll of 1.5707963 rad (90 degrees about X) and scale 0.001.

## Numbers used (from SolidWorks Mass Properties of the full assembly)

| Quantity | Value |
|---|---|
| Mass | 1361.15 g (the BOM adds up to 1361.25 g) |
| Centre of mass (SolidWorks axes) | X 0.45 mm, Y 8.07 mm, Z 0.00 mm |
| Inertia at the centre of mass (g mm^2) | Lxx 10207396, Lyy 18846281, Lzz 10833873 |
| Motor centres | radius 225 mm from the centre (+-159.1 mm in X and Z) |
| Landing legs | 135 mm from the centre, bottom at 87 mm below the plate |
| Camera | about 58.5 mm forward of the centre, 22.5 mm above the plate |

The Lyy value is about the vertical axis (SolidWorks Y), which is Gazebo z. That is why `izz` in the model is the largest.

## Parts

34 STL files, named `<part>-<instance>.stl`: 4 arms, bottom/top/clamp plates, battery tray and battery, 4 motors,
4 propellers, 4 standoffs, 4 posts, 4 landing legs, ESC, Pixhawk 6C Mini, Jetson Orin Nano kit, camera and camera mount.

Propellers are separate spinning links in the simulation, so the builder script removes their mass and inertia
from the body and adds them back as the four rotor links.

## Collision shapes

The body uses one simple box and the legs use thin cylinders. Using the detailed meshes for collision would be slow
and unstable in the physics engine.
