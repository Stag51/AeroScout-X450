#!/usr/bin/env python3
"""Builds the AeroScout X450 Gazebo model and PX4 airframe from the SolidWorks STL files.

Reads : ~/aeroscout_meshes/*.stl           (exported from the CAD assembly, millimetres)
        PX4's own x500 model files         (sensors and motor plugins are copied from them)
Writes: PX4-Autopilot/Tools/simulation/gz/models/aeroscout_x450/   (model.sdf, model.config, meshes)
        PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4090_gz_aeroscout_x450
"""
import glob
import math
import os
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

HOME = os.environ.get('AERO_HOME', os.path.expanduser('~'))
MESH_DIR = f'{HOME}/aeroscout_meshes'
PX4 = f'{HOME}/PX4-Autopilot'
MODELS = f'{PX4}/Tools/simulation/gz/models'
AIRFRAMES = f'{PX4}/ROMFS/px4fmu_common/init.d-posix/airframes'
MODEL = 'aeroscout_x450'
OUT = f'{MODELS}/{MODEL}'
AIRFRAME_NO = '4090'
AIRFRAME = f'{AIRFRAME_NO}_gz_{MODEL}'

# ---- numbers from the CAD (SolidWorks Mass Properties, drone_assembly) --------------
TOTAL_MASS = 1.36115                      # kg
COM_SW_MM = (0.45, 8.07, 0.00)            # X forward, Y up, Z right (SolidWorks axes), mm
I_TOTAL_SW = (10207396.39, 18846280.86, 10833873.29)   # Lxx, Lyy(vertical), Lzz, g*mm^2 about the CoM
PROP_MASS = 0.013                         # kg per propeller (CAD value)
PROP_I = (2.8e-7, 6.99e-5, 7.02e-5)       # prop inertia about its own centre, kg*m^2 (thin bar)
HOVER_THRUST = 0.4                        # MPC_THR_HOVER for ~1.36 kg with the x500 motor constants
SPAWN_Z = 0.09                            # m, model height so the leg tips (-87 mm) touch the ground

COLORS = {
    'arm': (0.10, 0.10, 0.10), 'bottom_plate': (0.15, 0.15, 0.17), 'top_plate': (0.15, 0.15, 0.17),
    'clamp_plate': (0.15, 0.15, 0.17), 'battery_tray': (0.15, 0.15, 0.17),
    'battery_tattu_4s5200': (0.10, 0.20, 0.80), 'motor_2212_920kv': (0.60, 0.60, 0.65),
    'standoff': (0.85, 0.85, 0.85), 'post_35': (0.85, 0.85, 0.85),
    'esc_speedybee_55a': (0.10, 0.50, 0.10), 'pixhawk_6c_mini': (0.90, 0.90, 0.90),
    'jetson_orin_nano_devkit': (0.45, 0.70, 0.10), 'camera_imx219': (0.05, 0.05, 0.05),
    'camera_mount': (0.95, 0.50, 0.10), 'landing_leg': (0.10, 0.10, 0.10),
    'propeller_1045': (0.80, 0.80, 0.85),
}


def read_stl_bounds(path):
    """Min and max of X, Y, Z (millimetres, SolidWorks axes) of a binary or ASCII STL."""
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    pts = []
    if 84 + n * 50 == len(data):
        for i in range(n):
            off = 84 + i * 50 + 12
            for v in range(3):
                pts.append(struct.unpack('<3f', data[off + v * 12: off + v * 12 + 12]))
    else:
        for line in data.decode(errors='ignore').splitlines():
            p = line.split()
            if len(p) == 4 and p[0] == 'vertex':
                pts.append(tuple(float(x) for x in p[1:]))
    return [(min(c), max(c)) for c in zip(*pts)]


def to_gazebo(b):
    """SolidWorks bounds (mm) -> centre and size in Gazebo axes (m): x forward, y left, z up."""
    (x0, x1), (y0, y1), (z0, z1) = b
    centre = ((x0 + x1) / 2e3, -(z0 + z1) / 2e3, (y0 + y1) / 2e3)
    size = ((x1 - x0) / 1e3, (z1 - z0) / 1e3, (y1 - y0) / 1e3)
    front_x = x1 / 1e3
    return centre, size, front_x


def fmt(v):
    return ' '.join(f'{x:.6g}' for x in v)


def parallel(m, d):
    """Parallel-axis terms (Ixx, Iyy, Izz) for mass m offset by d."""
    return (m * (d[1] ** 2 + d[2] ** 2), m * (d[0] ** 2 + d[2] ** 2), m * (d[0] ** 2 + d[1] ** 2))


def sdf_elems(path, tag, parent_name=None):
    root = ET.parse(path).getroot()
    model = root.find('model')
    scope = model
    if parent_name:
        scope = [l for l in model.findall('link') if l.get('name') == parent_name][0]
    elems = scope.findall(tag)
    for e in elems:
        ET.indent(e, space='  ', level=2)
    return [ET.tostring(e, encoding='unicode').strip() for e in elems]


def main():
    if not os.path.isdir(MESH_DIR) or not os.path.isdir(MODELS):
        sys.exit(f'Missing folder: {MESH_DIR} or {MODELS}')

    # ---- read every part -----------------------------------------------------------
    parts = []
    for f in sorted(glob.glob(f'{MESH_DIR}/*.stl')):
        name = os.path.basename(f)
        kind = re.sub(r'-\d+\.stl$', '', name)
        centre, size, front_x = to_gazebo(read_stl_bounds(f))
        parts.append(dict(file=name, kind=kind, centre=centre, size=size, front_x=front_x))
    print(f'{len(parts)} parts read')
    props = [p for p in parts if p['kind'] == 'propeller_1045']
    motors = [p for p in parts if p['kind'] == 'motor_2212_920kv']
    legs = [p for p in parts if p['kind'] == 'landing_leg']
    cams = [p for p in parts if p['kind'] == 'camera_imx219']
    if len(props) != 4 or len(motors) != 4 or len(legs) != 4 or len(cams) != 1:
        sys.exit('Expected 4 propellers, 4 motors, 4 landing legs and 1 camera STL files.')

    # ---- rotors: same numbering as PX4's x500 (0 front-right, 1 back-left, 2 front-left, 3 back-right)
    def rotor_index(p):
        x, y, _ = p['centre']
        return {(True, False): 0, (False, True): 1, (True, True): 2, (False, False): 3}[(x > 0, y > 0)]
    rotors = {}
    for p in props:
        rotors[rotor_index(p)] = p
    if sorted(rotors) != [0, 1, 2, 3]:
        sys.exit('Could not assign the four propellers to the four corners.')
    offset = sum(abs(c) for p in motors for c in p['centre'][:2]) / 8       # metres, per axis
    print(f'motor offset from centre: {offset*1000:.1f} mm per axis')

    # ---- mass and inertia of the body without the propellers --------------------------
    com_tot = (COM_SW_MM[0] / 1e3, -COM_SW_MM[2] / 1e3, COM_SW_MM[1] / 1e3)
    i_tot = (I_TOTAL_SW[0] * 1e-9, I_TOTAL_SW[2] * 1e-9, I_TOTAL_SW[1] * 1e-9)   # ixx, iyy, izz in Gazebo axes
    m_base = TOTAL_MASS - 4 * PROP_MASS
    moment = [TOTAL_MASS * c for c in com_tot]
    for p in rotors.values():
        moment = [moment[i] - PROP_MASS * p['centre'][i] for i in range(3)]
    com_base = tuple(mm / m_base for mm in moment)
    i_base = list(i_tot)
    for p in rotors.values():
        d = tuple(p['centre'][i] - com_tot[i] for i in range(3))
        par = parallel(PROP_MASS, d)
        for k in range(3):
            i_base[k] -= PROP_I[k] + par[k]
    d_b = tuple(com_base[i] - com_tot[i] for i in range(3))
    par = parallel(m_base, d_b)
    for k in range(3):
        i_base[k] -= par[k]
    if min(i_base) <= 0:
        sys.exit(f'Inertia check failed: {i_base}')
    print(f'base mass {m_base:.4f} kg, CoM {fmt(com_base)}, inertia {fmt(i_base)}')

    # ---- model.sdf ------------------------------------------------------------------
    mesh_pose = '0 0 0 1.5707963 0 0'          # SolidWorks Y-up -> Gazebo Z-up
    def visual(p, pose=mesh_pose, name=None):
        c = COLORS.get(p['kind'], (0.5, 0.5, 0.5))
        col = f'{c[0]} {c[1]} {c[2]} 1'
        return f'''      <visual name="{name or p['file'][:-4]}">
        <pose>{pose}</pose>
        <geometry><mesh><scale>0.001 0.001 0.001</scale><uri>model://{MODEL}/meshes/{p['file']}</uri></mesh></geometry>
        <material><ambient>{col}</ambient><diffuse>{col}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>
'''
    surface = '''        <surface>
          <contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact>
          <friction><ode /></friction>
        </surface>
'''
    base_visuals = ''.join(visual(p) for p in parts if p['kind'] != 'propeller_1045')
    collisions = f'''      <collision name="body_collision">
        <pose>0 0 0.015 0 0 0</pose>
        <geometry><box><size>0.15 0.11 0.10</size></box></geometry>
{surface}      </collision>
'''
    for i, p in enumerate(legs):
        cx, cy, cz = p['centre']
        collisions += f'''      <collision name="leg_collision_{i}">
        <pose>{fmt((cx, cy, cz))} 0 0 0</pose>
        <geometry><cylinder><radius>0.005</radius><length>{p['size'][2]:.4f}</length></cylinder></geometry>
{surface}      </collision>
'''
    sensors = '\n'.join('      ' + s.replace('\n', '\n      ')
                        for s in sdf_elems(f'{MODELS}/x500_base/model.sdf', 'sensor', 'base_link'))
    plugins = '\n'.join('    ' + s.replace('\n', '\n    ')
                        for s in sdf_elems(f'{MODELS}/x500/model.sdf', 'plugin'))

    rotor_xml = ''
    for i in range(4):
        p = rotors[i]
        cx, cy, cz = p['centre']
        rotor_xml += f'''    <link name="rotor_{i}">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay />
      <pose>{fmt((cx, cy, cz))} 0 0 0</pose>
      <inertial>
        <mass>{PROP_MASS}</mass>
        <inertia><ixx>{PROP_I[0]}</ixx><iyy>{PROP_I[1]}</iyy><izz>{PROP_I[2]}</izz></inertia>
      </inertial>
{visual(p, pose=f'{fmt((-cx, -cy, -cz))} 1.5707963 0 0', name=f'rotor_{i}_visual')}    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
      </axis>
    </joint>
'''
    cam = cams[0]
    cam_pose = fmt((cam['front_x'] + 0.001, cam['centre'][1], cam['centre'][2])) + ' 0 0 0'

    sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<!-- AeroScout X450: generated by build_aeroscout_model.py from the SolidWorks CAD -->
<sdf version='1.9'>
  <model name='{MODEL}'>
    <pose>0 0 {SPAWN_Z} 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <pose>{fmt(com_base)} 0 0 0</pose>
        <mass>{m_base:.5f}</mass>
        <inertia>
          <ixx>{i_base[0]:.6e}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{i_base[1]:.6e}</iyy><iyz>0</iyz>
          <izz>{i_base[2]:.6e}</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <velocity_decay />
{base_visuals}{collisions}{sensors}
    </link>
{rotor_xml}    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>{cam_pose}</pose>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">{cam_pose}</pose>
    </joint>
{plugins}
  </model>
</sdf>
'''
    ET.fromstring(sdf.split('?>', 1)[1])        # must be valid XML
    os.makedirs(f'{OUT}/meshes', exist_ok=True)
    for p in parts:
        shutil.copy(f'{MESH_DIR}/{p["file"]}', f'{OUT}/meshes/{p["file"]}')
    open(f'{OUT}/model.sdf', 'w').write(sdf)
    open(f'{OUT}/model.config', 'w').write(f'''<?xml version="1.0"?>
<model>
  <name>AeroScout X450</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>AeroScout X450 quadcopter, built from the SolidWorks CAD.</description>
</model>
''')
    print(f'model written to {OUT}')

    # ---- PX4 airframe, based on 4001_gz_x500 --------------------------------------------
    if glob.glob(f'{AIRFRAMES}/{AIRFRAME_NO}_*') and not os.path.exists(f'{AIRFRAMES}/{AIRFRAME}'):
        sys.exit(f'Airframe number {AIRFRAME_NO} is already used. Tell me, and we pick another.')
    af = open(f'{AIRFRAMES}/4001_gz_x500').read()
    af = af.replace('@name Gazebo x500', '@name AeroScout X450')
    af = af.replace('PX4_SIM_MODEL:=x500', f'PX4_SIM_MODEL:={MODEL}')
    af = af.replace('0.174', f'{offset:.4f}')
    af = re.sub(r'(MPC_THR_HOVER) [0-9.]+', rf'\1 {HOVER_THRUST}', af)
    af = re.sub(r'(NAV_DLL_ACT) \d+', r'\1 0', af)
    open(f'{AIRFRAMES}/{AIRFRAME}', 'w').write(af)
    print(f'airframe written: {AIRFRAME}')

    cm = f'{AIRFRAMES}/CMakeLists.txt'
    lines = open(cm).read().split('\n')
    if any(AIRFRAME in l for l in lines):
        print('airframe already listed in CMakeLists.txt')
    else:
        idx = [i for i, l in enumerate(lines) if '4001_gz_x500' in l]
        if not idx:
            print('WARNING: 4001_gz_x500 not found in airframes/CMakeLists.txt - tell me.')
        else:
            indent = re.match(r'\s*', lines[idx[0]]).group(0)
            lines.insert(idx[0] + 1, f'{indent}{AIRFRAME}')
            open(cm, 'w').write('\n'.join(lines))
            print('airframe added to CMakeLists.txt')
    print('DONE')


if __name__ == '__main__':
    main()