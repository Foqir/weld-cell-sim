"""Cell layout, workpiece and weld-seam geometry (pure numpy, no ROS).

Used by the launch file (Gazebo world), by the weld program (MoveIt scene, torch poses)
and by offline checks, so the cell is described in exactly one place (config/cell.yaml).
"""
import math

import numpy as np
import yaml


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------- transforms

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_tf(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def quat_from_matrix(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def matrix_from_quat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_between_z(v):
    """Quaternion rotating +Z onto vector v (for cylinders / markers)."""
    v = np.asarray(v, float)
    v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, v))
    if c < -0.999999:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = np.cross(z, v)
    q = np.array([axis[0], axis[1], axis[2], 1.0 + c])
    return q / np.linalg.norm(q)


def part_tf(cell, offset=(0.0, 0.0, 0.0)):
    """Part frame in the world. offset = (dx, dy, dyaw_deg) applied on top of the nominal pose."""
    p = cell['part']
    xyz = np.array(p['xyz'], float) + np.array([offset[0], offset[1], 0.0])
    yaw = math.radians(p.get('yaw', 0.0) + offset[2])
    return make_tf(rot_z(yaw), xyz)


# ---------------------------------------------------------------- seams

def _seam_curve(seam, part, step):
    """Seam in the PART frame: points, travel tangents, plate normals n1, wall normals n2."""
    n1 = np.array([0.0, 0.0, 1.0])
    if seam['type'] == 'fillet_line':
        rib = part['rib']
        if seam['feature'] == 'rib_left':
            y, n2 = rib['y'] - rib['thickness'] / 2, np.array([0.0, -1.0, 0.0])
        elif seam['feature'] == 'rib_right':
            y, n2 = rib['y'] + rib['thickness'] / 2, np.array([0.0, 1.0, 0.0])
        else:
            raise ValueError(f"unknown line feature {seam['feature']}")
        half = rib['length'] / 2 - seam.get('trim', 0.0)
        x0, x1 = (half, -half) if seam.get('reverse') else (-half, half)
        n = max(2, int(round(abs(x1 - x0) / step)) + 1)
        xs = np.linspace(x0, x1, n)
        P = np.stack([xs, np.full(n, y), np.zeros(n)], axis=1)
        t = np.array([np.sign(x1 - x0), 0.0, 0.0])
        T = np.tile(t, (n, 1))
        N2 = np.tile(n2, (n, 1))
    elif seam['type'] == 'fillet_circle':
        tube = part['tube']
        c = np.array([tube['x'], tube['y']])
        r = tube['radius']
        a0 = math.radians(seam.get('start_angle', 0.0))
        sweep = math.radians(seam.get('sweep', 360.0))
        n = max(2, int(round(abs(sweep) * r / step)) + 1)
        a = a0 + np.linspace(0.0, sweep, n)
        radial = np.stack([np.cos(a), np.sin(a), np.zeros(n)], axis=1)
        P = np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.zeros(n)])
        sgn = 1.0 if sweep >= 0 else -1.0
        T = sgn * np.stack([-np.sin(a), np.cos(a), np.zeros(n)], axis=1)
        N2 = radial
    else:
        raise ValueError(f"unknown seam type {seam['type']}")
    N1 = np.tile(n1, (len(P), 1))
    return P, T, N1, N2


def torch_frames(P, T, N1, N2, work_angle_deg, travel_angle_deg, flange_dir, bend_sign=1.0):
    """TCP frames along the seam. +Z of the TCP = wire direction (into the joint).

    work angle: angle between the torch and the base plate (45 = bisector of a fillet).
    travel angle: tilt along the travel direction, > 0 = push technique.
    The roll around the wire is chosen so that the flange axis is as close as possible to
    flange_dir (for a bent torch the flange then stays in a comfortable, constant attitude).
    """
    a = math.radians(work_angle_deg)
    b = math.radians(travel_angle_deg)
    frames = []
    d = np.asarray(flange_dir, float)
    for p, t, n1, n2 in zip(P, T, N1, N2):
        z = -(math.sin(a) * n1 + math.cos(a) * n2)
        z = math.cos(b) * z + math.sin(b) * t
        z /= np.linalg.norm(z)
        # flange z in TCP coordinates is (-sin(bend), 0, cos(bend)) -> x = -proj_perp(d) / |.|
        perp = d - np.dot(d, z) * z
        if np.linalg.norm(perp) < 1e-6:
            perp = t - np.dot(t, z) * z
        x = -bend_sign * perp / np.linalg.norm(perp)
        y = np.cross(z, x)
        frames.append(np.column_stack([x, y, z]))
    return np.array(frames)


def seam_poses(seam, cell, robot_cfg, step=0.001, offset=(0.0, 0.0, 0.0)):
    """World-frame seam: positions (N,3), TCP rotations (N,3,3), arc length (N,)."""
    Tp = part_tf(cell, offset)
    P, T, N1, N2 = _seam_curve(seam, cell['part'], step)
    Rw = Tp[:3, :3]
    Pw = (Rw @ P.T).T + Tp[:3, 3]
    Tw, N1w, N2w = (Rw @ T.T).T, (Rw @ N1.T).T, (Rw @ N2.T).T
    R = torch_frames(Pw, Tw, N1w, N2w, seam.get('work_angle', 45.0), seam.get('travel_angle', 0.0),
                     robot_cfg.get('preferred_flange_dir', [0, 0, -1]))
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Pw, axis=0), axis=1))])
    return Pw, R, s


def point_to_polyline(p, poly):
    """Shortest distance from point p to a polyline (N,3)."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0, 1)
    proj = a + ab * t[:, None]
    return float(np.min(np.linalg.norm(proj - p, axis=1)))


# ---------------------------------------------------------------- scene objects

def cell_objects(cell, offset=(0.0, 0.0, 0.0), include_fence=True):
    """Static cell objects: dicts with id, shape (box|cylinder), dims, xyz, quat, color, group."""
    objs = []
    q0 = [0.0, 0.0, 0.0, 1.0]

    tb = cell['table']
    cx, cy = tb['center']
    sx, sy = tb['size']
    h, th = tb['height'], tb['top_thickness']
    objs.append(dict(id='table_top', shape='box', dims=[sx, sy, th], xyz=[cx, cy, h - th / 2], quat=q0,
                     color=[0.35, 0.37, 0.40, 1], group='table'))
    for i, (dx, dy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        objs.append(dict(id=f'table_leg{i}', shape='box', dims=[0.06, 0.06, h - th],
                         xyz=[cx + dx * (sx / 2 - 0.05), cy + dy * (sy / 2 - 0.05), (h - th) / 2],
                         quat=q0, color=[0.25, 0.25, 0.27, 1], group='table'))

    # workpiece (actual pose = nominal + offset)
    Tp = part_tf(cell, offset)
    qp = quat_from_matrix(Tp[:3, :3])
    part = cell['part']

    def at(local):
        return (Tp[:3, :3] @ np.array(local, float) + Tp[:3, 3]).tolist()

    px, py, pt = part['plate']
    rib, tube = part['rib'], part['tube']
    steel = [0.62, 0.64, 0.68, 1]
    objs.append(dict(id='part_plate', shape='box', dims=[px, py, pt], xyz=at([0, 0, -pt / 2]),
                     quat=qp.tolist(), color=steel, group='part'))
    objs.append(dict(id='part_rib', shape='box', dims=[rib['length'], rib['thickness'], rib['height']],
                     xyz=at([0, rib['y'], rib['height'] / 2]), quat=qp.tolist(), color=steel, group='part'))
    objs.append(dict(id='part_tube', shape='cylinder', dims=[tube['radius'], tube['height']],
                     xyz=at([tube['x'], tube['y'], tube['height'] / 2]), quat=qp.tolist(), color=steel,
                     group='part'))

    # fixture: clamps on the plate edges and locating pins (nominal pose, the fixture is fixed)
    Tn = part_tf(cell)
    qn = quat_from_matrix(Tn[:3, :3]).tolist()

    def atn(local):
        return (Tn[:3, :3] @ np.array(local, float) + Tn[:3, 3]).tolist()

    cs = cell['fixture']['clamp_size']
    for i, (x, y) in enumerate([(-px / 2 + 0.03, -py / 2), (px / 2 - 0.03, -py / 2),
                                (-px / 2 + 0.03, py / 2), (px / 2 - 0.03, py / 2)]):
        sgn = -1 if y < 0 else 1
        objs.append(dict(id=f'clamp{i}', shape='box', dims=cs, xyz=atn([x, y + sgn * cs[1] / 2 - sgn * 0.01, cs[2] / 2]),
                         quat=qn, color=[0.85, 0.15, 0.10, 1], group='fixture'))
    pr = cell['fixture']['pin_radius']
    for i, (x, y) in enumerate([(-px / 2 - pr, 0.0), (0.0, -py / 2 - pr)]):
        objs.append(dict(id=f'pin{i}', shape='cylinder', dims=[pr, 0.03], xyz=atn([x, y, 0.015 - pt]),
                         quat=qn, color=[0.1, 0.1, 0.1, 1], group='fixture'))

    # equipment
    objs.append(dict(id='power_source', shape='box', dims=[0.6, 0.4, 0.8], xyz=[-0.7, 1.2, 0.4], quat=q0,
                     color=[0.10, 0.30, 0.60, 1], group='equipment'))
    objs.append(dict(id='wire_drum', shape='cylinder', dims=[0.3, 0.8], xyz=[-0.7, 0.55, 0.4], quat=q0,
                     color=[0.15, 0.45, 0.20, 1], group='equipment'))
    objs.append(dict(id='torch_cleaner', shape='box', dims=[0.3, 0.3, 0.9], xyz=[0.35, -1.1, 0.45], quat=q0,
                     color=[0.90, 0.60, 0.05, 1], group='equipment'))

    if include_fence:
        f = cell['fence']
        H, t = f['height'], 0.03
        L = f['x_max'] - f['x_min']
        mx = (f['x_max'] + f['x_min']) / 2
        W = 2 * f['y_half']
        panel = [0.25, 0.27, 0.30, 0.35]
        objs.append(dict(id='fence_back', shape='box', dims=[t, W, H], xyz=[f['x_max'], 0, H / 2], quat=q0,
                         color=panel, group='fence'))
        objs.append(dict(id='fence_left', shape='box', dims=[L, t, H], xyz=[mx, f['y_half'], H / 2], quat=q0,
                         color=panel, group='fence'))
        objs.append(dict(id='fence_right', shape='box', dims=[L, t, H], xyz=[mx, -f['y_half'], H / 2], quat=q0,
                         color=panel, group='fence'))
        objs.append(dict(id='fence_front', shape='box', dims=[t, W, H], xyz=[f['x_min'], 0, H / 2], quat=q0,
                         color=panel, group='fence'))
    return objs


# ---------------------------------------------------------------- Gazebo world

def _geom(o):
    if o['shape'] == 'box':
        return '<box><size>{} {} {}</size></box>'.format(*o['dims'])
    return '<cylinder><radius>{}</radius><length>{}</length></cylinder>'.format(*o['dims'])


def _rpy_from_quat(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def world_sdf(cell, offset=(0.0, 0.0, 0.0)):
    links = []
    for o in cell_objects(cell, offset):
        r, g, b, a = o['color']
        pose = '{} {} {} {} {} {}'.format(*o['xyz'], *_rpy_from_quat(o['quat']))
        transp = f'<transparency>{1 - a:.2f}</transparency>' if a < 1 else ''
        collision = '' if o['group'] == 'fence' else \
            f'<collision name="c"><geometry>{_geom(o)}</geometry></collision>'
        links.append(f"""
      <link name="{o['id']}">
        <pose>{pose}</pose>
        <visual name="v"><geometry>{_geom(o)}</geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular></material>{transp}</visual>
        {collision}
      </link>""")

    f = cell['fence']
    posts = []
    for i, (x, y) in enumerate([(f['x_min'], -f['y_half']), (f['x_min'], f['y_half']),
                                (f['x_max'], -f['y_half']), (f['x_max'], f['y_half'])]):
        posts.append(f"""
      <link name="post{i}"><pose>{x} {y} {f['height'] / 2} 0 0 0</pose>
        <visual name="v"><geometry><box><size>0.08 0.08 {f['height']}</size></box></geometry>
          <material><ambient>0.95 0.75 0.05 1</ambient><diffuse>0.95 0.75 0.05 1</diffuse></material></visual>
      </link>""")
    # yellow floor marking of the robot working area
    mark = """
      <link name="floor_mark"><pose>0.4 0 0.001 0 0 0</pose>
        <visual name="v"><geometry><box><size>3.2 3.4 0.001</size></box></geometry>
          <material><ambient>0.9 0.75 0.1 1</ambient><diffuse>0.9 0.75 0.1 1</diffuse></material></visual>
      </link>
      <link name="floor_inner"><pose>0.4 0 0.0015 0 0 0</pose>
        <visual name="v"><geometry><box><size>3.1 3.3 0.001</size></box></geometry>
          <material><ambient>0.45 0.47 0.5 1</ambient><diffuse>0.45 0.47 0.5 1</diffuse></material></visual>
      </link>"""

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="weld_cell">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -1.0</direction>
    </light>
    <light type="point" name="cell_lamp">
      <pose>1.0 0 3.0 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <attenuation><range>10</range><constant>0.5</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>0.55 0.57 0.6 1</ambient><diffuse>0.55 0.57 0.6 1</diffuse></material></visual>
      </link>
    </model>
    <model name="weld_cell_static">
      <static>true</static>{''.join(links)}{''.join(posts)}{mark}
    </model>
  </world>
</sdf>
"""
