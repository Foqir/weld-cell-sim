#!/usr/bin/env python3
"""Weld program executor - the "robot controller job" of the digital twin.

For every seam of config/weld_program.yaml:
  PTP air move (MoveIt/OMPL, collision-aware) -> linear approach -> arc ignition + dwell ->
  weld at constant TCP speed -> crater fill -> arc off -> linear retreat.
Linear parts are solved point by point (IK every 1 mm, seeded with the previous point) and timed here with a trapezoidal
TCP speed profile, then executed by the joint_trajectory_controller in Gazebo.

While running it measures the real (simulated) TCP from TF, publishes /weld/arc_on, /weld/state and
RViz markers (seams, bead with cooling colours, arc), draws the bead in Gazebo, and writes
CSV + summary.json + plots to /root/shared/logs/weld_<time>/.

  ros2 run weld_cell weld_program.py --ros-args -p use_sim_time:=true [-p seams:="['rib_left']"]
        [-p speed_factor:=1.0] [-p cycles:=1] [-p part_dx:=0.0 -p part_dy:=0.0 -p part_dyaw:=0.0]
"""
import collections
import csv
import json
import math
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, ObjectColor, PlanningScene,
                             RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from weld_cell import geometry as g
from weld_cell import report

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
J6_LIMIT = 6.283
ARC_START_DWELL = 0.3   # s, arc ignition at standstill
CRATER_DWELL = 0.5      # s, crater fill at the seam end
IN_POSITION = 0.0005    # m, TCP must be this close to the seam start before the arc is ignited
T_PEAK, T_AMB, T_TAU = 1500.0, 20.0, 5.0   # bead "temperature" for the cooling colours


def to_pose(p, R):
    q = g.quat_from_matrix(R)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, p)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


def dur(t):
    return DurationMsg(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def trapezoid_times(s, v, a):
    """Time stamps for arc-length samples s (from 0 to L) with speed v and acceleration a."""
    L = float(s[-1])
    if L < 1e-9:
        return np.zeros_like(s)
    s_acc = v * v / (2 * a)
    if 2 * s_acc > L:                       # triangle profile
        v = math.sqrt(a * L)
        s_acc = L / 2
    t_acc = v / a
    t_total = 2 * t_acc + (L - 2 * s_acc) / v
    t = np.empty_like(s)
    for i, si in enumerate(s):
        if si < s_acc:
            t[i] = math.sqrt(2 * si / a)
        elif si <= L - s_acc:
            t[i] = t_acc + (si - s_acc) / v
        else:
            t[i] = t_total - math.sqrt(max(0.0, 2 * (L - si) / a))
    return t


def temp_color(T):
    """Glowing weld metal: white -> yellow -> orange -> red -> steel grey."""
    stops = [(20, (0.30, 0.28, 0.26)), (450, (0.35, 0.25, 0.20)), (700, (0.75, 0.10, 0.02)),
             (1000, (1.00, 0.45, 0.02)), (1300, (1.00, 0.85, 0.30)), (1500, (1.00, 1.00, 0.90))]
    if T <= stops[0][0]:
        return stops[0][1]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if T <= t1:
            k = (T - t0) / (t1 - t0)
            return tuple(c0[i] + k * (c1[i] - c0[i]) for i in range(3))
    return stops[-1][1]


class GzMarkers:
    """Draws the weld bead and the arc glow in the Gazebo GUI through its /marker_array service."""

    def __init__(self, logger):
        self.log = logger
        self.q = queue.Queue()
        self.gz = shutil.which('gz')
        self.enabled = False
        if self.gz:
            try:
                out = subprocess.run([self.gz, 'service', '-l'], capture_output=True, text=True, timeout=10).stdout
                self.enabled = '/marker_array' in out
            except Exception:  # noqa: BLE001
                pass
        if self.enabled:
            threading.Thread(target=self._worker, daemon=True).start()
        else:
            self.log.info('Gazebo GUI marker service not found - bead is drawn in RViz only')

    @staticmethod
    def _mk(ns, mid, mtype, p, quat, scale, rgb, emissive=False):
        r, gg, b = rgb
        em = f', emissive: {{r: {r}, g: {gg}, b: {b}, a: 1}}' if emissive else ''
        return (f'marker {{ action: ADD_MODIFY, ns: "{ns}", id: {mid}, type: {mtype}, '
                f'pose: {{position: {{x: {p[0]:.5f}, y: {p[1]:.5f}, z: {p[2]:.5f}}}, '
                f'orientation: {{x: {quat[0]:.5f}, y: {quat[1]:.5f}, z: {quat[2]:.5f}, w: {quat[3]:.5f}}}}}, '
                f'scale: {{x: {scale[0]}, y: {scale[1]}, z: {scale[2]}}}, '
                f'material: {{ambient: {{r: {r}, g: {gg}, b: {b}, a: 1}}, diffuse: {{r: {r}, g: {gg}, b: {b}, a: 1}}{em}}} }}')

    def bead_segment(self, mid, p0, p1, width):
        if not self.enabled:
            return
        d = np.asarray(p1) - np.asarray(p0)
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return
        mid_p = (np.asarray(p0) + np.asarray(p1)) / 2
        self.q.put(self._mk('bead', mid, 'CAPSULE', mid_p, g.quat_between_z(d), (width, width, L),
                            (0.45, 0.38, 0.30)))

    def arc(self, p, on):
        if not self.enabled:
            return
        if on:
            self.q.put(self._mk('arc', 1, 'SPHERE', p, (0, 0, 0, 1), (0.025, 0.025, 0.025),
                                (0.75, 0.85, 1.0), emissive=True))
        else:
            self.q.put('marker { action: DELETE_MARKER, ns: "arc", id: 1 }')

    def clear(self):
        if self.enabled:
            self.q.put('marker { action: DELETE_ALL }')

    def _worker(self):
        while True:
            items = [self.q.get()]
            while not self.q.empty() and len(items) < 50:
                items.append(self.q.get())
            req = ' '.join(items)
            try:
                subprocess.run([self.gz, 'service', '-s', '/marker_array', '--reqtype', 'gz.msgs.Marker_V',
                                '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', req],
                               capture_output=True, timeout=10)
            except Exception as e:  # noqa: BLE001
                self.log.warn(f'gz marker call failed: {e}')


class WeldProgram(Node):
    def __init__(self):
        super().__init__('weld_program')
        share = get_package_share_directory('weld_cell')
        self.declare_parameter('program', os.path.join(share, 'config', 'weld_program.yaml'))
        self.declare_parameter('cell', os.path.join(share, 'config', 'cell.yaml'))
        self.declare_parameter('log_root', '/root/shared/logs')
        self.declare_parameter('seams', [''])
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('cycles', 1)
        self.declare_parameter('gz_markers', True)
        for n in ('part_dx', 'part_dy', 'part_dyaw'):
            self.declare_parameter(n, 0.0)

        self.prog = g.load_yaml(self.get_parameter('program').value)
        self.cell = g.load_yaml(self.get_parameter('cell').value)
        self.robot = self.prog['robot']
        self.motion = self.prog['motion']
        self.speed_factor = float(self.get_parameter('speed_factor').value)
        self.offset = tuple(float(self.get_parameter(n).value) for n in ('part_dx', 'part_dy', 'part_dyaw'))

        cb = ReentrantCallbackGroup()
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=cb)
        self.move_ac = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self.traj_ac = ActionClient(self, FollowJointTrajectory,
                                    '/joint_trajectory_controller/follow_joint_trajectory', callback_group=cb)
        self.joints = None
        self.create_subscription(JointState, '/joint_states', self._on_joints, 50, callback_group=cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arc_pub = self.create_publisher(Bool, '/weld/arc_on', 10)
        self.state_pub = self.create_publisher(String, '/weld/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/weld/markers', 10)

        # live state shared with the monitor timer
        self.lock = threading.Lock()
        self.job = None            # current seam execution: timing windows, nominal path, ...
        self.arc_on = False
        self.beads = {}            # seam index -> list of (point, time)
        self.hist = collections.deque()
        self.rows = []
        self.gz_bead_id = 0
        self.gz = None
        self.t_start = None
        self.tick = 0
        self.create_timer(0.02, self._monitor, callback_group=cb)

    # ------------------------------------------------------------ helpers
    def _on_joints(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.joints = [d[j] for j in JOINTS]

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, fut, timeout=60.0):
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise TimeoutError('ROS call timed out')
            time.sleep(0.005)
        return fut.result()

    def robot_state(self, q):
        rs = RobotState()
        rs.joint_state.name = JOINTS
        rs.joint_state.position = [float(x) for x in q]
        return rs

    def ik(self, p, R, seed, avoid_collisions=True, timeout=0.2):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.robot['group']
        req.ik_request.ik_link_name = self.robot['tcp_link']
        req.ik_request.robot_state = self.robot_state(seed)
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = to_pose(p, R)
        req.ik_request.timeout.nanosec = int(timeout * 1e9)
        res = self.wait(self.ik_cli.call_async(req))
        if res.error_code.val != 1:
            return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in JOINTS]

    def linear(self, q_start, P, R, avoid_collisions=True, max_jump=0.1):
        """Linear TCP motion through dense Cartesian points P/R (1 mm apart): IK in every point,
        seeded with the previous solution - like the linear interpolation of a robot controller.
        (MoveIt's compute_cartesian_path re-samples its result in time, which is too coarse here.)
        Returns (fraction, Q) with Q[0] = q_start and one joint vector per point."""
        Q = [np.array(q_start, float)]
        for k, (p, r) in enumerate(zip(P, R)):
            q = self.ik(p, r, Q[-1], avoid_collisions=avoid_collisions, timeout=0.05)
            if q is None or np.abs(np.array(q) - Q[-1]).max() > max_jump:
                return k / len(P), np.array(Q)
            Q.append(np.array(q))
        return 1.0, np.array(Q)

    @staticmethod
    def line(p0, p1, R, step):
        n = max(1, int(math.ceil(np.linalg.norm(np.asarray(p1) - p0) / step)))
        P = [np.asarray(p0) + (np.asarray(p1) - p0) * (k / n) for k in range(1, n + 1)]
        return P, [R] * n

    def apply_scene(self):
        ps = PlanningScene()
        ps.is_diff = True
        for o in g.cell_objects(self.cell):            # the program only knows the NOMINAL cell
            co = CollisionObject()
            co.header.frame_id = 'world'
            co.id = o['id']
            prim = SolidPrimitive()
            if o['shape'] == 'box':
                prim.type = SolidPrimitive.BOX
                prim.dimensions = [float(x) for x in o['dims']]
            else:
                prim.type = SolidPrimitive.CYLINDER
                prim.dimensions = [float(o['dims'][1]), float(o['dims'][0])]   # height, radius
            co.primitives.append(prim)
            co.primitive_poses.append(to_pose(o['xyz'], g.matrix_from_quat(o['quat'])))
            co.operation = CollisionObject.ADD
            ps.world.collision_objects.append(co)
            c = ObjectColor()
            c.id = o['id']
            c.color = ColorRGBA(r=float(o['color'][0]), g=float(o['color'][1]), b=float(o['color'][2]),
                                a=float(o['color'][3]))
            ps.object_colors.append(c)
        req = ApplyPlanningScene.Request()
        req.scene = ps
        self.wait(self.scene_cli.call_async(req))

    def escape_from_part(self):
        """After an aborted job the torch may still be at the part: back off along the wire first."""
        try:
            tf = self.tf_buffer.lookup_transform('world', self.robot['tcp_link'], Time(),
                                                 timeout=Duration(seconds=2.0))
        except Exception:  # noqa: BLE001
            return
        t, r = tf.transform.translation, tf.transform.rotation
        p = np.array([t.x, t.y, t.z])
        R = g.matrix_from_quat([r.x, r.y, r.z, r.w])
        tb = self.cell['table']
        near = (abs(p[0] - tb['center'][0]) < tb['size'][0] / 2 + 0.1 and
                abs(p[1] - tb['center'][1]) < tb['size'][1] / 2 + 0.1 and p[2] < tb['height'] + 0.15)
        if not near:
            return
        self.get_logger().warn('torch is close to the part - backing off along the torch axis first')
        target = p - 0.12 * R[:, 2]
        target[2] = max(target[2], tb['height'] + 0.25)
        f, Q = self.linear(self.joints, *self.line(p, target, R, 0.005), avoid_collisions=False)
        if f > 0.5 and len(Q) > 1:
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))])
            T = s / max(s[-1], 1e-6) * max(2.0, s[-1] / 0.3) + 0.2
            gh, _ = self.execute(Q, T)
            self.wait(gh.get_result_async(), timeout=60)

    def ptp(self, q_goal, scaling=None):
        scaling = scaling or float(self.motion['air_velocity_scaling'])
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.robot['group']
        r.num_planning_attempts = 5
        r.allowed_planning_time = 10.0
        r.max_velocity_scaling_factor = scaling
        r.max_acceleration_scaling_factor = scaling
        r.pipeline_id = 'ompl'
        r.planner_id = 'RRTConnectkConfigDefault'
        r.start_state.is_diff = True
        c = Constraints()
        for j, v in zip(JOINTS, q_goal):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        r.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        for attempt in range(4):
            # OMPL is randomised and the time parameterisation may round a corner into the part:
            # simply plan again (MoveIt error -2 = planning failed / invalid plan)
            gh = self.wait(self.move_ac.send_goal_async(goal))
            if not gh.accepted:
                raise RuntimeError('PTP goal rejected')
            res = self.wait(gh.get_result_async(), timeout=180).result
            if res.error_code.val == 1:
                return
            if res.error_code.val != -2:
                break
            self.get_logger().warn(f'PTP planning attempt {attempt + 1} failed, re-planning')
        raise RuntimeError(f'PTP move failed, MoveIt error {res.error_code.val}')

    def execute(self, Q, T, start_delay=0.3):
        """Sends the trajectory with an explicit start time (sim clock) and returns (goal handle, t_start)."""
        t_start = self.now_s() + start_delay
        jt = JointTrajectory()
        jt.header.stamp = Time(nanoseconds=int(t_start * 1e9), clock_type=self.get_clock().clock_type).to_msg()
        jt.joint_names = JOINTS
        V = np.zeros_like(Q)
        if len(T) > 2:
            V = np.gradient(Q, T, axis=0)
            V[0] = V[-1] = 0.0
            # standstill (dwell) points: repeated positions -> exactly zero velocity, otherwise the
            # controller's cubic spline bulges past the stop point
            same = np.all(np.abs(np.diff(Q, axis=0)) < 1e-9, axis=1)
            V[np.concatenate([same, [False]]) | np.concatenate([[False], same])] = 0.0
        for q, v, t in zip(Q, V, T):
            jt.points.append(JointTrajectoryPoint(positions=[float(x) for x in q], velocities=[float(x) for x in v],
                                                  time_from_start=dur(float(t))))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        gh = self.wait(self.traj_ac.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('trajectory rejected by the controller')
        return gh, t_start

    # ------------------------------------------------------------ seam planning
    def plan_seam(self, idx, seam):
        step = float(self.motion['step'])
        P, R, s = g.seam_poses(seam, self.cell, self.robot, step=step)
        d_app = float(self.motion['approach_distance'])
        p_app = P[0] - d_app * R[0][:, 2]
        p_ret = P[-1] - d_app * R[-1][:, 2]
        up = np.array([0.0, 0.0, float(self.motion['clearance_height'])])
        air_step = 5 * step

        seeds = [self.joints, self.robot['home_joints']]
        seeds += [list(np.array(sd) + np.array([0, 0, 0, 0, 0, sgn * math.pi])) for sd in seeds for sgn in (1, -1)]
        tried, why = [], []
        for seed in seeds:
            q_app = self.ik(p_app, R[0], seed)
            if q_app is None:
                why.append('ik(approach)')
                continue
            for k in (0, 1, -1):              # other turns of the endless-ish 6th axis
                q = list(q_app)
                q[5] += 2 * math.pi * k
                if abs(q[5]) > J6_LIMIT or any(np.allclose(q, t, atol=1e-3) for t in tried):
                    continue
                tried.append(q)
                # clearance point straight above the approach point: PTP air moves end/start up there,
                # the descent to the part is linear and collision-checked every few millimetres
                f0, Q0 = self.linear(q, *self.line(p_app, p_app + up, R[0], air_step))
                if f0 < 0.999:
                    why.append(f'clearance {f0:.2f}')
                    continue
                f1, Q1 = self.linear(q, *self.line(p_app, P[0], R[0], step))
                if f1 < 0.999:
                    why.append(f'approach {f1:.2f}')
                    continue
                f2, Q2 = self.linear(Q1[-1], P[1:], R[1:])
                if f2 < 0.999:
                    f2, Q2 = self.linear(Q1[-1], P[1:], R[1:], avoid_collisions=False)
                    if f2 < 0.999:
                        why.append(f'weld {f2:.2f}')
                        continue
                    self.get_logger().warn(f'{seam["name"]}: weld path touches the scene model (allowed)')
                f3, Q3 = self.linear(Q2[-1], *self.line(P[-1], p_ret, R[-1], step))
                if f3 < 0.999:
                    why.append(f'retreat {f3:.2f}')
                    continue
                f4, Q4 = self.linear(Q3[-1], *self.line(p_ret, p_ret + up, R[-1], air_step))
                if f4 < 0.999:
                    why.append(f'rise {f4:.2f}')
                    continue
                return dict(q_app=list(Q0[-1]), Q0=Q0[::-1], Q1=Q1, Q2=Q2, Q3=Q3, Q4=Q4, P=P, R=R, s=s)
        self.get_logger().warn(f'{seam["name"]}: planning attempts failed: {", ".join(why)}')
        return None

    def timeline(self, plan, seam):
        """Joint trajectory with timing + arc on/off window (relative to trajectory start)."""
        a = float(self.motion['accel'])
        v_air = float(self.motion['approach_speed'])
        v_weld = float(seam['speed']) * self.speed_factor
        d_app = float(self.motion['approach_distance'])

        def uniform_s(Q, L):
            return np.linspace(0.0, L, len(Q))

        h = float(self.motion['clearance_height'])
        v_clr = float(self.motion['clearance_speed'])
        segs = []
        # descent from the clearance point
        T0 = trapezoid_times(uniform_s(plan['Q0'], h), v_clr, a) + 0.2
        segs.append((plan['Q0'], T0))
        t0 = T0[-1] + 0.05
        # approach
        T1 = trapezoid_times(uniform_s(plan['Q1'], d_app), v_air, a) + t0
        segs.append((plan['Q1'], T1))
        t_arc_on = T1[-1]
        # ignition dwell
        t = T1[-1] + ARC_START_DWELL
        # weld
        Q2 = plan['Q2']
        s = plan['s'] if len(plan['s']) == len(Q2) else uniform_s(Q2, plan['s'][-1])
        T2 = trapezoid_times(s, v_weld, a) + t
        t_weld_start = t
        t = T2[-1] + CRATER_DWELL
        t_arc_off = t
        segs.append((Q2, T2))
        segs.append((Q2[-1:], np.array([t])))            # crater dwell (hold the last point)
        T3 = trapezoid_times(uniform_s(plan['Q3'], d_app), v_air, a) + t
        segs.append((plan['Q3'][1:], T3[1:]))
        # rise to the clearance point
        T4 = trapezoid_times(uniform_s(plan['Q4'], h), v_clr, a) + T3[-1] + 0.05
        segs.append((plan['Q4'], T4))
        Q = np.vstack([q for q, _ in segs])
        T = np.concatenate([tt for _, tt in segs])
        keep = np.concatenate([[True], np.diff(T) > 1e-6])
        return Q[keep], T[keep], (t_arc_on, t_arc_off), (t_weld_start, T2[-1])

    # ------------------------------------------------------------ monitoring
    def tcp(self):
        try:
            tf = self.tf_buffer.lookup_transform('world', self.robot['tcp_link'], Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:  # noqa: BLE001
            return None

    def _monitor(self):
        if self.t_start is None:
            return
        p = self.tcp()
        if p is None:
            return
        now = self.now_s()
        self.tick += 1
        with self.lock:
            job = self.job
            arc = False
            if job is not None and job['t_exec'] is not None:
                el = now - job['t_exec']
                if job['ignited'] is None and el >= job['arc'][0]:
                    # ignite only when the TCP is "in position" at the seam start (or the dwell is over)
                    if np.linalg.norm(p - job['nominal'][0]) < IN_POSITION or el >= job['arc'][0] + ARC_START_DWELL:
                        job['ignited'] = el
                arc = bool(job['ignited'] is not None and el <= job['arc'][1])
            if arc != self.arc_on:
                self.arc_on = arc
                self.arc_pub.publish(Bool(data=arc))
                if self.gz and not arc:
                    self.gz.arc(p, False)
            # TCP speed over a ~0.2 s window (TF updates slower than this timer)
            self.hist.append((p, now))
            while len(self.hist) > 2 and now - self.hist[1][1] >= 0.2:
                self.hist.popleft()
            p0, t0 = self.hist[0]
            speed = float(np.linalg.norm(p - p0) / (now - t0)) if now - t0 > 0.05 else 0.0
            dev_nom = dev_act = float('nan')
            seam_name = '-'
            if job is not None:
                seam_name = job['name']
                if arc:
                    dev_nom = g.point_to_polyline(p, job['nominal'])
                    dev_act = g.point_to_polyline(p, job['actual'])
                    pts = self.beads.setdefault(job['idx'], [])
                    if not pts or np.linalg.norm(p - pts[-1][0]) > 0.0015:
                        pts.append((p.copy(), now))
                    if self.gz and (job.get('gz_last') is None or np.linalg.norm(p - job['gz_last']) > 0.004):
                        if job.get('gz_last') is not None:
                            self.gz_bead_id += 1
                            self.gz.bead_segment(self.gz_bead_id, job['gz_last'], p, 0.007)
                        job['gz_last'] = p.copy()
                        self.gz.arc(p, True)
            self.rows.append([round(now - self.t_start, 4), seam_name, int(arc), *np.round(p, 6),
                              round(dev_nom * 1000, 4), round(dev_act * 1000, 4), round(speed * 1000, 3),
                              *(np.round(self.joints, 5) if self.joints else [float('nan')] * 6)])
            if self.tick % 5 == 0:
                self._publish_markers(p, now, job, arc)

    def _publish_markers(self, p, now, job, arc):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, pts in self.beads.items():
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = 'bead', idx, Marker.SPHERE_LIST, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.007
            for q, t in pts:
                m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
                T = T_AMB + (T_PEAK - T_AMB) * math.exp(-(now - t) / T_TAU)
                r, gg, b = temp_color(T)
                m.colors.append(ColorRGBA(r=r, g=gg, b=b, a=1.0))
            ma.markers.append(m)
        a = Marker()
        a.header.frame_id = 'world'
        a.header.stamp = stamp
        a.ns, a.id, a.type = 'arc', 0, Marker.SPHERE
        a.action = Marker.ADD if arc else Marker.DELETE
        a.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        a.pose.orientation.w = 1.0
        flick = 0.02 + 0.006 * math.sin(now * 40.0)
        a.scale.x = a.scale.y = a.scale.z = flick
        a.color = ColorRGBA(r=0.8, g=0.9, b=1.0, a=0.9)
        ma.markers.append(a)
        txt = Marker()
        txt.header.frame_id = 'world'
        txt.header.stamp = stamp
        txt.ns, txt.id, txt.type, txt.action = 'status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        tp = self.cell['part']['xyz']
        txt.pose.position = Point(x=float(tp[0]), y=float(tp[1]), z=float(tp[2]) + 0.45)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.05
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        txt.text = job['status'] if job else 'air move'
        if arc and job:
            txt.text += '  ARC ON'
        ma.markers.append(txt)
        self.marker_pub.publish(ma)

    def publish_seam_preview(self, seams):
        ma = MarkerArray()
        for i, seam in enumerate(seams):
            for ns, off, col in (('seam_nominal', (0, 0, 0), (0.1, 0.9, 0.2)),
                                 ('seam_actual', self.offset, (0.9, 0.2, 0.9))):
                if ns == 'seam_actual' and not any(self.offset):
                    continue
                P, _, _ = g.seam_poses(seam, self.cell, self.robot, step=0.005, offset=off)
                m = Marker()
                m.header.frame_id = 'world'
                m.ns, m.id, m.type, m.action = ns, i, Marker.LINE_STRIP, Marker.ADD
                m.pose.orientation.w = 1.0
                m.scale.x = 0.0015
                m.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
                m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.0005) for p in P]
                ma.markers.append(m)
        self.marker_pub.publish(ma)

    # ------------------------------------------------------------ main job
    def run(self):
        log = self.get_logger()
        log.info('Waiting for MoveIt, controllers and joint states...')
        for cli in (self.ik_cli, self.scene_cli):
            while not cli.wait_for_service(timeout_sec=2.0):
                log.info(f'  waiting for {cli.srv_name}')
        self.move_ac.wait_for_server()
        self.traj_ac.wait_for_server()
        while self.joints is None:
            time.sleep(0.1)
        time.sleep(1.0)

        wanted = [s for s in self.get_parameter('seams').value if s]
        seams = [s for s in self.prog['seams'] if not wanted or s['name'] in wanted]
        stamp = time.strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.join(self.get_parameter('log_root').value, f'weld_{stamp}')
        os.makedirs(log_dir, exist_ok=True)

        self.apply_scene()
        self.escape_from_part()
        if self.get_parameter('gz_markers').value:
            self.gz = GzMarkers(log)
            self.gz.clear()
        self.publish_seam_preview(seams)
        self.t_start = self.now_s()
        summary = dict(robot_model=os.environ.get('ROBOT_MODEL', 'hdr20_17'), part_offset=self.offset,
                       speed_factor=self.speed_factor, seams=[])

        cycles = int(self.get_parameter('cycles').value)
        for cycle in range(cycles):
            log.info(f'=== cycle {cycle + 1}/{cycles}: home ===')
            with self.lock:
                self.beads.clear()
            if self.gz:
                self.gz.clear()
            self.ptp(self.robot['home_joints'])
            for idx, seam in enumerate(seams):
                log.info(f'--- seam {seam["name"]}: planning')
                plan = self.plan_seam(idx, seam)
                if plan is None:
                    log.error(f'seam {seam["name"]}: no collision-free reachable solution, skipped')
                    summary['seams'].append(dict(name=seam['name'], status='unreachable'))
                    continue
                Q, T, arc, weld = self.timeline(plan, seam)
                log.info(f'--- seam {seam["name"]}: PTP to approach point')
                t_air0 = self.now_s()
                self.ptp(plan['q_app'])
                t_air = self.now_s() - t_air0
                v = float(seam['speed']) * self.speed_factor
                heat = float(self.prog['process']['arc_efficiency']) * seam['voltage'] * seam['current'] / (v * 1000) / 1000
                status = (f"{seam['name']}: I={seam['current']} A  U={seam['voltage']} V  "
                          f"v={v * 60:.2f} m/min  Q={heat:.2f} kJ/mm")
                actual = g.seam_poses(seam, self.cell, self.robot, step=0.002, offset=self.offset)[0]
                job = dict(idx=idx, name=seam['name'], arc=arc, t_exec=None, nominal=plan['P'], actual=actual,
                           status=status, gz_last=None, ignited=None)
                with self.lock:
                    self.job = job
                log.info(f'--- seam {seam["name"]}: welding ({plan["s"][-1] * 1000:.0f} mm, {status})')
                self.state_pub.publish(String(data=json.dumps(dict(seam=seam['name'], phase='weld', **{
                    k: seam[k] for k in ('current', 'voltage', 'speed', 'wire_feed')}))))
                t_seam0 = self.now_s()
                gh, t_exec = self.execute(Q, T)
                with self.lock:
                    job['t_exec'] = t_exec
                res = self.wait(gh.get_result_async(), timeout=T[-1] * 5 + 60).result
                t_seam = self.now_s() - t_seam0
                with self.lock:
                    self.job = None
                if res.error_code != 0:
                    log.warn(f'controller reported error {res.error_code}: {res.error_string}')
                arc_time = arc[1] - (job['ignited'] if job['ignited'] is not None else arc[0])
                wire = self.prog['process']
                area = math.pi * (wire['wire_diameter_mm'] / 1000) ** 2 / 4
                summary['seams'].append(dict(
                    name=seam['name'], status='done', cycle=cycle + 1, type=seam['type'],
                    length_mm=round(float(plan['s'][-1]) * 1000, 1), speed_mm_s=round(v * 1000, 2),
                    current_A=seam['current'], voltage_V=seam['voltage'], wire_feed_m_min=seam['wire_feed'],
                    heat_input_kJ_mm=round(heat, 3), arc_time_s=round(arc_time, 2),
                    weld_time_s=round(weld[1] - weld[0], 2), air_move_s=round(t_air, 2),
                    seam_cycle_s=round(t_seam, 2),
                    wire_mass_g=round(wire['wire_density'] * area * seam['wire_feed'] / 60 * arc_time * 1000, 1)))
            self.ptp(self.robot['home_joints'])

        summary['cycle_time_s'] = round(self.now_s() - self.t_start, 2)
        done = [s for s in summary['seams'] if s['status'] == 'done']
        summary['arc_time_s'] = round(sum(s['arc_time_s'] for s in done), 2)
        summary['arc_on_ratio'] = round(summary['arc_time_s'] / max(summary['cycle_time_s'], 1e-6), 3)
        with self.lock:
            rows = list(self.rows)
        header = ['t', 'seam', 'arc', 'x', 'y', 'z', 'dev_nominal_mm', 'dev_actual_seam_mm', 'speed_mm_s', *JOINTS]
        csv_path = os.path.join(log_dir, 'tcp_log.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
        report.add_deviation_stats(summary, csv_path)
        with open(os.path.join(log_dir, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        try:
            report.make_plots(log_dir)
        except Exception as e:  # noqa: BLE001
            log.warn(f'plots failed: {e}')
        log.info(f'=== DONE: cycle {summary["cycle_time_s"]} s, arc-on {summary["arc_time_s"]} s '
                 f'({summary["arc_on_ratio"] * 100:.0f} %). Report: {log_dir}')
        print(json.dumps(summary, indent=2))


def main():
    rclpy.init()
    node = WeldProgram()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        code = 130
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f'weld program aborted: {e}')
        code = 1
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
