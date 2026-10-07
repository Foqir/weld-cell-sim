"""Offline reachability check of the weld program (no ROS): numeric IK of HDR20-17 + torch
along every seam. Run: python3 tools/check_reach.py  (inside any container with numpy + yaml)."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'ros2_ws', 'src', 'weld_cell')
sys.path.insert(0, PKG)
from weld_cell import geometry as g  # noqa: E402


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def axis_rot(axis, a):
    axis = np.asarray(axis, float)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


# hdr20_17 from hdr_description: (origin xyz, origin pitch, axis, lower, upper)
CHAIN = [
    ([0, 0, 0], 0.0, [0, 0, 1], -3.228, 3.228),
    ([0.16, 0, 0.4466], 1.5708, [0, -1, 0], -1.221, 3.403),
    ([0, 0, 0.77], 0.0, [0, -1, 0], -1.396, 3.141),
    ([0, 0, 0.14], 0.0, [1, 0, 0], -3.403, 3.403),
    ([0.802, 0, 0], 0.0, [0, -1, 0], -2.356, 2.356),
    ([0.1286, 0, 0], 0.0, [1, 0, 0], -6.283, 6.283),
]
BEND, L1, L2 = math.radians(45), 0.14, 0.195
T_FL_TOOL0 = g.make_tf(Ry(1.5708), [0, 0, 0])
T_TOOL0_TCP = g.make_tf(Ry(BEND), [L2 * math.sin(BEND), 0, L1 + L2 * math.cos(BEND)])


def fk(q):
    T = np.eye(4)
    for (xyz, pitch, axis, _, _), qi in zip(CHAIN, q):
        T = T @ g.make_tf(Ry(pitch), xyz) @ g.make_tf(axis_rot(axis, qi), [0, 0, 0])
    return T @ T_FL_TOOL0 @ T_TOOL0_TCP


def ik(Tgoal, q0, iters=300):
    q = np.array(q0, float)
    lo = np.array([c[3] for c in CHAIN])
    hi = np.array([c[4] for c in CHAIN])
    for _ in range(iters):
        T = fk(q)
        ep = Tgoal[:3, 3] - T[:3, 3]
        Re = Tgoal[:3, :3] @ T[:3, :3].T
        eo = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        e = np.concatenate([ep, eo])
        J = np.zeros((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            T2 = fk(q + dq)
            J[:3, i] = (T2[:3, 3] - T[:3, 3]) / 1e-6
            Rd = T2[:3, :3] @ T[:3, :3].T
            J[3:, i] = 0.5 * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / 1e-6
        if np.linalg.norm(ep) < 1e-5 and np.linalg.norm(eo) < 1e-4:
            break
        lam = 0.01
        q = q + J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        q = np.clip(q, lo, hi)
    T = fk(q)
    return q, np.linalg.norm(Tgoal[:3, 3] - T[:3, 3]), np.linalg.svd(J, compute_uv=False)[-1]


def ik_multi(Tgoal, q_prev, home, tries=40):
    """IK with restarts (like MoveIt/KDL): returns the converged solution closest to q_prev."""
    rng = np.random.default_rng(0)
    lo = np.array([c[3] for c in CHAIN])
    hi = np.array([c[4] for c in CHAIN])
    seeds = [np.array(q_prev), np.array(home)]
    for d6 in (math.pi, -math.pi):
        seeds.append(np.array(q_prev) + np.array([0, 0, 0, 0, 0, d6]))
    seeds += [rng.uniform(lo, hi) for _ in range(tries)]
    best = None
    for s in seeds:
        q, err, _ = ik(Tgoal, np.clip(s, lo, hi), iters=500)
        if err < 1e-4:
            d = np.linalg.norm(q - q_prev)
            if best is None or d < best[0]:
                best = (d, q)
    return best[1] if best else np.array(q_prev)


def main():
    cell = g.load_yaml(os.path.join(PKG, 'config', 'cell.yaml'))
    prog = g.load_yaml(os.path.join(PKG, 'config', 'weld_program.yaml'))
    q = np.array(prog['robot']['home_joints'], float)
    T = fk(q)
    print('home TCP xyz', np.round(T[:3, 3], 3), ' TCP z-axis', np.round(T[:3, 2], 2))
    ok_all = True
    for seam in prog['seams']:
        P, R, s = g.seam_poses(seam, cell, prog['robot'], step=0.005)
        q_start = ik_multi(g.make_tf(R[0], P[0]), q, prog['robot']['home_joints'])
        # like the weld program: try the other turns of j6 if the seam runs into the j6 limit
        for k in (0, 1, -1):
            q = q_start.copy()
            q[5] += k * 2 * math.pi
            if abs(q[5]) > 6.283:
                continue
            worst, min_sv, qs = 0.0, 1e9, []
            for p, r in zip(P, R):
                q, err, sv = ik(g.make_tf(r, p), q)
                qs.append(q)
                worst, min_sv = max(worst, err), min(min_sv, sv)
            qs = np.array(qs)
            jump = np.max(np.abs(np.diff(qs, axis=0))) if len(qs) > 1 else 0
            ok = worst < 1e-3 and jump < 0.2
            if ok:
                break
        ok_all &= ok
        print(f"{seam['name']:10s} len={s[-1]*1000:6.1f} mm  max pos err={worst*1000:6.3f} mm  "
              f"min sing.val={min_sv:.3f}  max joint step={jump:.3f} rad  {'OK' if ok else 'PROBLEM'}")
        print('   start q', np.round(qs[0], 2), '\n   end   q', np.round(qs[-1], 2))
        print('   joint ranges used:', np.round(qs.min(0), 2), np.round(qs.max(0), 2))
    print('ALL REACHABLE' if ok_all else 'SOME SEAMS NOT REACHABLE')


if __name__ == '__main__':
    main()
