"""Checks collisions between the leg parts (real meshes) over the whole range of motion.

Uses the STLs exported by leg.scad (stl/ folder). The servos are checked with a model
shrunk by 0.3 mm (servo_check.stl) so that the faces where the parts rest/are bolted
do not count as collisions.

  python check_leg.py            # collision-free ranges per joint + gait
"""

import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
STL = HERE / "stl"

# Dimensions: the same as in lib/params.scad
from build_mjcf import read_params  # noqa: E402

_P = read_params()
S_DISC_Z = _P["S_DISC_Z"]
D_YAW_ROLL, D_ROLL_PITCH = _P["D_YAW_ROLL"], _P["D_ROLL_PITCH"]
D_THIGH, D_SHIN, D_ANKLE = _P["D_THIGH"], _P["D_SHIN"], _P["D_ANKLE"]
HIP_Y = _P["HIP_Y"]
SHIN_X = [0, 0, 1] if _P["SHIN_UP"] else [1, 0, 0]
DEPTH_TOL = 0.2  # mm


def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        R = [[1, 0, 0], [0, c, -s], [0, s, c]]
    elif axis == "y":
        R = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    else:
        R = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    M = np.eye(4)
    M[:3, :3] = R
    return M


def tr(x, y, z):
    M = np.eye(4)
    M[:3, 3] = [x, y, z]
    return M


def servo_frame(o, Z, X):
    Z, X = np.array(Z, float), np.array(X, float)
    Y = np.cross(Z, X)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = X, Y, Z
    M[:3, 3] = o
    return M


def leg_objects(q):
    """List of (name, file, transform in the leg frame) for the pose q (rad)."""
    yaw, roll, pitch, knee, ankle, aroll = q
    out = [("servo_yaw", "servo_check", servo_frame([0, 0, S_DISC_Z], [0, 0, -1], [-1, 0, 0]))]
    L1 = rot("z", yaw)
    out.append(("yaw_link", "yaw_link", L1))
    L2 = L1 @ tr(0, 0, -D_YAW_ROLL) @ rot("x", roll)
    out += [("servo_roll", "servo_check", L2 @ servo_frame([0, 0, 0], [1, 0, 0], [0, 1, 0])),
            ("roll_link", "roll_link", L2),
            ("servo_pitch", "servo_check", L2 @ servo_frame([0, 0, -D_ROLL_PITCH], [0, 1, 0], [1, 0, 0]))]
    L3 = L2 @ tr(0, 0, -D_ROLL_PITCH) @ rot("y", pitch)
    out += [("thigh", "thigh", L3),
            ("servo_knee", "servo_check", L3 @ servo_frame([0, 0, -D_THIGH], [0, 1, 0], [-1, 0, 0]))]
    L4 = L3 @ tr(0, 0, -D_THIGH) @ rot("y", knee)
    out += [("shin", "shin", L4),
            ("servo_ankle", "servo_check", L4 @ servo_frame([0, 0, -D_SHIN], [0, 1, 0], SHIN_X))]
    L5 = L4 @ tr(0, 0, -D_SHIN) @ rot("y", ankle)
    out += [("ankle_cross", "ankle_cross", L5),
            ("servo_ankle_roll", "servo_check", L5 @ servo_frame([0, 0, -D_ANKLE], [1, 0, 0], [0, 1, 0]))]
    L6 = L5 @ tr(0, 0, -D_ANKLE) @ rot("x", aroll)
    out += [("foot", "foot", L6), ("sole", "sole", L6)]
    return out


_MESHES = {}


def mesh(name):
    if name not in _MESHES:
        _MESHES[name] = trimesh.load(STL / f"{name}.stl")
    return _MESHES[name]


MIRROR = np.diag([1.0, -1.0, 1.0, 1.0])


def collisions(q_left, q_right=None):
    """Pairs that interpenetrate by more than DEPTH_TOL mm. q_right in the simulation's convention."""
    cm = trimesh.collision.CollisionManager()
    for n, f, M in leg_objects(q_left):
        cm.add_object("L_" + n, mesh(f), transform=tr(0, HIP_Y, 0) @ M)
    if q_right is not None:
        yaw, roll, pitch, knee, ankle, aroll = q_right
        for n, f, M in leg_objects([-yaw, -roll, pitch, knee, ankle, -aroll]):
            m = mesh(f).copy()
            m.apply_transform(MIRROR @ tr(0, HIP_Y, 0) @ M)
            cm.add_object("R_" + n, m)
    hit, contacts = cm.in_collision_internal(return_names=False, return_data=True)
    bad = {}
    for c in contacts:
        if c.depth > DEPTH_TOL:
            key = tuple(sorted(c.names))
            bad[key] = max(bad.get(key, 0), c.depth)
    return bad


sys.path.insert(0, str(HERE.parent / "sim"))
from gait import GaitParams, targets  # noqa: E402

STAND = targets(0.0, GaitParams(robot="mini16"))[:6]  # standing pose of the gait
RANGES = {  # simulation range (rad)
    "hip_yaw": (0, -0.6, 0.6), "hip_roll": (1, -0.5, 0.5), "hip_pitch": (2, -1.0, 1.6),
    "knee": (3, -2.4, 0.1), "ankle": (4, -1.4, 1.4), "ankle_roll": (5, -0.5, 0.5),
}


def free_range(j, lo, hi, base=STAND, step=0.05):
    """Continuous collision-free range around the base pose, moving only joint j."""
    ok = {}
    for a in np.arange(lo, hi + 1e-9, step):
        q = base.copy()
        q[j] = a
        ok[round(a, 3)] = collisions(q)
    a0 = base[j]
    keys = sorted(ok)
    i0 = min(range(len(keys)), key=lambda i: abs(keys[i] - a0))
    lo_ok = hi_ok = keys[i0]
    for k in keys[i0::-1]:
        if ok[k]:
            break
        lo_ok = k
    for k in keys[i0:]:
        if ok[k]:
            break
        hi_ok = k
    first_hit = {k: v for k, v in ok.items() if v}
    return lo_ok, hi_ok, first_hit


def main():
    print("Standing pose:", collisions(STAND, STAND) or "no collisions")
    print("\nCollision-free range per joint (from the standing pose):")
    for name, (j, lo, hi) in RANGES.items():
        a, b, hits = free_range(j, lo, hi)
        culprits = set()
        for v in hits.values():
            culprits |= {p for k in v for p in k}
        print(f"  {name:11s} {np.degrees(a):6.0f}° .. {np.degrees(b):5.0f}°   "
              f"(simulation {np.degrees(lo):.0f}..{np.degrees(hi):.0f}°)  parts: {sorted(culprits)[:4]}")
    if len(sys.argv) > 1 and sys.argv[1] == "--gait":
        for turn in (0.0, 0.3, -0.3):
            p = GaitParams(robot="mini16", turn=turn, steer=False)
            worst = {}
            for t in np.arange(2.0, 4.0, 0.02):
                q = targets(t, p)
                for k, v in collisions(q[0:6], q[6:12]).items():
                    worst[k] = max(worst.get(k, 0), v)
            print(f"\nGait (turn {turn}):", {k: round(v, 1) for k, v in worst.items()} or "no collisions")


if __name__ == "__main__":
    main()
