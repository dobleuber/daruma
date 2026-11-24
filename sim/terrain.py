"""Test terrains: ramps and steps in front of the robot.

Typical use:
    from terrain import load_with_ramp, load_with_step
    out = simulate(params, 20, load_fn=lambda robot: load_with_ramp(10, robot=robot))
"""

import mujoco
import numpy as np

import robots

START_X = 0.12  # m, distance from the robot to the obstacle


def _compile(spec):
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def _add_box(spec, name, pos, size, euler=(0, 0, 0)):
    g = spec.worldbody.add_geom()
    g.name = name
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.pos = pos
    g.size = size
    g.quat = _euler_to_quat(euler)
    g.contype = 1
    g.conaffinity = 2
    g.friction = [0.9, 0.02, 0.01]
    g.rgba = [0.75, 0.6, 0.4, 1]
    return g


def _euler_to_quat(euler):
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.asarray(euler, dtype=float), "xyz")
    return q


def load_with_ramp(angle_deg: float, length: float = 0.8, robot=None):
    """Ramp that rises in +x, starting at START_X."""
    spec = mujoco.MjSpec.from_file(str(robots.get(robot).scene))
    th = np.radians(angle_deg)
    L, t = length / 2, 0.01
    center = [START_X + L * np.cos(th) + t * np.sin(th), 0, L * np.sin(th) - t * np.cos(th)]
    _add_box(spec, "ramp", center, [L, 0.3, t], euler=(0, -th, 0))
    # Platform at the end so it does not fall off the edge.
    top = length * np.sin(th)
    x_end = START_X + length * np.cos(th)
    _add_box(spec, "ramp_top", [x_end + 0.3, 0, top / 2], [0.3, 0.3, top / 2])
    return _compile(spec)


def load_with_step(height: float, depth: float = 0.6, robot=None):
    """Step (platform) of height `height` starting at START_X."""
    spec = mujoco.MjSpec.from_file(str(robots.get(robot).scene))
    _add_box(spec, "step", [START_X + depth / 2, 0, height / 2], [depth / 2, 0.3, height / 2])
    return _compile(spec)


def ramp_progress(out):
    """How far it advanced on the ramp (m, measured along the floor)."""
    return max(0.0, out["x"] - START_X)


def on_step(model, data, height):
    """True if both feet are on top of the step."""
    bottoms = []
    for s in ("left", "right"):
        g = model.geom(f"{s}_sole").id
        bottoms.append(data.geom_xpos[g][2] - model.geom_size[g][2])
    return min(bottoms) > height * 0.6 and data.qpos[0] > START_X
