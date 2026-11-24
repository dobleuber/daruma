"""Tests of the models (Mini13, Mini16 and Mini14) and their gait. Run with: python -m pytest -q"""

import mujoco
import numpy as np
import pytest

import common
import robots
from common import load, simulate
from gait import GaitParams, leg_command, targets

ROBOTS = ["mini13", "mini16", "mini14"]

# Per-robot limits. Mini13 (no ankle roll) sways its heading ±15-20° when walking and its
# scripted gait only works on flat ground: ramps and steps are left to RL.
SPEC = {
    "mini13": dict(mass=(0.9, 1.0), height=(0.25, 0.30), min_x=0.4, yaw_tol=25, ramps=[]),
    # Mini16 generated from the CAD (real servos): ~32 cm and ~1.18 kg; with the scripted gait
    # it climbs 10° ramps, not 15° (that is left to RL).
    "mini16": dict(mass=(1.1, 1.25), height=(0.28, 0.36), min_x=0.3, yaw_tol=10, ramps=[10]),
    # Mini14 (XL330, soft): without RL it leans ~8° when standing and the reference gait
    # only takes steps without falling; moving forward and turning is learned by RL.
    "mini14": dict(mass=(0.55, 0.75), height=(0.25, 0.29), min_x=None, yaw_tol=None, ramps=[],
                   stand_tilt=10, torque=0.7, cmd_vel=9.0),
}
STS3215 = dict(stand_tilt=3, torque=1.3, cmd_vel=4.5)  # < ~70 % of the maximum torque; ~4.7 rad/s


def lim(robot, key):
    return SPEC[robot].get(key, STS3215[key])


WALKERS = [r for r in ROBOTS if SPEC[r]["min_x"] is not None]


@pytest.mark.parametrize("robot", ROBOTS)
def test_motors_in_expected_order(robot):
    rb = robots.get(robot)
    model, _ = load(robot)
    names = [model.actuator(i).name for i in range(model.nu)]
    head = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"][: rb.n_head]
    expected = [f"left_{j}" for j in rb.leg_joints] + [f"right_{j}" for j in rb.leg_joints] + head
    assert names == expected
    # The actuator order matches the joint order (qpos 7...)
    for i in range(model.nu):
        assert model.jnt_qposadr[model.actuator_trnid[i, 0]] == 7 + i


@pytest.mark.parametrize("robot", ROBOTS)
def test_size_and_mass(robot):
    model, data = load(robot)
    mass = model.body_subtreemass[model.body("torso").id]
    lo, hi = SPEC[robot]["mass"]
    assert lo < mass <= hi
    head = model.geom("head").id
    head_top = data.geom_xpos[head][2] + model.geom_size[head][2]
    lo, hi = SPEC[robot]["height"]
    assert lo < head_top < hi


@pytest.mark.parametrize("robot", ROBOTS)
def test_stands_still(robot):
    model, data = load(robot)
    for _ in range(int(3 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert not common.fallen(data)
    assert common.torso_tilt(data) < np.radians(lim(robot, "stand_tilt"))


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (0.015, 0.008, 0.005), (-0.012, -0.006, 0.01)])
def test_leg_ik_matches_model(robot, side, offset):
    """The inverse kinematics angles must bring the model's ankle to the requested point."""
    rb = robots.get(robot)
    p = GaitParams(robot=robot)
    target = np.array([0, 0, -(p.hip_height - rb.ankle_to_sole)]) + offset
    model, data = load(robot)
    q = leg_command(0.0, *target, rb)
    for name, val in zip(rb.leg_joints, q):
        data.qpos[model.jnt_qposadr[model.joint(f"{side}_{name}").id]] = val
    data.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
    mujoco.mj_kinematics(model, data)
    ref = np.array([0, model.body(f"{side}_hip_yaw_link").pos[1], 0.5])
    ankle_pos = data.xanchor[model.joint(f"{side}_ankle").id]
    np.testing.assert_allclose(ankle_pos - ref, target, atol=1e-3)
    foot_R = data.xmat[model.body(f"{side}_foot").id].reshape(3, 3)
    assert abs(foot_R[2, 0]) < 1e-3  # foot parallel to the torso in pitch
    if rb.ankle_roll:
        np.testing.assert_allclose(foot_R[2], [0, 0, 1], atol=1e-3)  # and also in roll


@pytest.mark.parametrize("robot", ROBOTS)
def test_targets_within_servo_limits(robot):
    model, _ = load(robot)
    lo, hi = model.actuator_ctrlrange.T
    for turn in (0.0, 0.35, -0.35):
        p = GaitParams(robot=robot, turn=turn)
        for t in np.linspace(0, 6, 300):
            q = targets(t, p)
            assert np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9)


@pytest.mark.parametrize("robot", ROBOTS)
def test_walks_forward_within_servo_limits(robot):
    out = simulate(GaitParams(robot=robot), 10)
    assert out["fell_at"] is None
    assert out["p99_leg_torque"] < lim(robot, "torque")
    assert out["max_cmd_vel"] < lim(robot, "cmd_vel")
    if SPEC[robot]["min_x"] is None:
        return  # only the reference for RL
    assert out["x"] > SPEC[robot]["min_x"]
    assert abs(out["y"]) < 0.1
    assert abs(out["yaw_deg"]) < SPEC[robot]["yaw_tol"]  # heading controlled with the IMU


@pytest.mark.parametrize("robot", WALKERS)
def test_turns_both_ways_within_servo_limits(robot):
    for turn, sign in [(0.2, 1), (-0.2, -1)]:  # rad/s requested from the heading control
        out = simulate(GaitParams(robot=robot, turn=turn), 10)
        assert out["fell_at"] is None
        assert sign * out["yaw_deg"] > 60
        assert out["p99_leg_torque"] < 1.3
        assert out["max_cmd_vel"] < 4.5


@pytest.mark.parametrize("robot,angle", [(r, a) for r in ROBOTS for a in SPEC[r]["ramps"]])
def test_climbs_ramps(robot, angle):
    from terrain import load_with_ramp, ramp_progress

    out = simulate(GaitParams(robot=robot), 20, load_fn=lambda r: load_with_ramp(angle, robot=r))
    assert out["fell_at"] is None
    assert ramp_progress(out) > 0.3
